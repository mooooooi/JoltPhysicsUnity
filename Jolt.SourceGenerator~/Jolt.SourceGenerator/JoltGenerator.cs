using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Jolt.SourceGenerator;

public struct Parameter
{
    public readonly string type;
    public readonly string name;

    public Parameter(IParameterSymbol symbol)
    {
        type = symbol.Type.ToDisplayString();
        name = symbol.Name;
    }
}

public readonly struct MethodDefine
{
    public readonly bool IsInstance;
    public readonly string Name;
    public readonly string Call;
    public readonly string ReturnType;
    public readonly ImmutableArray<Parameter> Parameters;

    public MethodDefine(bool isInstance, string name, IMethodSymbol symbol)
    {
        IsInstance = isInstance;
        Name = name;
        Call = $"{symbol.ContainingType.ToDisplayString()}.{symbol.Name}";
        ReturnType = symbol.ReturnType.ToDisplayString();
        Parameters = symbol.Parameters.Select(x => new Parameter(x)).ToImmutableArray();
    }

    public void Generate(SourceCodeScopeHelper helper)
    {
        var isCreateByReturn = false;
        var returnType = ReturnType;
        if (ReturnType.EndsWith("*") && ReturnType.StartsWith(Context.k_Prefix))
        {
            returnType = ReturnType.Substring(Context.k_Prefix.Length, ReturnType.Length - Context.k_Prefix.Length - 1);
            isCreateByReturn = true;
        }

        var parameters = IsInstance
            ? string.Join(", ", Parameters.Skip(1).Select(x => $"{x.type} {x.name}"))
            : string.Join(", ", Parameters.Select(x => $"{x.type} {x.name}"));
        var parametersNameOnly = IsInstance
            ? string.Join(", ", Parameters.Skip(1).Select(x => x.name))
            : string.Join(", ", Parameters.Select(x => x.name));
        var header = IsInstance
            ? $"public unsafe {returnType} {Name}({parameters})"
            : $"public unsafe static {returnType} {Name}({parameters})";

        using (var method = helper.Scope(header))
        {
            var dotSplit = IsInstance && !string.IsNullOrEmpty(parametersNameOnly) ? ", " : string.Empty;
            if (ReturnType == "void")
            {
                method.AppendLine($"{Call}({(IsInstance ? "*Ptr" : string.Empty)}{dotSplit}{parametersNameOnly});");
            }
            else
            {
                if (isCreateByReturn)
                {
                    method.AppendLine(
                        $"var value = {Call}({(IsInstance ? "*Ptr" : string.Empty)}{dotSplit}{parametersNameOnly});");
                    method.AppendLine(
                        $"var ptr = ({ReturnType}*)UnsafeUtility.MallocTracked(sizeof(void**), UnsafeUtility.AlignOf<IntPtr>(), Allocator.Persistent, 1);");
                    method.AppendLine($"*ptr = value;");
                    method.AppendLine($"return new {returnType}() {{ Ptr = ptr }};");
                }
                else
                {
                    method.AppendLine($"return {Call}({(IsInstance ? "*Ptr" : string.Empty)}{dotSplit}{parametersNameOnly});");
                }
            }
        }
    }
}

public readonly struct Struct(string typeName, ImmutableArray<MethodDefine> methods, bool isInstance)
{
    public readonly string TypeName = typeName;
    public readonly ImmutableArray<MethodDefine> Methods = methods;
    public readonly bool IsInstance = isInstance;

    public void Generate(SourceCodeScopeHelper helper)
    {
        var funcHeader = IsInstance
            ? $"[NativeContainer] public struct {TypeName}"
            : $"public static class {TypeName}";
        using (var cls = helper.Scope(funcHeader))
        {
            if (IsInstance)
            {
                cls.AppendLine($"[NativeDisableUnsafePtrRestriction] internal unsafe JPH_{TypeName}** Ptr;");
            }

            foreach (var methodDef in Methods)
            {
                cls.AppendLine($"//{methodDef.Name} -> {methodDef.Call}");
                methodDef.Generate(cls);
            }
        }
    }
}

public class Context : IEnumerable<Struct>
{
    public const string k_Prefix = "Jolt.JPH_";

    List<(bool isInstance, List<MethodDefine> methodDefines)> m_Entries = new();
    Dictionary<string, int> m_Type2Info = new();

    public int GetOrAddType(string typeName)
    {
        if (!m_Type2Info.TryGetValue(typeName, out var index))
        {
            index = m_Entries.Count;
            m_Type2Info[typeName] = index;
            m_Entries.Add((false, new List<MethodDefine>()));
        }

        return index;
    }

    public void AddMethodByTypeIndex(int typeIndex, MethodDefine methodDefine)
    {
        var methods = m_Entries[typeIndex].methodDefines;
        methods.Add(methodDefine);

        if (methodDefine.IsInstance)
        {
            MarkIsInstance(typeIndex);
        }
    }

    public void MarkIsInstance(int typeIndex)
    {
        var entry = m_Entries[typeIndex];
        entry.isInstance = true;
        m_Entries[typeIndex] = entry;
    }

    public IEnumerator<Struct> GetEnumerator()
    {
        foreach (var kv in m_Type2Info)
        {
            var info = kv.Value;
            var entry = m_Entries[info];
            var methods = entry.methodDefines;

            var isInstance = entry.isInstance | methods.Any(x => x.IsInstance || x.ReturnType.Contains($"{kv.Key}*"));
            yield return new Struct(kv.Key, methods.ToImmutableArray(), isInstance);
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

[Generator]
public class JoltGenerator : IIncrementalGenerator
{
    [ThreadStatic] public static StringBuilder m_StringBuilder;

    public void Initialize(IncrementalGeneratorInitializationContext context)
    {
        var nativeStructs = context.SyntaxProvider.CreateSyntaxProvider(
                static (x, token) => x is StructDeclarationSyntax s && s.Identifier.ValueText.StartsWith("JPH_"),
                static (x, token) =>
                {
                    var symbol = x.SemanticModel.GetDeclaredSymbol(x.Node);
                    return symbol?.ToDisplayString();
                }
            )
            .Where(x => x is not null)
            .Collect();

        var structs = context.SyntaxProvider.CreateSyntaxProvider
            (
                static (x, token) => x is ClassDeclarationSyntax { Identifier.ValueText: "UnsafeBindings" },
                static (x, token) => Parse(x, token)
            )
            .SelectMany((x, token) => x);

        context.RegisterSourceOutput(structs, (gctx, source) =>
        {
            var helper = new SourceCodeHelper(m_StringBuilder ??= new());
            helper.Using("System");
            helper.Using("Unity");
            helper.Using("Unity.Mathematics");
            helper.Using("Unity.Collections");
            helper.Using("Unity.Collections.LowLevel.Unsafe");
            try
            {
                using (var @namespace = helper.Scope("namespace Jolt"))
                {
                    source.Generate(@namespace);
                }

                gctx.AddSource($"{source.TypeName}.g.cs", m_StringBuilder.ToString());
            }
            catch (Exception e)
            {
                gctx.AddSource($"{source.TypeName}.g.cs", $"/* {e}*/");
            }
            finally
            {
                m_StringBuilder.Clear();
            }
        });
    }

    static readonly string[] k_Forbiddens = ["Quat", "Quaternion", "Vec3", "Matrix4x4", "RMatrix4x4"];

    static readonly Dictionary<string, string> k_Mappings = new Dictionary<string, string>()
    {
        { "ObjectVsBroadPhaseLayerFilterTable", "ObjectVsBroadPhaseLayerFilter" },
        { "ObjectLayerPairFilterMask", "ObjectLayerPairFilte" },
    };

    private static IEnumerable<Struct> Parse(GeneratorSyntaxContext ctx, CancellationToken token)
    {
        var symbol = ctx.SemanticModel.GetDeclaredSymbol(ctx.Node, token) as INamedTypeSymbol;
        if (symbol is null) yield break;

        var context = new Context();

        var methodSymbols = symbol.GetMembers().OfType<IMethodSymbol>();
        foreach (var method in methodSymbols)
        {
            var splits = method.Name.Split('_');

            // throw new Exception(string.Join("--", splits));
            string typeName;
            string methodName;

            if (splits.Length == 3)
            {
                typeName = splits[1];
                methodName = splits[2];
            }
            else if (splits.Length == 2)
            {
                typeName = "JotCore";
                methodName = splits[1];
            }
            else
            {
                throw new Exception("Unknown type");
            }

            if (k_Forbiddens.Contains(typeName)) continue;

            var typeIndex = context.GetOrAddType(typeName);
            var isInstance = method.Parameters.Length > 0 &&
                             method.Parameters[0].Type.ToDisplayString().Contains($"JPH_{typeName}*");
            var methodDef = new MethodDefine(isInstance, methodName, method);

            if (methodDef.ReturnType.StartsWith(Context.k_Prefix) && methodDef.ReturnType.EndsWith("*"))
            {
                var returnTypeName = methodDef.ReturnType.Substring(Context.k_Prefix.Length,
                    methodDef.ReturnType.Length - Context.k_Prefix.Length - 1);
                var returnTypeIndex = context.GetOrAddType(returnTypeName);
                context.MarkIsInstance(returnTypeIndex);
            }

            context.AddMethodByTypeIndex(typeIndex, methodDef);
        }

        foreach (var s in context)
        {
            yield return s;
        }
    }
}
