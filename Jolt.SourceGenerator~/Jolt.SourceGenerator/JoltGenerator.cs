using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Jolt.SourceGenerator;

public struct TempStructTypeInfo
{
    public int Index;
    public bool IsInstance;
}

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

public struct Method
{
    public readonly string Name;
    public readonly string Call;
    public readonly string ReturnType;
    public readonly ImmutableArray<Parameter> Parameters;

    public Method(string name, IMethodSymbol symbol)
    {
        Name = name;
        Call = $"{symbol.ContainingType.ToDisplayString()}.{symbol.Name}";
        ReturnType = $"{symbol.ReturnType.ToDisplayString()}";
        Parameters = symbol.Parameters.Select(x => new Parameter(x)).ToImmutableArray();
    }

    public void Generate(SourceCodeScopeHelper helper)
    {
        var parameters = string.Join(",", Parameters.Select(x => $"{x.type} {x.name}"));
        var header = $"public unsafe static {ReturnType} {Name}({parameters})";
        using (var method = helper.Scope(header))
        {
            var parametersNameOnly = string.Join(",", Parameters.Select(x => x.name));
            if (ReturnType == "void")
            {
                method.AppendLine($"{Call}({parametersNameOnly});");
            }
            else
            {
                method.AppendLine($"return {Call}({parametersNameOnly});");
            }
        }
    }

    public void GenerateInstance(SourceCodeScopeHelper helper)
    {
        var parameters = string.Join(",", Parameters.Select(x => $"{x.type} {x.name}"));
        var header = $"public unsafe static {ReturnType} {Name}({parameters})";
        using (var method = helper.Scope(header))
        {
            var parametersNameOnly = string.Join(",", Parameters.Select(x => x.name));
            if (ReturnType == "void")
            {
                method.AppendLine($"{Call}({parametersNameOnly});");
            }
            else
            {
                method.AppendLine($"return {Call}({parametersNameOnly});");
            }
        }
    }
}

public struct Struct(bool isInstance, string typeName, ImmutableArray<Method> methods)
{
    public readonly bool IsInstance = isInstance;
    public readonly string TypeName = typeName;
    public readonly ImmutableArray<Method> Methods = methods;

    public void Generate(SourceCodeScopeHelper helper)
    {
        if (IsInstance)
        {
            using (var cls = helper.Scope($"public struct {TypeName}"))
            {
                cls.AppendLine($"internal unsafe JPH_{TypeName}* Ptr;");
                foreach (var methodDef in Methods)
                {
                    cls.AppendLine($"//{methodDef.Name} -> {methodDef.Call}");
                    methodDef.GenerateInstance(cls);
                }
            }
        }
        else
        {
            using (var cls = helper.Scope($"public static class {TypeName}"))
            {
                foreach (var methodDef in Methods)
                {
                    cls.AppendLine($"//{methodDef.Name} -> {methodDef.Call}");
                    methodDef.Generate(cls);
                }
            }
        }
    }
}

[Generator]
public class JoltGenerator : IIncrementalGenerator
{
    [ThreadStatic]
    public static StringBuilder m_StringBuilder;
    
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
            helper.Using("Unity");
            helper.Using("Unity.Mathematics");
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

    static readonly string[] k_Forbiddens = ["Quat", "Quaternion", "Vec3",  "Matrix4x4", "RMatrix4x4"];
    static readonly Dictionary<string, string> k_Mappings = new Dictionary<string, string>()
    {
        { "ObjectVsBroadPhaseLayerFilterTable", "ObjectVsBroadPhaseLayerFilter" },
        { "ObjectLayerPairFilterMask", "ObjectLayerPairFilte" },
    };
    private static IEnumerable<Struct> Parse(GeneratorSyntaxContext ctx, CancellationToken token)
    {
        var symbol = ctx.SemanticModel.GetDeclaredSymbol(ctx.Node, token) as INamedTypeSymbol;
        if (symbol is null) yield break;
        
        List<List<Method>> methodDefines = new();
        Dictionary<string, TempStructTypeInfo> type2Info = new();

        var methodSymbols = symbol.GetMembers().OfType<IMethodSymbol>();
        foreach (var method in methodSymbols)
        {
            var splits = method.Name.Split('_');

            // throw new Exception(string.Join("--", splits));
            string typeName;
            string methodName;
            bool isInstance = false;
            
            if (splits.Length == 3)
            {
                typeName = splits[1];
                methodName = splits[2];
                isInstance = true;
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
            
            if (!type2Info.TryGetValue(typeName, out var tempInfo))
            {
                var index = methodDefines.Count;
                type2Info[typeName] = tempInfo = new TempStructTypeInfo()
                {
                    Index = index, IsInstance = isInstance
                };
                methodDefines.Add(new List<Method>());
            }
            
            var methods = methodDefines[tempInfo.Index];
            methods.Add(new Method(methodName, method));
        }
        
        foreach (var kv in type2Info)
        {
            var info = kv.Value;
            var methods =  methodDefines[info.Index];
            
            yield return new Struct(info.IsInstance, kv.Key, methods.ToImmutableArray());
        }
    }
}
