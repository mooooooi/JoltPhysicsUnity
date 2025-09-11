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
    public readonly string nativeType;
    public readonly string name;

    public Parameter(IParameterSymbol symbol)
    {
        type = symbol.Type.ToDisplayString();
        nativeType = symbol.GetAttributes().FirstOrDefault(x => x.AttributeClass?.Name == "NativeTypeNameAttribute")?.ConstructorArguments.FirstOrDefault().Value?.ToString();
        name = symbol.Name;
    }

    public string AsDefinition
    {
        get
        {
            if (nativeType == "bool" && type == "byte")
            {
                return $"{nativeType} {name}";
            }
            return $"{type} {name}";
        }
    }

    public string AsCaller
    {
        get
        {
            if (nativeType == "bool" && type == "byte")
            {
                return $"{name} ? (byte)1 : (byte)0";
            }
            return name;
        }
    }
}

public readonly struct MethodDefinition
{
    public readonly bool IsInstance;
    public readonly string Name;
    public readonly string Call;
    public readonly string ReturnType;
    public readonly string NativeReturnType;
    public readonly ImmutableArray<Parameter> Parameters;

    public MethodDefinition(bool isInstance, string name, IMethodSymbol symbol)
    {
        IsInstance = isInstance;
        Name = name;
        Call = $"{symbol.ContainingType.ToDisplayString()}.{symbol.Name}";
        ReturnType = symbol.ReturnType.ToDisplayString();
        Parameters = symbol.Parameters.Select(x => new Parameter(x)).ToImmutableArray();
        NativeReturnType = symbol.GetReturnTypeAttributes()
            .FirstOrDefault(x => x.AttributeClass?.Name == "NativeTypeNameAttribute")
            ?.ConstructorArguments[0].Value?.ToString();
    }

    public void Generate(SourceCodeScopeHelper helper, bool isSuper)
    {
        var isCreateByReturn = false;
        var isDisposing = Name == "Destroy";
        
        var returnType = ReturnType;
        string returnExpression = null;
        
        if (returnType.EndsWith("*") && returnType.StartsWith(Context.k_Prefix))
        {
            returnType = returnType.Substring(Context.k_Prefix.Length, returnType.Length - Context.k_Prefix.Length - 1);
            isCreateByReturn = true;
        }
        else if (NativeReturnType == "bool" && returnType == "byte")
        {
            returnType = NativeReturnType;
            returnExpression = "return returnValue != 0;";
        }

        var parameters = IsInstance
            ? string.Join(", ", Parameters.Skip(1).Select(x => x.AsDefinition))
            : string.Join(", ", Parameters.Select(x => x.AsDefinition));
        var parametersNameOnly = IsInstance
            ? string.Join(", ", Parameters.Skip(1).Select(x => x.AsCaller))
            : string.Join(", ", Parameters.Select(x => x.AsCaller));
        var header = IsInstance
            ? $"public unsafe {returnType} {Name}({parameters})"
            : $"public unsafe static {returnType} {Name}({parameters})";

        using (var method = helper.Scope(header))
        {
            var ptrExpression = string.Empty;
            if (IsInstance && !isSuper)
            {
                ptrExpression = "*Ptr";
            }
            else if (IsInstance && isSuper)
            {
                ptrExpression = $"({Parameters[0].type})*Ptr";
            }
            var dotSplit = IsInstance && !string.IsNullOrEmpty(parametersNameOnly) ? ", " : string.Empty;
            
            if (ReturnType == "void")
            {
                method.AppendLine($"{Call}({(ptrExpression)}{dotSplit}{parametersNameOnly});");
            }
            else
            {
                method.AppendLine(
                    $"var returnValue = {Call}({ptrExpression}{dotSplit}{parametersNameOnly});");
                if (isCreateByReturn)
                {

                    method.AppendLine(
                        $"var ptr = ({ReturnType}*)UnsafeUtility.MallocTracked(sizeof(void**), UnsafeUtility.AlignOf<IntPtr>(), Allocator.Persistent, 1);");
                    method.AppendLine($"*ptr = returnValue;");
                    method.AppendLine($"return new {returnType}() {{ Ptr = ptr }};");
                }
                else
                {
                    method.AppendLine(returnExpression ?? $"return returnValue;");
                }
            }
        }
    }
}

public readonly struct StructDefinition(string typeName, ImmutableArray<MethodDefinition> methods, bool isInstance)
{
    public readonly string TypeName = typeName;
    public readonly ImmutableArray<MethodDefinition> Methods = methods;
    public readonly bool IsInstance = isInstance;

    public void Generate(SourceCodeScopeHelper helper, string superTypeName = null)
    {
        var isSuper = !string.IsNullOrEmpty(superTypeName);
        
        if (IsInstance && !isSuper)
            helper.AppendLine("[NativeContainer]");
        var funcHeader = IsInstance
            ? $"public partial struct {superTypeName ?? TypeName}"
            : $"public static class {TypeName}";
        using (var cls = helper.Scope(funcHeader))
        {
            if (IsInstance && !isSuper)
            {
                cls.AppendLine("[NativeDisableUnsafePtrRestriction]");
                cls.AppendLine($"internal unsafe JPH_{TypeName}** Ptr;");
                cls.AppendLine();
                
                cls.AppendLine($"public unsafe bool IsCreated => Ptr != null;");
                
                using (var func = cls.Scope($"public unsafe JPH_{TypeName}* ToUnsafePtr()"))
                {
                    func.AppendLine($"return *Ptr;");
                }
            }

            if (IsInstance && isSuper)
            {
                cls.AppendLine($"public unsafe {TypeName} As{TypeName} => new {TypeName}() {{ Ptr = (JPH_{TypeName}**)Ptr }};");
            }

            foreach (var methodDef in Methods)
            {
                if (!methodDef.IsInstance && isSuper) continue;
                cls.AppendLine($"//{methodDef.Name} -> {methodDef.Call}");
                methodDef.Generate(cls, isSuper);
            }
        }
    }
}

public class Context : IEnumerable<StructDefinition>
{
    public const string k_Prefix = "Jolt.JPH_";

    List<(bool isInstance, List<MethodDefinition> methodDefines)> m_Entries = new();
    Dictionary<string, int> m_Type2Info = new();

    public int GetOrAddType(string typeName)
    {
        if (!m_Type2Info.TryGetValue(typeName, out var index))
        {
            index = m_Entries.Count;
            m_Type2Info[typeName] = index;
            m_Entries.Add((false, new List<MethodDefinition>()));
        }

        return index;
    }

    public void AddMethodByTypeIndex(int typeIndex, MethodDefinition methodDefinition)
    {
        var methods = m_Entries[typeIndex].methodDefines;
        methods.Add(methodDefinition);

        if (methodDefinition.IsInstance)
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

    public IEnumerator<StructDefinition> GetEnumerator()
    {
        foreach (var kv in m_Type2Info)
        {
            var info = kv.Value;
            var entry = m_Entries[info];
            var methods = entry.methodDefines;

            var isInstance = entry.isInstance | methods.Any(x => x.IsInstance || x.ReturnType.Contains($"{kv.Key}*"));
            yield return new StructDefinition(kv.Key, methods.ToImmutableArray(), isInstance);
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

        context.RegisterSourceOutput(structs.Collect(), (gctx, sources) =>
        {
            var sourceLookup = sources.ToDictionary(x => x.TypeName);
            foreach (var source in sources)
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
                        source.Generate(@namespace, null);

                        var typeName = source.TypeName;
                        while (k_Extends.TryGetValue(typeName, out var superTypeName))
                        {
                            if (sourceLookup.TryGetValue(superTypeName, out var superStruct))
                            {
                                @namespace.AppendLine($"//{typeName} extends {superTypeName}");
                                superStruct.Generate(@namespace, source.TypeName);
                            }

                            typeName = superTypeName;
                        }
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
            }
        });
    }

    public static readonly string[] k_Forbiddens = ["Quat", "Quaternion", "Vec3", "Matrix4x4", "RMatrix4x4"];

    public static readonly Dictionary<string, string> k_Mappings = new()
    {
        { "ObjectLayerPairFilterTable", "ObjectLayerPairFilter" },
        { "ObjectLayerPairFilterMask", "ObjectLayerPairFilter" },        
        
        { "ObjectVsBroadPhaseLayerFilterTable", "ObjectVsBroadPhaseLayerFilter" },
        { "ObjectVsBroadPhaseLayerFilterMask", "ObjectVsBroadPhaseLayerFilter" },               
        
        { "BroadPhaseLayerInterfaceTable", "BroadPhaseLayerInterface" },
        { "BroadPhaseLayerInterfaceMask", "BroadPhaseLayerInterface" },        
    };

    public static readonly Dictionary<string, string> k_Extends = new Dictionary<string, string>()
    {
        { "ConvexShape", "Shape" },
        { "ConvexShapeSettings", "ShapeSettings" },
        { "BoxShape", "ConvexShape" },
        { "BoxShapeSettings", "ConvexShapeSettings" },
        { "SphereShape", "ConvexShape" },
        { "SphereShapeSettings", "ConvexShapeSettings" },
        { "PlaneShape", "Shape" },
        { "PlaneShapeSettings", "ShapeSettings" },
        { "TriangleShape", "ConvexShape" },
        { "TriangleShapeSettings", "ConvexShapeSettings" },
        { "CapsuleShape", "ConvexShape" },
        { "CapsuleShapeSettings", "ConvexShapeSettings" },
        { "CylinderShape", "ConvexShape" },
        { "CylinderShapeSettings", "ConvexShapeSettings" },
        { "TaperedCylinderShape", "ConvexShape" },
        { "TaperedCylinderShapeSettings", "ConvexShapeSettings" },
        { "ConvexHullShape", "ConvexShape" },
        { "ConvexHullShapeSettings", "ConvexShapeSettings" },
        { "MeshShape", "Shape" },
        { "MeshShapeSettings", "ShapeSettings" },
        { "HeightFieldShape", "Shape" },
        { "HeightFieldShapeSettings", "ShapeSettings" },
        { "TaperedCapsuleShape", "ConvexShape" },
        { "TaperedCapsuleShapeSettings", "ConvexShapeSettings" },
        { "CompoundShape", "Shape" },
        { "CompoundShapeSettings", "ShapeSettings" },
        { "StaticCompoundShape", "CompoundShape" },
        { "StaticCompoundShapeSettings", "CompoundShapeSettings" },
        { "MutableCompoundShape", "CompoundShape" },
        { "MutableCompoundShapeSettings", "CompoundShapeSettings" },
        { "DecoratedShape", "Shape" },
        { "DecoratedShapeSettings", "ShapeSettings" },
        { "RotatedTranslatedShape", "DecoratedShape" },
        { "RotatedTranslatedShapeSettings", "DecoratedShapeSettings" },
        { "ScaledShape", "DecoratedShape" },
        { "ScaledShapeSettings", "DecoratedShapeSettings" },
        { "OffsetCenterOfMassShape", "DecoratedShape" },
        { "OffsetCenterOfMassShapeSettings", "DecoratedShapeSettings" },
        { "EmptyShape", "Shape" },
        { "EmptyShapeSettings", "ShapeSettings" },
    };

    private static IEnumerable<StructDefinition> Parse(GeneratorSyntaxContext ctx, CancellationToken token)
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
                typeName = "JoltCore";
                methodName = splits[1];
            }
            else
            {
                throw new Exception("Unknown type");
            }

            if (k_Forbiddens.Contains(typeName)) continue;
            typeName = k_Mappings.TryGetValue(typeName, out var replaced) ? replaced : typeName;

            var typeIndex = context.GetOrAddType(typeName);
            var isInstance = method.Parameters.Length > 0 &&
                             method.Parameters[0].Type.ToDisplayString().Contains($"JPH_{typeName}*");
            var methodDef = new MethodDefinition(isInstance, methodName, method);

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
