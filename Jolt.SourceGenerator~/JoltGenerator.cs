using System;
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
        // throw new NotImplementedException();
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
}

public struct Struct(string typeName, ImmutableArray<Method> methods)
{
    public readonly string TypeName = typeName;
    public readonly ImmutableArray<Method> Methods = methods;

    public void Generate(SourceCodeScopeHelper helper)
    {
        foreach (var method in Methods)
        {
            helper.AppendLine($"//{method.Name} -> {method.Call}");
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
        var structs = context.SyntaxProvider.CreateSyntaxProvider
        (
            static (x, token) => x is ClassDeclarationSyntax { Identifier.ValueText: "UnsafeBindings" },
            static (x, token) => parse(x, token)
        )
        .SelectMany((x, token) => x);
        
        context.RegisterSourceOutput(structs, (gctx, source) =>
        {
            var helper = new SourceCodeHelper(m_StringBuilder ??= new());
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

    private static IEnumerable<Struct> parse(GeneratorSyntaxContext ctx, CancellationToken token)
    {
        var symbol = ctx.SemanticModel.GetDeclaredSymbol(ctx.Node, token) as INamedTypeSymbol;
        if (symbol is null) yield break;
        
        List<List<Method>> methodDefines = new();
        Dictionary<string, int> type2Index = new();

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
            
            if (!type2Index.TryGetValue(typeName, out var index))
            {
                index = methodDefines.Count;
                type2Index[typeName] = index;
                methodDefines.Add(new List<Method>());
            }
            
            var methods = methodDefines[index];
            methods.Add(new Method(methodName, method));
        }
        
        foreach (var kv in type2Index)
        {
            var methods =  methodDefines[kv.Value];
            
            yield return new Struct(kv.Key, methods.ToImmutableArray());
        }
    }
}
