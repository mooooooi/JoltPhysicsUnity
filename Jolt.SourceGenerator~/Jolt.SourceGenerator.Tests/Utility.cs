using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Xunit;

namespace Jolt.SourceGenerator.Tests;

public static class Utility
{
    public static (GeneratorDriver, CSharpCompilation) CreateDriver<TIncrementalGenerator>(string source, string assemblyName) where TIncrementalGenerator : IIncrementalGenerator, new()
    {
        var compilation = CSharpCompilation.Create(assemblyName,
            new[] { CSharpSyntaxTree.ParseText(source) },
            Basic.Reference.Assemblies.Net70.References.All,
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        var generator = new TIncrementalGenerator();
        var sourceGenerator = generator.AsSourceGenerator();

        // trackIncrementalGeneratorSteps allows to report info about each step of the generator
        GeneratorDriver driver = CSharpGeneratorDriver.Create(
            generators: [sourceGenerator],
            driverOptions: new GeneratorDriverOptions(default, trackIncrementalGeneratorSteps: true));

        // Run the generator
        driver = driver.RunGenerators(compilation);
        
        return (driver, compilation);
        // Assert the driver doesn't recompute the output
        // var result = driver.GetRunResult().Results.Single();
        // var allOutputs = result.TrackedOutputSteps.SelectMany(outputStep => outputStep.Value).SelectMany(output => output.Outputs);
        // Assert.Collection(allOutputs, output => Assert.Equal(IncrementalStepRunReason.Cached, output.Reason));
        //
        // foreach (var generatedSource in result.GeneratedSources)
        // {
        //     Console.WriteLine(generatedSource.SourceText);
        // }
        // Assert the driver use the cached result from AssemblyName and Syntax
        // var assemblyNameOutputs = result.TrackedSteps["AssemblyName"].Single().Outputs;
        // Assert.Collection(assemblyNameOutputs, output => Assert.Equal(IncrementalStepRunReason.Unchanged, output.Reason));
        //
        // var syntaxOutputs = result.TrackedSteps["Syntax"].Single().Outputs;
        // Assert.Collection(syntaxOutputs, output => Assert.Equal(IncrementalStepRunReason.Unchanged, output.Reason));
    }

    public static (GeneratorDriver, CSharpCompilation) AssertCache(GeneratorDriver driver,
        CSharpCompilation compilation, params (string stepName, IncrementalStepRunReason reason)[] validations)
    {
        compilation = compilation.AddSyntaxTrees(CSharpSyntaxTree.ParseText("// dummy"));
        driver = driver.RunGenerators(compilation);

        var result = driver.GetRunResult().Results.Single();
        
        var allOutputs = result.TrackedOutputSteps.SelectMany(outputStep => outputStep.Value).SelectMany(output => output.Outputs);
        Assert.Collection(allOutputs, output => Assert.Equal(IncrementalStepRunReason.Cached, output.Reason));

        foreach (var (stepName, reason) in validations)
        {
            var assemblyNameOutputs = result.TrackedSteps[stepName].Single().Outputs;
            Assert.Collection(assemblyNameOutputs, output => Assert.Equal(reason, output.Reason));
        }
        
        // var syntaxOutputs = result.TrackedSteps["Syntax"].Single().Outputs;
        // Assert.Collection(syntaxOutputs, output => Assert.Equal(IncrementalStepRunReason.Unchanged, output.Reason));
        
        return (driver, compilation);
    }

    public static (GeneratorDriver, CSharpCompilation) CreateDriverAndAssertCache<TIncrementalGenerator>(string source,
        string assemblyName, params (string stepName, IncrementalStepRunReason reason)[] validations) where TIncrementalGenerator : IIncrementalGenerator, new()
    {
        var (driver, compilation) = CreateDriver<TIncrementalGenerator>(source, assemblyName);
        return AssertCache(driver, compilation, validations);
    }
}
