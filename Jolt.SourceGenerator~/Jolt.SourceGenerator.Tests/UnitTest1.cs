using Microsoft.CodeAnalysis;
using Xunit;
using Xunit.Abstractions;

namespace Jolt.SourceGenerator.Tests;

public class Tests(ITestOutputHelper outputHelper)
{
    private ITestOutputHelper m_OutputHelper = outputHelper;
    [Fact]
    public void Test1()
    {
        (string, IncrementalStepRunReason Unchanged)[] validations =
        [
            ("TypeMeta", IncrementalStepRunReason.Unchanged)
        ];
        
        var (driver, _) = Utility.CreateDriver<JoltGenerator>(
            File.ReadAllText("../../../UnsafeBindings.txt"), "Jolt"/*, validations*/);
        
        // 检查缓存
        var ret = driver.GetRunResult();
        var generateRet = ret.Results.Single();

        Assert.Null(generateRet.Exception);
        
        m_OutputHelper.WriteLine("----------------Source----------------");
        foreach (var source in generateRet.GeneratedSources)
        {
            m_OutputHelper.WriteLine(source.SourceText.ToString());
        }
    }
}
