using System.Text;

namespace Jolt.SourceGenerator;

public ref struct SourceCodeHelper
{
    private StringBuilder m_StringBuilder;

    public SourceCodeHelper(StringBuilder sb)
    {
        m_StringBuilder = sb;
    }

    public void Comments(string chars) => m_StringBuilder.Append("//").Append(chars).AppendLine();

    public void Using(string @namespace) => m_StringBuilder.AppendFormat("using {0};", @namespace).AppendLine();

    public SourceCodeScopeHelper Scope(string header, string tail = null) => new(m_StringBuilder, header, tail, 0);
}

public ref struct SourceCodeLineHelper
{
    private StringBuilder m_StringBuilder;
    private int m_Depth;

    public SourceCodeLineHelper(StringBuilder sb, int depth)
    {
        m_StringBuilder = sb;
        m_Depth = depth;
    }

    public void AppendLine() => AppendLine(string.Empty);

    public void AppendLine(string x)
    {
        m_StringBuilder.AppendFormat("{0}{1}", m_Depth > 0 ? new string('\t', m_Depth) : string.Empty, x).AppendLine();
    }

    public void Append(string x)
    {
        if (m_StringBuilder.Length == 0 || m_StringBuilder[m_StringBuilder.Length - 1] is '\r' or '\n')
        {
            m_StringBuilder.Append(m_Depth > 0 ? new string('\t', m_Depth) : string.Empty);
        }

        m_StringBuilder.Append(x);
    }
}

public ref struct SourceCodeScopeHelper
{
    private StringBuilder m_StringBuilder;
    private int m_Depth;
    private string m_Tail;

    public bool IsValid => m_StringBuilder is not null;

    public SourceCodeScopeHelper(StringBuilder sb, string header, string tail, int depth)
    {
        m_StringBuilder = sb;
        m_Depth = depth;
        m_Tail = tail;

        if (header != null)
            Line().AppendLine(header);
        Line().AppendLine("{");
        m_Depth++;
    }

    public SourceCodeLineHelper Line() => new(m_StringBuilder, m_Depth);
    public void AppendLine(string x) => Line().AppendLine(x);
    public void AppendLine() => Line().AppendLine(string.Empty);
    public void Append(string x) => Line().Append(x);

    public SourceCodeScopeHelper Scope(string header = null, string tail = null) =>
        new(m_StringBuilder, header, tail, m_Depth);

    public void Dispose()
    {
        if (m_StringBuilder == null) return;
        m_Depth--;
        Line().AppendLine($"}}{m_Tail}");
        m_StringBuilder = null;
    }
}
