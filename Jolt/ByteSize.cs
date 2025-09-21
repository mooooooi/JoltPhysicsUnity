using System;

namespace Jolt
{
    public readonly struct ByteSize : IFormattable
    {
        private readonly long _bytes;

        public ByteSize(long bytes)
        {
            _bytes = bytes;
        }

        public static implicit operator ByteSize(long bytes) => new ByteSize(bytes);
    
        public override string ToString() => ToString(null, null);
    
        public string ToString(string format, IFormatProvider formatProvider)
        {
            string[] suffixes = { "B", "KB", "MB", "GB", "TB" };
            int counter = 0;
            double number = _bytes;
        
            while (number >= 1024 && counter < suffixes.Length - 1)
            {
                counter++;
                number /= 1024;
            }
        
            // 根据值的大小决定小数位数
            string numberFormat;
            if (counter == 0) // B
                numberFormat = "0";
            else if (number < 10) // 小于10的值显示2位小数
                numberFormat = "0.##";
            else if (number < 100) // 小于100的值显示1位小数
                numberFormat = "0.#";
            else // 大于等于100的值显示整数
                numberFormat = "0";
        
            return $"{number.ToString(numberFormat)} {suffixes[counter]}";
        }
    }
}
