using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace _03_C_
{
	public class EntryPoint
	{
		public static void Main(string[] args)
		{
			SerialPortController spc = new SerialPortController(HandleRead);
			spc.Start();
			Console.ReadLine();
			spc.Stop();
		}

		public static void HandleRead (string content)
		{
			Console.WriteLine(content);
		}
	}
}
