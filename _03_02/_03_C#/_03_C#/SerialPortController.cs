using System.IO.Ports;

namespace _03_C_
{
    public delegate void OnReadDelegate(string data);

    public class SerialPortController
    {
        private int _defaultBaudRate = 115200;
        private int _defaultDataBits = 8;
        private static bool _continue = false;
        private Thread _readThread;
        private static SerialPort _serialPort;
        private static OnReadDelegate _dataRead;

        public SerialPortController(OnReadDelegate onRead)
        {
            _dataRead = onRead;
            _serialPort = new SerialPort(GetPortName(),  _defaultBaudRate, Parity.None, _defaultDataBits, StopBits.One);
        }

        private string GetPortName()
        {
            string[] portNames = SerialPort.GetPortNames();
            return portNames[0];
        }


        public void Start()
        {
            _serialPort.Open();
            _continue = true;
            _readThread = new Thread(Read);
            _readThread.Start();
        }

        [STAThread]
        private static void Read()
        {
            while (_continue)
            {
                string data = _serialPort.ReadLine();
                _dataRead.Invoke(data);
            }
        }

        public void Stop()
        {
            _continue = false;
            _readThread.Join();
            _serialPort.Close();
        }
    }
}
