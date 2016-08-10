/*============================================================================
*
* -- Class:        SerialCom.cs
*
* -- Authur:       Engineerbeard
*
* -- Description:  Wrapper class for Windows APIs to use very basic functions
*                  to communicate over serial ports. Solves Sierra Wireless
*                  driver interaction issues with Microsoft SerialPort class.
*                  Also fixes massive memory leak in SerialPort class when
*                  communicating to Sierra Modems and unplugging.
*
* -- Usage:        VTISerialPort(115200, Ports.StopBits.One, Ports.Parity.None)
*
* ===========================================================================
*/

using System;
using System.Runtime.InteropServices;
using System.Threading;


namespace SharpSerial
{

   /// <summary>
   /// Class that allows for serial communication in Windows.
   /// </summary>
   ///

   public class VTISerialPort : IDisposable //Need to implement dispose?
   {
       private readonly int baudRate;
       private readonly byte byteSize;
       private bool isOpen = false;
       private string portName;
       private IntPtr handle = IntPtr.Zero;
       private IntPtr hEvent = IntPtr.Zero;
       private readonly System.IO.Ports.Parity parity;
       private readonly System.IO.Ports.StopBits stopBits;
       //private readonly System.IO.Ports.Handshake handShake;
       public event EventHandler<RecievedDataEventArgs> DataRecieved;
       NativeOverlapped lpOverlapped = new NativeOverlapped();

       Overlapped ol = new Overlapped();


       public System.IO.Ports.Parity Parity
       {
           get { return parity; }
           set { }
       }
       public bool IsOpen //Not Acutally implemented
       {
           get { return isOpen; }
           set { isOpen = value; }
       }
       public string PortName
       {
           get { return portName; }
           set { portName = value; }
       }
       public System.IO.Ports.StopBits StopBits
       {
           get { return stopBits; }
           set { }
       }


       public VTISerialPort(int baudRate, System.IO.Ports.StopBits stopBits, System.IO.Ports.Parity parity, byte byteSize)
       {

           if (stopBits == System.IO.Ports.StopBits.None)
               throw new ArgumentException("stopBits cannot be StopBits.None", "stopBits");
           if (byteSize < 5 || byteSize > 8)
               throw new ArgumentOutOfRangeException("The number of data bits must be 5 to 8 bits.", "byteSize");
           if (baudRate < 110 || baudRate > 256000)
               throw new ArgumentOutOfRangeException("Invalid baud rate specified.", "baudRate");
           if ((byteSize == 5 && stopBits == System.IO.Ports.StopBits.Two) || (stopBits == System.IO.Ports.StopBits.OnePointFive && byteSize > 5))
               throw new ArgumentException("The use of 5 data bits with 2 stop bits is an invalid combination as is 6, 7, or 8 data bits with 1.5 stop bits.");

           this.baudRate = baudRate;
           this.byteSize = byteSize;
           this.stopBits = stopBits;
           this.parity = parity;

           hEvent = CreateEvent(IntPtr.Zero , true, false,string.Empty );

           if (hEvent != IntPtr .Zero )
           {
               ol.hEvent  = hEvent;
           }
           //this.handShake = System.IO.Ports.Handshake.None;
       }

       public VTISerialPort(int baudRate, System.IO.Ports.StopBits stopBits, System.IO.Ports.Parity parity)
           : this(baudRate, stopBits, parity, 8)
       { }

       public void Dispose()
       {
           IsOpen = false;
           if (handle != IntPtr.Zero)
           {
               CloseHandle(handle);
               handle = IntPtr.Zero;
           }

       }
       public void DiscardInBuffer()
       {
           ReadExisting();
       }
       public bool Flush()
       {
           FailIfNotConnected("Fail flushing");

           const int PURGE_RXCLEAR = 0x0008; // input buffer
           const int PURGE_TXCLEAR = 0x0004; // output buffer
           return PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
       }

       public bool Open()
       {

           handle = CreateFile("//./" + portName, System.IO.FileAccess.ReadWrite, System.IO.FileShare.None, IntPtr.Zero, System.IO.FileMode.Open,FILE_FLAG_OVERLAPPED,  IntPtr.Zero);

           if (handle == IntPtr.Zero)
               return false;

           if (ConfigureSerialPort())
           {
               IsOpen = true; //Needs reworking
               return true;
           }
           else
           {
               Dispose();
               return false;
           }
       }

       public unsafe int Read(byte[] buffer, int index, int count)
       {
           int n = 0;
           uint lpNumberOfBytesWritten;
           fixed (byte* p = buffer)
           {


               ReadFile2(handle, p, buffer.Length, &n, ref lpOverlapped);
               GetOverlappedResult(handle, ref lpOverlapped, out lpNumberOfBytesWritten, true);
               //args.recData = System.Text.ASCIIEncoding.ASCII.GetString(buffer, 0, (int)lpNumberOfBytesWritten);

               //if (!ReadFile(handle, p + index, count, &n, 0))
                   //return 0;
           }
           return (int)lpNumberOfBytesWritten;
       }

       public string ReadString(int maxBytesToRead)
       {
           if (BytesToRead() < 1)
               return string.Empty;

           if (maxBytesToRead < 1)
               throw new ArgumentOutOfRangeException("maxBytesToRead < 1");

           byte[] bytes = new byte[maxBytesToRead];
           int numBytes = Read(bytes, 0, maxBytesToRead);
           string data = System.Text.ASCIIEncoding.ASCII.GetString(bytes, 0, numBytes);
           return data;
       }

       public byte readByte()
       {
           if (BytesToRead() < 1)
               throw new Exception("Error in readByte");

           byte[] bytes = new byte[1];
           int numBytes = Read(bytes, 0, 1);
           return bytes[0];
       }

       public string ReadExisting()
       {
           if (BytesToRead() < 1)
               return string.Empty;

           byte[] bytes = new byte[10000];
           int numBytes = Read(bytes, 0, 10000);
           return System.Text.ASCIIEncoding.ASCII.GetString(bytes, 0, numBytes);
       }

       public string ReadExistingBlocking()
       {
           byte[] bytes = new byte[10000];
           int numBytes = Read(bytes, 0, 10000);
           return System.Text.ASCIIEncoding.ASCII.GetString(bytes, 0, numBytes);
       }

       //public int writeByte(byte b)
       //{
       //    byte[] data = new byte[1];
       //    data[0] = b;
       //    FailIfNotConnected("write byte");
       //    if (data == null)
       //        return 0;

       //    int bytesWritten;
       //    if (WriteFile(handle, data, 1, out bytesWritten, 0))
       //        return bytesWritten;
       //    return -1;
       //}

       public int Write(byte[] data)
       {
           FailIfNotConnected("write");
           if (data == null)
               return 0;

           int bytesWritten;
           if (WriteFile2(handle, data, data.Length, out bytesWritten, ref ol))
               return bytesWritten;
           return -1;
       }

       public int Write(byte[] data, int offset, int count)
       {
           //_OVERLAPPED_ENTRY ol = new _OVERLAPPED_ENTRY ();
           FailIfNotConnected("write[]");
           if (data == null) return 0;

           int bytesWritten;

           if (WriteFile2(handle, data, count, out bytesWritten, ref ol))
               return bytesWritten;
           return -1;
       }

       public int Write(string data)
       {
           FailIfNotConnected("write string");

           byte[] bytes;
           if (data == null)
               bytes = null;
           else
               bytes = System.Text.ASCIIEncoding.ASCII.GetBytes(data);
           return Write(bytes);
       }

       public int WriteLine(string data)
       {
           if (data != null && !data.EndsWith("\r\n"))
               data += "\r\n";
           return Write(data);
       }

       public int BytesToRead()
       {
           int err = 0;
           COMSTAT comstat_struct = new COMSTAT();
           ClearCommError(handle, ref err, ref comstat_struct);

           return (int)comstat_struct.cbInQue;
       }



       public void OnDataRecieved(RecievedDataEventArgs e)
       {
           EventHandler<RecievedDataEventArgs> handler = DataRecieved;
           if (handler != null)
           {
               handler(this, e);
           }
       }

       public class RecievedDataEventArgs : EventArgs
       {
           public string recData { get; set; }
       }

       public unsafe void streamIn()
       {

           int dwBytesTransferred;
           uint lpNumberOfBytesWritten;
           uint dwCommModemStatus;
           byte[] buffer = new byte[1000];


           SetCommMask(handle, EV_RXCHAR | EV_CTS | EV_DSR | EV_RLSD | EV_RING);

           while (handle != INVALID_HANDLE_VALUE)
           {
               WaitCommEvent(handle, out dwCommModemStatus, IntPtr.Zero);
               SetCommMask(handle, EV_RXCHAR | EV_CTS | EV_DSR | EV_RING | EV_BREAK | EV_RLSD);


               if (dwCommModemStatus  == EV_RXCHAR)
               {
                   do
                   {
                       fixed (byte* p = buffer)
                       {
                           RecievedDataEventArgs args = new RecievedDataEventArgs();
                           ReadFile2(handle, p, buffer.Length, &dwBytesTransferred, ref lpOverlapped );
                           GetOverlappedResult(handle, ref lpOverlapped, out lpNumberOfBytesWritten, true);
                           args.recData = System.Text.ASCIIEncoding.ASCII.GetString(buffer, 0, (int)lpNumberOfBytesWritten);
                           OnDataRecieved(args);
                       }
                   }
                   while (dwBytesTransferred > 0);
               }
           }
       }

       System.Threading.Thread t;
       public void runStream()
       {
            t = new System.Threading.Thread(streamIn);
           t.Start();
       }

       public void endStream()
       {
           t.Abort();
           Dispose();
       }


       private bool ConfigureSerialPort()
       {
           DCB serialConfig = new DCB();

           if (GetCommState(handle, ref serialConfig))
           {
               serialConfig.BaudRate = (uint)this.baudRate;
               serialConfig.ByteSize = this.byteSize;
               serialConfig.fBinary = 1;           // must be true
               serialConfig.fDtrControl = 1;       // DTR_CONTROL_ENABLE "Enables the DTR line when the device is opened and leaves it on." changed from 1 to 0 per mark
               serialConfig.fAbortOnError = 0;     // false
               serialConfig.fTXContinueOnXoff = 0; // false
               serialConfig.fParity = 1;           // true so that the Parity member is looked at

               switch (this.Parity)
               {
                   case System.IO.Ports.Parity.Even:
                       serialConfig.Parity = 2;
                       break;
                   case System.IO.Ports.Parity.Mark:
                       serialConfig.Parity = 3;
                       break;
                   case System.IO.Ports.Parity.Odd:
                       serialConfig.Parity = 1;
                       break;
                   case System.IO.Ports.Parity.Space:
                       serialConfig.Parity = 4;
                       break;
                   case System.IO.Ports.Parity.None:
                   default:
                       serialConfig.Parity = 0;
                       break;
               }

               switch (this.StopBits)
               {
                   case System.IO.Ports.StopBits.One:
                       serialConfig.StopBits = 0;
                       break;
                   case System.IO.Ports.StopBits.OnePointFive:
                       serialConfig.StopBits = 1;
                       break;
                   case System.IO.Ports.StopBits.Two:
                       serialConfig.StopBits = 2;
                       break;
                   case System.IO.Ports.StopBits.None:
                   default:
                       throw new ArgumentException("stopBits cannot be StopBits.None");
               }

               if (SetCommState(handle, ref serialConfig))
               {
                   // set the serial connection timeouts
                   COMMTIMEOUTS timeouts = new COMMTIMEOUTS();
                   timeouts.ReadIntervalTimeout = 1; //Changed from 1 sean d.
                   timeouts.ReadTotalTimeoutMultiplier = 0;
                   timeouts.ReadTotalTimeoutConstant = 0;
                   timeouts.WriteTotalTimeoutMultiplier = 0;
                   timeouts.WriteTotalTimeoutConstant = 0;

                   if (SetCommTimeouts(handle, ref timeouts))
                       return true;
                   else
                       return false;
               }
               else
               {
                   return false;
               }
           }
           else
           {
               return false;
           }
       }

       // Helper that throws a InvalidOperationException if we don't have a serial connection.
       private void FailIfNotConnected(string err_source)
       {


           if (handle == IntPtr.Zero)
           {
               //Console.WriteLine(orig);
               Dispose();
               Open();
               throw new InvalidOperationException(string.Format("You must be connected to the serial port before performing this operation: {0}", err_source));
           }
       }



       #region Win API Calls

       const int EV_BREAK = 0x0040;    //A break was detected on input.
       const int EV_CTS = 0x0008;      //The CTS (clear-to-send) signal changed state.
       const int EV_DSR = 0x0010;      //The DSR (data-set-ready) signal changed state.
       const int EV_ERR = 0x0080;      //A line-status error occurred. Line-status errors are CE_FRAME, CE_OVERRUN, and CE_RXPARITY.
       const int EV_RING = 0x0100;     //A ring indicator was detected.
       const int EV_RLSD = 0x0020;     //The RLSD (receive-line-signal-detect) signal changed state.
       const int EV_RXCHAR = 0x0001;   //A character was received and placed in the input buffer.
       const int EV_RXFLAG = 0x0002;   //The event character was received and placed in the input buffer. The event character is specified in the device's DCB structure, which is applied to a serial port by using the SetCommState function.
       const int EV_TXEMPTY = 0x0004;  //The last character in the output buffer was sent.
       IntPtr  INVALID_HANDLE_VALUE = IntPtr.Zero;

       const int FILE_FLAG_OVERLAPPED = 0x40000000;



       // Used to get a handle to the serial port so that we can read/write to it.
       [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
       static extern IntPtr CreateFile
       (
           string fileName,
           [MarshalAs(UnmanagedType.U4)] System.IO.FileAccess fileAccess,
           [MarshalAs(UnmanagedType.U4)] System.IO.FileShare fileShare,
           IntPtr securityAttributes,
           [MarshalAs(UnmanagedType.U4)] System.IO.FileMode creationDisposition,
           int flags,
           IntPtr template
       );

       // Used to close the handle to the serial port.
       [DllImport("kernel32.dll", SetLastError = true)]
       static extern bool CloseHandle(IntPtr hObject);

       // Used to get the state of the serial port so that we can configure it.
       [DllImport("kernel32.dll")]
       static extern bool GetCommState(IntPtr hFile, ref DCB lpDCB);

       // Used to configure the serial port.
       [DllImport("kernel32.dll")]
       static extern bool SetCommState(IntPtr hFile, [In] ref DCB lpDCB);

       // Used to set the connection timeouts on our serial connection.
       [DllImport("kernel32.dll", SetLastError = true)]
       static extern bool SetCommTimeouts(IntPtr hFile, ref COMMTIMEOUTS lpCommTimeouts);

       // Used to read buffered data
       [System.Runtime.InteropServices.DllImport("kernel32", SetLastError = true)]
       static extern unsafe bool ReadFile(System.IntPtr hFile, void* pBuffer, int NumberOfBytesToRead, int* pNumberOfBytesRead, int Overlapped);

       // Used to read buffered data
       [System.Runtime.InteropServices.DllImport("kernel32", SetLastError = true,EntryPoint = "ReadFile")]
       static extern unsafe bool ReadFile2(System.IntPtr hFile, void* pBuffer, int NumberOfBytesToRead, int* pNumberOfBytesRead, [In] ref System.Threading.NativeOverlapped lpOverlapped);


       [DllImport("kernel32.dll")]
       static extern bool SetCommMask(IntPtr hFile, uint dwEvtMask);

       [DllImport("kernel32.dll")]
       static extern bool WaitCommEvent(IntPtr hFile, out uint lpEvtMask,IntPtr lpOverlapped);



       // Used to write bytes to the serial connection.
       [DllImport("kernel32.dll", SetLastError = true)]
       static extern bool WriteFile
       (
           IntPtr hFile,
           byte[] lpBuffer,
           int nNumberOfBytesToWrite,
           out int lpNumberOfBytesWritten,
           int lpOverlapped
       );

       // Used to write bytes to the serial connection.
       [DllImport("kernel32.dll", SetLastError = true,EntryPoint = "WriteFile")]
       static extern bool WriteFile2
       (
           IntPtr hFile,
           byte[] lpBuffer,
           int nNumberOfBytesToWrite,
           out int lpNumberOfBytesWritten,
           ref Overlapped flag
       );

       // Used to flush the I/O buffers.
       [DllImport("kernel32.dll", SetLastError = true)]
       static extern bool PurgeComm(IntPtr hFile, int dwFlags);

       // Used to populate the COMSTAT structure
       [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
       internal static extern bool ClearCommError(IntPtr hFile, ref int lpErrors, ref COMSTAT lpStat);

       // InBufferBytes and OutBufferBytes directly expose cbInQue and cbOutQue to reading, respectively.
       internal struct COMSTAT
       {
           public uint Flags;
           public uint cbInQue;
           public uint cbOutQue;
       }

       // Contains the time-out parameters for a communications device.
       [StructLayout(LayoutKind.Sequential)]
       struct COMMTIMEOUTS
       {
           public uint ReadIntervalTimeout;
           public uint ReadTotalTimeoutMultiplier;
           public uint ReadTotalTimeoutConstant;
           public uint WriteTotalTimeoutMultiplier;
           public uint WriteTotalTimeoutConstant;
       }

       // Defines the control setting for a serial communications device.
       [StructLayout(LayoutKind.Sequential)]
       struct DCB
       {
           public int DCBlength;
           public uint BaudRate;
           public uint Flags;
           public ushort wReserved;
           public ushort XonLim;
           public ushort XoffLim;
           public byte ByteSize;
           public byte Parity;
           public byte StopBits;
           public sbyte XonChar;
           public sbyte XoffChar;
           public sbyte ErrorChar;
           public sbyte EofChar;
           public sbyte EvtChar;
           public ushort wReserved1;
           public uint fBinary;
           public uint fParity;
           public uint fOutxCtsFlow;
           public uint fOutxDsrFlow;
           public uint fDtrControl;
           public uint fDsrSensitivity;
           public uint fTXContinueOnXoff;
           public uint fOutX;
           public uint fInX;
           public uint fErrorChar;
           public uint fNull;
           public uint fRtsControl;
           public uint fAbortOnError;
       }

       [StructLayout(LayoutKind.Sequential)]
       public struct Overlapped
       {
           public IntPtr intrnal;
           public IntPtr internalHigh;
           public int offset;
           public int offsetHigh;
           public IntPtr hEvent;
       }

       /*[StructLayout(LayoutKind.Sequential)]
       public class Overlapped
       {
           IntPtr intrnal;
           IntPtr internalHigh;
           int offset;
           int offsetHigh;
           IntPtr hEvent;
       }*/

       [DllImport("kernel32.dll")]
       static extern IntPtr CreateEvent(IntPtr lpEventAttributes, bool bManualReset, bool bInitialState, string lpName);

       [DllImport("kernel32.dll", SetLastError = true)]
       static extern bool GetOverlappedResult(IntPtr hFile,
          [In] ref System.Threading.NativeOverlapped lpOverlapped,
          out uint lpNumberOfBytesTransferred, bool bWait);

       /*typedef struct _OVERLAPPED_ENTRY {
 ULONG_PTR    lpCompletionKey;
 LPOVERLAPPED lpOverlapped;
 ULONG_PTR    Internal;
 DWORD        dwNumberOfBytesTransferred;
} OVERLAPPED_ENTRY, *LPOVERLAPPED_ENTRY;*/

       #endregion
   }


}
