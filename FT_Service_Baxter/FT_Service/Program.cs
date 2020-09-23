using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Runtime.InteropServices;
using RobotRaconteur;
using sensors.ati.mini45;
using System.Timers; 

namespace FT_Service
{
    class Program
    {
        static void Main(string[] args)
        {
            RobotRaconteurNativeLoader.Load();
            ATImini45Host_impl fthost;

            RobotRaconteurNode.s.RegisterServiceType(new sensors__ati__mini45Factory());
            RobotRaconteurNode.s.NodeName = "sensors.ati";

            TcpTransport c = new TcpTransport();
            c.StartServer(5300);
            RobotRaconteurNode.s.RegisterTransport(c);

            fthost = new ATImini45Host_impl();
            RobotRaconteurNode.s.RegisterService("ATImini45Host", "sensors.ati.mini45", fthost);

            // Initialize GripperHost
            Console.WriteLine("Initializing Sensors");
            Console.WriteLine("tcp://localhost:5300/sensors.ati/ATImini45Host");
            fthost.initialize();

            Console.WriteLine("Service started! - Press Enter to End");
            Console.ReadLine();

            fthost.shutdown();
            RobotRaconteurNode.s.Shutdown();
        }
    }

    public class ATImini45Host_impl : ATImini45Host
    {
        ATImini45_impl[] sensors;

        public void initialize()
        {
            // Set up two F/T sensors
            sensors = new ATImini45_impl[2];
            sensors[0] = new ATImini45_impl("192.168.1.175");
            sensors[1] = new ATImini45_impl("192.168.1.66");
            
        }

        public void shutdown()
        {
            sensors[0].shutdown();
            sensors[1].shutdown();
        }

        // Return a sensor reference
        public ATImini45 get_ft(int ind)
        {
			if (ind == 0 || ind == 1)
			{
				Console.WriteLine("Get_FT called");
				return sensors[ind];
			}
			else
				throw new Exception("Bad index");
        }
    }

    public class ATImini45_impl : ATImini45
    {
        private string address;
        private UdpClient client;
        private bool running;
        StreamWriter sw;
        Thread t;

        public ATImini45_impl(String address)
        {
            this.address = address;
            client = new UdpClient(address, 49152); //49152 is the F/T sensor port
            client.Client.ReceiveTimeout = 100;
            rd = 0;
            running = true;
            wr = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            t = new Thread(BackgroundExecThread);
            t.Start();
        }

        public void shutdown()
        {
            running = false;
            t.Join();
        }

        //Change the calibration configuration of the sensor
        //0 = low loads (more precise), 1 == high loads (less precise)
        public void changeCalibration(byte config)
        {
            if (config >= 0 && config <= 15)
            {
                HttpWebRequest request = (HttpWebRequest)WebRequest.Create("http://" + this.address + "/config.cgi?cfgid=15&cfgcalsel=" + config);
                request.Timeout = 5000; //5 Second Timeout
                HttpWebResponse response = (HttpWebResponse)request.GetResponse(); //Execute response
                response.Close();
                return;
            }
            else
                throw new Exception("Invalid calibration index!");
        }

        public void bias()
        {
            Byte[] datagram = new Byte[8];

            // Spec calls for this specific header
            UInt16 command_header = 0x3412;
            UInt16 command = 0x4200; // Set software bias
            UInt32 sample_count = 0x01000000;

            //Populate the outgoing datagram. See the F/T manual for more info
            Array.Copy(BitConverter.GetBytes(command_header), 0, datagram, 0, 2);
            Array.Copy(BitConverter.GetBytes(command), 0, datagram, 2, 2);
            Array.Copy(BitConverter.GetBytes(sample_count), 0, datagram, 4, 4);
            client.Send(datagram, 8); // Send UDP packet

        }

        private double[] wr;
        public double[] wrench
        {
            set
            { }
            get
            {
                return wr;
            }
        }

        private byte rd;
        public byte recording_data
        {
            set { }
            get
            {
                return rd;
            }
        }


        public void startRecordingData(string filename)
        {
            if (rd == 1)
                return;
			
			//sw = new StreamWriter(filename + ".csv");
            QueryPerformanceCounter(out record_start_time);
            rd = 1;
        }

        public void stopRecordingData()
        {
            if (rd == 0)
                return;

            rd = 0;
            Thread.Sleep(50);
            //sw.Close();

        }

        long record_start_time;
		private Pipe<FTData> _FTDataStream = null;
		private PipeBroadcaster<FTData> _FTDataStreamBroadcaster;

		//Property for the FrameStream pipe
		public Pipe<FTData> FTDataStream
		{
			get
			{
				return _FTDataStream;
			}
			set
			{
				_FTDataStream = value;
				_FTDataStreamBroadcaster = new PipeBroadcaster<FTData>(_FTDataStream, 3);

			}
		}
		void BackgroundExecThread()
        {
            int force_scale = 1000000; // Divider scale for newtons
            int torque_scale = 1000000; // Divider scale for newton-meters
            // Spec calls for this specific header
            UInt16 command_header = 0x3412;
            UInt16 command = 0x0200; // Request a single measurement
            UInt32 sample_count = 0x01000000;
            Byte[] datagram = new Byte[8];
            Byte[] received = new Byte[32];
            
            //Populate the outgoing datagram. See the F/T manual for more info
            Array.Copy(BitConverter.GetBytes(command_header), 0, datagram, 0, 2);
            Array.Copy(BitConverter.GetBytes(command), 0, datagram, 2, 2);
            Array.Copy(BitConverter.GetBytes(sample_count), 0, datagram, 4, 4);
            
            IPEndPoint ep = null;

            long ticksPerSec;
            long loop_start_time;
            long loop_time;

            QueryPerformanceFrequency(out ticksPerSec);

            while (running)
            {

                QueryPerformanceCounter(out loop_start_time);
                client.Send(datagram, 8); // Send UDP packet

                //Receive and unpack data
                try
                {
                    received = client.Receive(ref ep);
                    Array.Reverse(received);
                    lock (wr)
                    {
                        wr[0] = (double)(BitConverter.ToInt32(received, 20)) / force_scale;
                        wr[1] = (double)(BitConverter.ToInt32(received, 16)) / force_scale;
                        wr[2] = (double)(BitConverter.ToInt32(received, 12)) / force_scale;
                        wr[3] = (double)(BitConverter.ToInt32(received, 8)) / torque_scale;
                        wr[4] = (double)(BitConverter.ToInt32(received, 4)) / torque_scale;
                        wr[5] = (double)(BitConverter.ToInt32(received, 0)) / torque_scale;
                    }
                }
                catch (Exception e)
                {
                    //Console.WriteLine(e.Message);
                }

                if (recording_data == 1)
                {
					FTData o = new FTData();
					o.time = (loop_start_time - record_start_time);
					o.ft_data = wr;
					try
					{
						_FTDataStreamBroadcaster.AsyncSendPacket(o, () => { });
					}
					catch { }
					//string wr_line = "";
					//wr_line += (loop_start_time - record_start_time) / (ticksPerSec / 1000);

                    //for (int n = 0; n < 6; n++)
                    //    wr_line += ", " + wr[n];
                    //sw.WriteLine(wr_line);
                }
                

                QueryPerformanceCounter(out loop_time);
                long tMillisec = (loop_time - loop_start_time) / (ticksPerSec / 1000);
                while (tMillisec < 10)
                {
                    QueryPerformanceCounter(out loop_time);
                    tMillisec = (loop_time - loop_start_time) / (ticksPerSec / 1000);
                }
            }
        }

        [DllImport("Kernel32.dll")]
        private static extern bool QueryPerformanceCounter(out long lpPerformanceCount);

        [DllImport("Kernel32.dll")]
        private static extern bool QueryPerformanceFrequency(out long lpFrequency);
    }
}
