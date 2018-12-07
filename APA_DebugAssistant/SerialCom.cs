using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace APA_DebugAssistant
{
    class SerialCom
    {
        private bool closing;
        private bool listening;
        private UInt64 err_count;
        private string[] m_baudRate = new string[11] { "4800", "9600", "14400", "19200", "38400", "76800", "115200", "194000", "460800", "921600", "1843200" };
        private List<byte> buffer = new List<byte>(4096);//默认分配1页内存，并始终限制不允许超过
        private byte[] binary_data = new byte[64];//AA 55 08 01 02 03 04 05 06 07 08 EAH EAL 
        

        public bool Closing
        {
            get
            {
                return closing;
            }
            set
            {
                closing = value;
            }
        }
        public bool Listening
        {
            get
            {
                return listening;
            }
            set
            {
                listening = value;
            }
        }
        public List<byte> Buffer
        {
            get
            {
                return buffer;
            }
            set
            {
                buffer = value;
            }
        }
        public byte[] BinaryData
        {
            get
            {
                return binary_data;
            }
            set
            {
                binary_data = value;
            }
        }
        public UInt64 ErrCount
        {
            get
            {
                return err_count;
            }
            set
            {
                err_count = value;
            }
        }


        public SerialCom()
        {
            closing = false;
            listening = false;
            err_count = 0;
        }
        
        public void AddBaudRate(ComboBox BaudRateComBox)
        {
            for(int i=0;i<11;i++)
            {
                BaudRateComBox.Items.Add(m_baudRate[i]);
            }
            BaudRateComBox.SelectedIndex = 4;
        }

        /// <summary>
        /// 查询可用的串口号
        /// </summary>
        /// <param name="MyPort"> the Port </param>
        /// <param name="MyBox"> </param>
        public void SearchAndAddSerialToComboBox(SerialPort MyPort, ComboBox MyBox)
        {                                                               //将可用端口号添加到ComboBox                         
            string Buffer;                                              //缓存
            bool ComExist = false;
            MyBox.Items.Clear();                                        //清空ComboBox内容
            for (int i = 1; i < 20; i++)                                //循环
            {
                try                                                     //核心原理是依靠try和catch完成遍历
                {
                    Buffer = "COM" + i.ToString();
                    MyPort.PortName = Buffer;
                    MyPort.Open();                                      //如果失败，后面的代码不会执行                    
                    MyBox.Items.Add(Buffer);                            //打开成功，添加至下俩列表
                    MyPort.Close();                                     //关闭
                    ComExist = true;
                }
                catch
                {

                }
            }
            if(ComExist)
            {
                MyBox.SelectedIndex = 0;
            }
        }

        /// <summary>
        /// 进行串口的打开和关闭操作
        /// </summary>
        /// <param name="MyPort"></param>
        /// <param name="bt"></param>
        /// <param name="port"></param>
        /// <param name="BaudRate"></param>
        public void OpenAndCloseSerial(SerialPort MyPort,Button bt,string port, int BaudRate)
        {
            try
            {
                if (MyPort.IsOpen)//端口处于打开状态，我可以进行端口关闭操作
                {
                    closing = true;
                    while (Listening) Application.DoEvents();
                    //打开时点击，则关闭串口
                    MyPort.Close();
                    closing = false;
                    bt.Text = "串口关闭";
                }
                else
                {
                    MyPort.PortName = port;
                    MyPort.BaudRate = BaudRate;
                    MyPort.DataBits = 8;
                    MyPort.StopBits = StopBits.One;
                    MyPort.Parity = Parity.None;
                    try
                    {
                        MyPort.Open();
                        bt.Text = "串口打开";
                    }
                    catch (Exception ex)
                    {
                        //捕获到异常信息，创建一个新的comm对象，之前的不能用了。  
                        //serialPort1 = new SerialPort();
                        //现实异常信息给客户。  
                        MessageBox.Show(ex.Message);
                    }
                }
            }
            catch
            {
                MessageBox.Show("端口错误，请检查端口选择是否正确", "错误提示");
            }
        }
    }
}
