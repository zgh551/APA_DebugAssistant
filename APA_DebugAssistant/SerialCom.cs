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
        bool data_catched = false;//缓存记录数据是否捕获到  
        private UInt64 err_count;
        private string[] m_baudRate = new string[11] { "4800", "9600", "14400", "19200", "38400", "76800", "115200", "194000", "460800", "921600", "1843200" };
        private List<byte> buffer = new List<byte>(4096);//默认分配1页内存，并始终限制不允许超过
        private byte[] binary_data = new byte[64];//AA 55 08 01 02 03 04 05 06 07 08 EAH EAL 

        private byte HeadFirst = 0x55;
        private byte HeadSecond = 0x77;

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

        public bool DataCatched
        {
            get
            {
                return data_catched;
            }
            set
            {
                data_catched = value;
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
            BaudRateComBox.SelectedIndex = 6;
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
                    bt.Text = "串口打开";
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
                        bt.Text = "串口关闭";
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

        public void ReceiveDataProcess(byte[] data)
        {
            //<协议解析>  
            data_catched = false;//缓存记录数据是否捕获到   
            buffer.AddRange(data);
            //2.完整性判断 
            while (buffer.Count >= 12)//至少要包含头（2字节）+命令（1字节）+ 长度（1字节）+校验（1字节）
            {
                //2.1 查找数据头
                if (buffer[0] == HeadFirst && buffer[1] == HeadSecond)
                {
                    //2.2 探测缓存数据是否有一条数据的字节，如果不够，就不用费劲的做其他验证了  
                    //前面已经限定了剩余长度>=4，那我们这里一定能访问到buffer[2]这个长度  
                    int len = buffer[2];//数据长度  
                                        //数据完整判断第一步，长度是否足够  
                                        //len是数据段长度,4个字节是while行注释的3部分长度  
                    if (buffer.Count < len + 4) break;//数据不够的时候什么都不做  
                                                      //这里确保数据长度足够，数据头标志找到，我们开始计算校验  
                                                      //2.3 校验数据，确认数据正确  
                                                      //异或校验，逐个字节异或得到校验码  
                                                      /**/
                    byte checksum = 0;
                    for (int i = 3; i < len + 3; i++)//len+3表示校验之前的位置  
                    {
                        checksum += buffer[i];
                    }
                    if (checksum != buffer[len + 3]) //如果数据校验失败，丢弃这一包数据  
                    {
                        buffer.RemoveRange(0, len + 4);//从缓存中删除错误数据  
                        continue;//继续下一次循环  
                    }
                    //至此，已经被找到了一条完整数据。我们将数据直接分析，或是缓存起来一起分析  
                    //我们这里采用的办法是缓存一次，好处就是如果你某种原因，数据堆积在缓存buffer中  
                    //已经很多了，那你需要循环的找到最后一组，只分析最新数据，过往数据你已经处理不及时  
                    //了，就不要浪费更多时间了，这也是考虑到系统负载能够降低。  
                    buffer.CopyTo(3, binary_data, 0, len);//复制一条完整数据到具体的数据缓存  
                    data_catched = true;
                    buffer.RemoveRange(0, len + 4);//正确分析一条数据，从缓存中移除数据。  
                }
                else
                {
                    //这里是很重要的，如果数据开始不是头，则删除数据  
                    buffer.RemoveAt(0);
                }
            }//while结束
             //分析数据 
        }
    }
}
