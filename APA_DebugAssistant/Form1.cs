using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace APA_DebugAssistant
{
    public partial class Form1 : Form
    {
        SerialCom m_SerialCom = new SerialCom();
        Vehicle m_Vehicle = new Vehicle();

        string[] GearState = new string[8] { "No Request", "驻车", "倒车", "空挡", "前进", "无效", "保留", "保留" };
        UInt16 AckCnt;
        #region 函数
        #region 车辆参数配置函数
        private void VehicleParameterConfigure()
        {
            byte[] Data = new byte[32];//动态分配内存
            byte[] Data_Temp = new byte[8];//动态分配内存
            byte CheckSum = 0;
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    Data[0] = 0xAA;//识别标志1
                    Data[1] = 0x55;//识别标志2
                    Data[2] = 0x3E;//数据标志
                    Data[3] = 19;//数据长度
                    Data[4] =(byte)(
                          (Convert.ToByte(checkBox1.Checked) << 5)
                        | (Convert.ToByte(checkBox2.Checked) << 4)
                        | (Convert.ToByte(checkBox3.Checked) << 3)
                        | (Convert.ToByte(checkBox4.Checked) << 2)
                        | (Convert.ToByte(checkBox5.Checked) << 1)
                        |  Convert.ToByte(checkBox5.Checked)     );//使能信号
                    Data[5] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位
                    Data[6] = Convert.ToByte(textBox3.Text);// 扭矩
                    Data[7] = 0;
                    Data_Temp = BitConverter.GetBytes(Convert.ToInt16(textBox4.Text));//转向角
                    Data[8] = Data_Temp[0];
                    Data[9] = Data_Temp[1];
                    Data_Temp = BitConverter.GetBytes(Convert.ToInt16(textBox5.Text));//转向角速度
                    Data[10] = Data_Temp[0];
                    Data[11] = Data_Temp[1];

                    Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox2.Text));//AEB
                    Data[12] = Data_Temp[0];
                    Data[13] = Data_Temp[1];
                    Data[14] = Data_Temp[2];
                    Data[15] = Data_Temp[3];

                    Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox1.Text));//ACC
                    Data[16] = Data_Temp[0];
                    Data[17] = Data_Temp[1];
                    Data[18] = Data_Temp[2];
                    Data[19] = Data_Temp[3];

                    Data[20] = 0;
                    Data[21] = 0;
                    Data[22] = 0x12;

                    CheckSum = 0;
                    for (int i = 0; i < 23; i++)
                    {
                        CheckSum += Data[i];
                    }
                    Data[23] = CheckSum;
                    serialPort1.Write(Data, 0, 24);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
            else
            {
                MessageBox.Show("请打开串口", "提示");
            }
        }
        #endregion

        #region 窗口显示函数
        private void VehicleImformationShow()
        {
            label4.Text = m_SerialCom.ErrCount.ToString();//帧错计数

            label5.ForeColor = m_Vehicle.EPS_Failed ? Color.AliceBlue : Color.BlueViolet;
            label6.ForeColor = m_Vehicle.ESPQDCACC ? Color.AliceBlue : Color.BlueViolet;
            label7.ForeColor = m_Vehicle.EMSQECACC ? Color.AliceBlue : Color.BlueViolet;

            label11.Text = m_Vehicle.SteeringAngleActual.ToString();//转向角
            label12.Text = m_Vehicle.SteeringAngleSpeed.ToString();//转向角速度
            label14.Text = m_Vehicle.SteeringTorque.ToString();//转向角扭矩


            label20.Text = m_Vehicle.VehicleSpeed.ToString();//车身速度

            label21.Text = m_Vehicle.WheelSpeedFrontLeftData.ToString();//左前轮速
            label22.Text = m_Vehicle.WheelSpeedFrontRightData.ToString();//右前轮速
            label23.Text = m_Vehicle.WheelSpeedRearLeftData.ToString();//左后轮速
            label24.Text = m_Vehicle.WheelSpeedRearRightData.ToString();//右后轮速

            label36.Text = m_Vehicle.WheelSpeedFrontLeftPulse.ToString();//左前脉冲
            label37.Text = m_Vehicle.WheelSpeedFrontRightPulse.ToString();//右前脉冲
            label38.Text = m_Vehicle.WheelSpeedRearLeftPulse.ToString();//左后脉冲
            label39.Text = m_Vehicle.WheelSpeedRearRightPulse.ToString();//右后脉冲
        }

        #endregion
        #endregion

        #region 事件
        public Form1()
        {
            InitializeComponent();

            m_SerialCom.AddBaudRate(comboBox2);
            serialPort1.Encoding = Encoding.GetEncoding("GB2312");
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(serialPort1_DataReceived);
        }
        private void Form1_Load(object sender, EventArgs e)
        {
            label5.ForeColor = Color.AliceBlue;
            label6.ForeColor = Color.AliceBlue;
            label7.ForeColor = Color.AliceBlue;

            timer_show.Enabled = true;

            for (int i = 0; i < 8; i++)
            {
                comboBox3.Items.Add(GearState[i]);
            }
            comboBox3.SelectedIndex = 1;
        }


        #region 串口操作事件
        /// <summary>
        /// 搜寻端口号
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void label1_Click(object sender, EventArgs e)
        {
            m_SerialCom.SearchAndAddSerialToComboBox(serialPort1, comboBox1);
        }

        /// <summary>
        /// 打开关闭串口
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            m_SerialCom.OpenAndCloseSerial(serialPort1, button1, comboBox1.Text, Convert.ToInt32(comboBox2.Text));
        }

        /// <summary>
        /// 控制参数发送
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button2_Click(object sender, EventArgs e)
        {
            VehicleParameterConfigure();
        }

        /// <summary>
        /// 串口接收事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (m_SerialCom.Closing) return;//如果正在关闭，忽略操作，直接返回，尽快的完成串口监听线程的一次循环  
            try
            {
                m_SerialCom.Listening = true;//设置标记，说明我已经开始处理数据，一会儿要使用系统UI的。
                int n = serialPort1.BytesToRead;//先记录下来，避免某种原因，人为的原因，操作几次之间时间长，缓存不一致  
                byte[] data = new byte[n];
                serialPort1.Read(data, 0, n);//读取缓冲数据 

                //<协议解析>  
                bool data_catched = false;//缓存记录数据是否捕获到  
                                          //1.缓存数据  
                m_SerialCom.Buffer.AddRange(data);
                //2.完整性判断 
                while (m_SerialCom.Buffer.Count >= 6)//至少要包含头（2字节）+命令（1字节）+ 长度（1字节）+ 数据（4字节）+ 校验（1字节）
                {
                    //2.1 查找数据头
                    if (m_SerialCom.Buffer[0] == 0x7F && m_SerialCom.Buffer[1] == 0x80)
                    {
                        int len = m_SerialCom.Buffer[3];//数据长度  
                        if (m_SerialCom.Buffer.Count < len + 5) break;
                        byte checksum = 0;
                        for (int i = 0; i < len + 4; i++)//len+3表示校验之前的位置  
                        {
                            checksum += m_SerialCom.Buffer[i];
                        }
                        if (checksum != m_SerialCom.Buffer[len + 4]) //如果数据校验失败，丢弃这一包数据  
                        {
                            m_SerialCom.Buffer.RemoveRange(0, len + 5);//从缓存中删除错误数据 
                            m_SerialCom.ErrCount++;
                            continue;//继续下一次循环  
                        }
                        //至此，已经被找到了一条完整数据。我们将数据直接分析，或是缓存起来一起分析  
                        //我们这里采用的办法是缓存一次，好处就是如果你某种原因，数据堆积在缓存buffer中  
                        //已经很多了，那你需要循环的找到最后一组，只分析最新数据，过往数据你已经处理不及时  
                        //了，就不要浪费更多时间了，这也是考虑到系统负载能够降低。  
                        m_SerialCom.Buffer.CopyTo(2, m_SerialCom.BinaryData, 0, len + 2);//复制一条完整数据到具体的数据缓存  
                        data_catched = true;
                        m_SerialCom.Buffer.RemoveRange(0, len + 5);//正确分析一条数据，从缓存中移除数据。  
                    }
                    else
                    {
                        //这里是很重要的，如果数据开始不是头，则删除数据  
                        m_SerialCom.Buffer.RemoveAt(0);
                    }
                }//while结束
                 //分析数据 

                if (data_catched)
                {
                    //我们的数据都是定好格式的，所以当我们找到分析出的数据1，就知道固定位置一定是这些数据，我们只要显示就可以了  
                    //progressBar1.Value = Convert.ToInt32(binary_data_1[4]);                 
                    //x = BitConverter.ToSingle(binary_data_1, 3);//3,4,5,6
                    //更新界面  
                    this.Invoke((EventHandler)(delegate
                    {
                        switch(m_SerialCom.BinaryData[0])
                        {
                            case 0x6F:
                                m_Vehicle.EPS_Failed = Convert.ToBoolean(m_SerialCom.BinaryData[2] & 0x01);
                                m_Vehicle.ESPQDCACC = Convert.ToBoolean((m_SerialCom.BinaryData[2] >> 1) & 0x01);
                                m_Vehicle.EMSQECACC = Convert.ToBoolean((m_SerialCom.BinaryData[2] >> 2) & 0x01);
                                m_Vehicle.SteeringAngleActual = BitConverter.ToInt16(m_SerialCom.BinaryData, 3);
                                break;

                            case 0x6E:
                                m_Vehicle.VehicleSpeed = (double)BitConverter.ToSingle(m_SerialCom.BinaryData, 2);

                                m_Vehicle.WheelSpeedFrontLeftData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 6) * 0.1);
                                m_Vehicle.WheelSpeedFrontRightData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 8) * 0.1);
                                m_Vehicle.WheelSpeedRearLeftData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 10) * 0.1);
                                m_Vehicle.WheelSpeedRearRightData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 12) * 0.1);

                                m_Vehicle.WheelSpeedFrontLeftPulse = m_SerialCom.BinaryData[14];
                                m_Vehicle.WheelSpeedFrontRightPulse = m_SerialCom.BinaryData[15];
                                m_Vehicle.WheelSpeedRearLeftPulse = m_SerialCom.BinaryData[16];
                                m_Vehicle.WheelSpeedRearRightPulse = m_SerialCom.BinaryData[17];

                                break;

                            case 0x3E:
                                if(m_SerialCom.BinaryData[2] == 0xA5)
                                {
                                    AckCnt = 0;
                                    timer_ack.Enabled = true;
                                }
                                break;

                            default:
                                Console.WriteLine("异常ID");
                                break;
                        }
                    }));
                }
            }
            finally
            {
                m_SerialCom.Listening = false;//我用完了，ui可以关闭串口了。     
            }// end try    
        }

        #endregion

        #region 定时器事件
        private void timer_show_Tick(object sender, EventArgs e)
        {
            VehicleImformationShow();
        }

        /// <summary>
        /// configure ack single
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer_ack_Tick(object sender, EventArgs e)
        {
            if(AckCnt == 0)
            {
                button2.BackColor = Color.Green;
            }
            else if(AckCnt >= 5)
            {
                button2.BackColor = Color.Transparent;
                timer_ack.Enabled = false;
            }
            AckCnt++;
        }
        #endregion

        #endregion


    }
}
