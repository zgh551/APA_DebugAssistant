using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using static APA_DebugAssistant.Ultrasonic;

namespace APA_DebugAssistant
{
    public partial class Form1 : Form
    {
        #region 全局变量

        #region 串口相关变量
        SerialCom m_SerialCom = new SerialCom();
        UInt16 AckCnt;
        byte AckId;
        #endregion

        #region CAN相关变量
        ZLGCAN m_ZLGCAN = new ZLGCAN();
        #endregion

        #region 车辆本身相关变量
        Vehicle m_Vehicle = new Vehicle();
        double setTargetVehicleSpeed = 0;
        string[] GearState = new string[8] { "No Request", "驻车", "倒车", "空挡", "前进", "无效", "保留", "保留" };
        string[] VehicleDirection = new string[4] { "前进","后退","停车","无效"};
        string[] SteeringAngleActiveStatus = new string[4] { "无请求", "请求控制", "控制激活", "无效" };
        #endregion

        #region 数据显示变量
        Waveform m_Waveform = new Waveform();

        float test_cnt = 0;
        #endregion

        #region 超声参数
        public LIN_STP313_ReadData[] m_LIN_STP313_ReadData = new LIN_STP313_ReadData[4];
        public LIN_STP318_ReadData[] m_LIN_STP318_ReadData = new LIN_STP318_ReadData[8];

        //LRU STP313 传感器 控件显示
        Label[][] SensingControl_LRU = new Label[4][];
        //SRU STP318 传感器 控件显示
        Label[][] SensingControl_SRU = new Label[8][];

        Ultrasonic m_Ultrasonic = new Ultrasonic();
        #endregion

        #region PID参数
        float Kp, Ki, Kd, Threshold;
        #endregion

        #region System Working State
        string[] WorkingModule = new string[4] { "调试", "标定", "测试", "正常"};
        string[][] FunctionStatus = new string[4][]
        {
            new string [4] { "直接控制", "速度控制", "长安车控制", "超声波收发"},
            new string [3] { "速度标定", "脉冲标定", "超声波标定"},
            new string [4] { "车位检测", "侧方停车", "垂直停车", "斜向停车" },
            new string [5] { "APA_1.0", "APA_2.0", "APA_3.0", "APA_4.0", "APA_5.0"}
        };

        byte WorkingModuleValue = 0, FunctionStatusValue = 0;
        #endregion

        #region File Operation Relation Variable
        StreamWriter DataSave;
        string FilePath, localFilePath, newFileName, fileNameExt;
        bool DataSaveStatus = false;
        #endregion

        #endregion

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
                    Data[2] = 0x1A;//数据标志
                    Data[3] = 19;//数据长度
                    Data[4] =(byte)(
                          (Convert.ToByte(checkBox1.Checked) << 5)
                        | (Convert.ToByte(checkBox2.Checked) << 4)
                        | (Convert.ToByte(checkBox3.Checked) << 3)
                        | (Convert.ToByte(checkBox4.Checked) << 2)
                        | (Convert.ToByte(checkBox5.Checked) << 1)
                        |  Convert.ToByte(checkBox5.Checked)     );//使能信号
                    Data[5] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位

                    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox3.Text) * 10));//发动机扭矩
                    Data[6] = Data_Temp[0];
                    Data[7] = Data_Temp[1];

                    Data_Temp = BitConverter.GetBytes( (Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
                    Data[8] = Data_Temp[0];
                    Data[9] = Data_Temp[1];

                    Data_Temp = BitConverter.GetBytes( (UInt16)(Convert.ToSingle(textBox5.Text) * 100));//转向角速度
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
                    Data[22] = 0;

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

        #region PID参数设置
        private void PID_ParameterConfigure()
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
                    Data[2] = 0x2A;//数据标志
                    Data[3] = 19;//数据长度
                    
                    Kp = Convert.ToSingle(textBox6.Text);
                    Data_Temp = BitConverter.GetBytes(Kp);//KP
                    Data[4] = Data_Temp[0];
                    Data[5] = Data_Temp[1];
                    Data[6] = Data_Temp[2];
                    Data[7] = Data_Temp[3];

                    Ki = Convert.ToSingle(textBox7.Text);
                    Data_Temp = BitConverter.GetBytes(Ki);//KI
                    Data[8] = Data_Temp[0];
                    Data[9] = Data_Temp[1];
                    Data[10] = Data_Temp[2];
                    Data[11] = Data_Temp[3];

                    Kd = Convert.ToSingle(textBox8.Text);
                    Data_Temp = BitConverter.GetBytes(Kd);//KD
                    Data[12] = Data_Temp[0];
                    Data[13] = Data_Temp[1];
                    Data[14] = Data_Temp[2];
                    Data[15] = Data_Temp[3];

                    for (int i=16;i<23;i++)
                    {
                        Data[i] = 0;
                    }

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

        #region PID修正参数设置
        private void PID_ReviseParameterConfigure()
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
                    Data[2] = 0x2B;//数据标志
                    Data[3] = 19;//数据长度

                    Threshold = Convert.ToSingle(textBox11.Text);
                    Data_Temp = BitConverter.GetBytes(Threshold);//OutputOffset
                    Data[4] = Data_Temp[0];
                    Data[5] = Data_Temp[1];
                    Data[6] = Data_Temp[2];
                    Data[7] = Data_Temp[3];

                    //Ki = Convert.ToSingle(textBox7.Text);
                    //Data_Temp = BitConverter.GetBytes(Ki);//KI
                    //Data[8] = Data_Temp[0];
                    //Data[9] = Data_Temp[1];
                    //Data[10] = Data_Temp[2];
                    //Data[11] = Data_Temp[3];

                    //Kd = Convert.ToSingle(textBox8.Text);
                    //Data_Temp = BitConverter.GetBytes(Kd);//KD
                    //Data[12] = Data_Temp[0];
                    //Data[13] = Data_Temp[1];
                    //Data[14] = Data_Temp[2];
                    //Data[15] = Data_Temp[3];


                    //Data[16] = Data_Temp[0];
                    //Data[17] = Data_Temp[1];
                    //Data[18] = Data_Temp[2];
                    //Data[19] = Data_Temp[3];

                    for (int i = 8; i < 23; i++)
                    {
                        Data[i] = 0;
                    }

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

        #region 目标速度参数设置
        private void TargetSpeedParameterConfigure()
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
                    Data[2] = 0x3A;//数据标志
                    Data[3] = 11;//数据长度

                    setTargetVehicleSpeed = Convert.ToSingle(textBox9.Text);
                    Data_Temp = BitConverter.GetBytes((float)setTargetVehicleSpeed);//目标速度值(m/s)                   
                    Data[4] = Data_Temp[0];
                    Data[5] = Data_Temp[1];
                    Data[6] = Data_Temp[2];
                    Data[7] = Data_Temp[3];

                    for (int i = 8; i < 15; i++)
                    {
                        Data[i] = 0;
                    }

                    CheckSum = 0;
                    for (int i = 0; i < 15; i++)
                    {
                        CheckSum += Data[i];
                    }
                    Data[15] = CheckSum;
                    serialPort1.Write(Data, 0, 16);
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

        #region 系统模式设定
        private void SystemModuleConfigure()
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
                    Data[2] = 0x4A;//数据标志
                    Data[3] = 3;//数据长度
 
                    Data[4] = Convert.ToByte(comboBox4.SelectedIndex);
                    Data[5] = Convert.ToByte(comboBox5.SelectedIndex);
                    Data[6] = 0;

                    CheckSum = 0;
                    for (int i = 0; i < 7; i++)
                    {
                        CheckSum += Data[i];
                    }
                    Data[7] = CheckSum;
                    serialPort1.Write(Data, 0, 8);
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

        #region 长安对接接口
        /// <summary>
        /// 用于长安车调试的接口控制函数
        /// </summary>
        private void ChangAnInterfaceTest()
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
                    Data[2] = 0x5A;//数据标志
                    Data[3] = 11;//数据长度
                    Data[4] = (byte)(
                                      (Convert.ToByte(checkBox6.Checked) << 3)// Taget Speed 
                                    | (Convert.ToByte(checkBox1.Checked) << 2)// ACC
                                    | (Convert.ToByte(checkBox4.Checked) << 1)// Steering Angle
                                    |  Convert.ToByte(checkBox5.Checked)      // Gear enable
                                    );     
                    Data[5] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位

                    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
                    Data[6] = Data_Temp[0];
                    Data[7] = Data_Temp[1];

                    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox5.Text) * 100));//转向角速度
                    Data[8] = Data_Temp[0];
                    Data[9] = Data_Temp[1];

                    Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox1.Text));//ACC
                    Data[10] = Data_Temp[0];
                    Data[11] = Data_Temp[1];
                    Data[12] = Data_Temp[2];
                    Data[13] = Data_Temp[3];

                    Data[14] = (byte)(Convert.ToSingle(textBox12.Text) * 100);// Target Speed

                    CheckSum = 0;
                    for (int i = 0; i < (Data[3] + 4); i++)
                    {
                        CheckSum += Data[i];
                    }
                    Data[Data[3] + 4] = CheckSum;
                    serialPort1.Write(Data, 0, Data[3] + 5);
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

        private void ChangAnInterfaceCAN1()
        {
            uint id = 0x506;
            byte len = 8;
            byte CheckSum;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            dat[0] = (byte)(
                                (Convert.ToByte(checkBox6.Checked) << 5)// Velocity 
                            | (Convert.ToByte(checkBox3.Checked) << 4)// Torque 
                            | (Convert.ToByte(checkBox2.Checked) << 3)// AEB 
                            | (Convert.ToByte(checkBox1.Checked) << 2)// ACC
                            | (Convert.ToByte(checkBox4.Checked) << 1)// Steering Angle
                            | Convert.ToByte(checkBox5.Checked)      // Gear enable
                            );
            dat[1] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位

            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];

            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox5.Text) * 100));//转向角速度
            dat[4] = Data_Temp[0];
            dat[5] = Data_Temp[1];

            dat[6] = 0;
            CheckSum = 0;
            for (int i = 0; i < 7; i++)
            {
                CheckSum += dat[i];
            }
            CheckSum ^= 0xFF;
            dat[7] = CheckSum;
            m_ZLGCAN.CAN_Send(0, id, len, dat);
        }
        private void ChangAnInterfaceCAN2()
        {
            uint id = 0x507;
            byte len = 8;
            byte CheckSum;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox1.Text) * 1000));//ACC加速度
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];

            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox2.Text) * 1000));//AEB减速度
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];

            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox12.Text) * 1000));//车辆速度
            dat[4] = Data_Temp[0];
            dat[5] = Data_Temp[1];

            dat[6] = (byte)(Convert.ToSingle(textBox3.Text) * 2);// 扭矩
            CheckSum = 0;
            for (int i = 0; i < 7; i++)
            {
                CheckSum += dat[i];
            }
            CheckSum ^= 0xFF;
            dat[7] = CheckSum;
            m_ZLGCAN.CAN_Send(0, id, len, dat);
        }
        #endregion

        #region 终端解码函数
        /// <summary>
        /// Chang An vehicle imformation receive
        /// </summary>
        /// <param name="m_packet"></param>
        /// <param name="m_vehicle"></param>
        unsafe void Parse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            byte[] tmp_dat = new byte[2] { 0, 0 };
            byte[] dat = new byte[8];
            switch (m_packet.ID)
            {
                case 0x400://传感器1
                case 0x401://传感器2
                case 0x402://传感器3
                case 0x403://传感器4
                case 0x404://传感器5
                case 0x405://传感器6
                case 0x406://传感器7
                case 0x407://传感器8
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_LIN_STP318_ReadData[m_packet.ID & 0x007].TOF      = BitConverter.ToUInt16(tmp_dat, 0);
                    m_LIN_STP318_ReadData[m_packet.ID & 0x007].status   = m_packet.Data[6];
                    break;
                case 0x408://传感器9
                case 0x409://传感器10
                case 0x40A://传感器11
                case 0x40B://传感器12
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF1 = BitConverter.ToUInt16(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF2 = BitConverter.ToUInt16(tmp_dat, 0);
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].Level = m_packet.Data[4];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].Width = m_packet.Data[5];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].status = m_packet.Data[6];
                    break;
                default:

                    break;
            }
        }

        #endregion

        #region 窗口显示函数
        private void VehicleImformationShow()
        {
            try
            {
                label4.Text = m_SerialCom.ErrCount.ToString();//帧错计数

                label5.ForeColor = m_Vehicle.EPS_Failed ? Color.AliceBlue : Color.BlueViolet;
                label6.ForeColor = m_Vehicle.ESPQDCACC ? Color.AliceBlue : Color.BlueViolet;
                label7.ForeColor = m_Vehicle.EMSQECACC ? Color.AliceBlue : Color.BlueViolet;

                label52.ForeColor = m_Vehicle.TargetAccelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                label53.ForeColor = m_Vehicle.TargetDecelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                label54.ForeColor = m_Vehicle.TorqueEnable             ? Color.DarkGoldenrod : Color.AliceBlue;
                label55.ForeColor = m_Vehicle.GearShiftEnable          ? Color.DarkGoldenrod : Color.AliceBlue;

                label59.Text = SteeringAngleActiveStatus[m_Vehicle.SteeringAngleActive];
                label60.Text = m_Vehicle.TargetAccelerationACC.ToString();
                label61.Text = m_Vehicle.Torque.ToString();

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

                if(m_Vehicle.WheelSpeedDirection <= VehicleDirection.Length)
                {
                    label40.Text = VehicleDirection[m_Vehicle.WheelSpeedDirection];
                }
                
                if(WorkingModuleValue <= WorkingModule.Length)
                {
                    label50.Text = WorkingModule[WorkingModuleValue];
                }
                if(FunctionStatusValue <= FunctionStatus[WorkingModuleValue].Length)
                {
                    label51.Text = FunctionStatus[WorkingModuleValue][FunctionStatusValue];
                }
            }
            catch
            {
                MessageBox.Show("反馈参数异常", "数据类型错误");
            }
        }

        private void UltrasonicImformationShow()
        {
            for(int i =0;i<8;i++ )
            {
                m_Ultrasonic.DataMapping2Control_STP318(m_LIN_STP318_ReadData[i],ref SensingControl_SRU[i]);
            }
            for (int i = 0; i < 4; i++)
            {
                m_Ultrasonic.DataMapping2Control_STP313(m_LIN_STP313_ReadData[i], ref SensingControl_LRU[i]);
            }
            //label86.Text = (m_LIN_STP313_ReadData[0].TOF1 / 58.0).ToString();
        }
        #endregion

        #region DataSave Relation Function
        private void DataLog()
        {
            //数据保存
            if (DataSaveStatus)
            {
                DataSave.Write( "{0:R16} {1:D} {2:R16} {3:R16} " +
                                "{4:R16} {5:R16} {6:R16} {7:R16} " +
                                "{8:D} {9:D} {10:D} {11:D} {12:D} " +
                                "{13:R} {14:R} {15:R} {16:R} " +
                                "\r\n",
                                /// Steering Angle 
                                m_Vehicle.SteeringAngleActual,
                                m_Vehicle.SteeringAngleSpeed,
                                m_Vehicle.SteeringTorque,
                                /// Vehicle Speed
                                m_Vehicle.VehicleSpeed,
                                /// WheelSpeed
                                m_Vehicle.WheelSpeedFrontLeftData,
                                m_Vehicle.WheelSpeedFrontRightData,
                                m_Vehicle.WheelSpeedRearLeftData,
                                m_Vehicle.WheelSpeedRearRightData,
                                /// WheelSpeedPulse
                                m_Vehicle.WheelSpeedFrontLeftPulse,
                                m_Vehicle.WheelSpeedFrontRightPulse,
                                m_Vehicle.WheelSpeedRearLeftPulse,
                                m_Vehicle.WheelSpeedRearRightPulse,
                                m_Vehicle.WheelSpeedDirection,
                                /// PID Parameter
                                Kp, Ki, Kd, Threshold
                );
            }
        }
        #endregion

        #endregion

        #region 事件
        #region 初始窗口事件
        public Form1()
        {
            InitializeComponent();

            m_SerialCom.AddBaudRate(comboBox2);
            serialPort1.Encoding = Encoding.GetEncoding("GB2312");
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(serialPort1_DataReceived);

            // add the thread of the can receive
            ThreadStart CANTreadChild = new ThreadStart(CallToCANReceiveThread);
            Thread m_CanReceiveChildThread = new Thread(CANTreadChild);
            m_CanReceiveChildThread.Priority = ThreadPriority.Normal;
            m_CanReceiveChildThread.IsBackground = true;
            m_CanReceiveChildThread.Start();
        }
        private void Form1_Load(object sender, EventArgs e)
        {
            m_SerialCom.SearchAndAddSerialToComboBox(serialPort1, comboBox1);

            label5.ForeColor = Color.AliceBlue;
            label6.ForeColor = Color.AliceBlue;
            label7.ForeColor = Color.AliceBlue;

            timer_show.Enabled = true;

            for (int i = 0; i < 8; i++)
            {
                comboBox3.Items.Add(GearState[i]);
            }
            comboBox3.SelectedIndex = 1;

            for(int i = 0; i < 4;i++)
            {
                comboBox4.Items.Add(WorkingModule[i]);
            }
            comboBox4.SelectedIndex = 0;

            //长距离传感器的控件显示
            SensingControl_LRU = new Label[4][] {
                new Label[5]{ label86, label87, label88, label89, label90 },
                new Label[5]{ label91, label92, label93, label94, label95 },
                new Label[5]{ label96, label97, label98, label99, label100 },
                new Label[5]{ label101, label102, label103, label104, label105 }
            };
            //短距离传感器的控件初始化
            SensingControl_SRU = new Label[8][]
            {
                new Label[2]{label70,label71 },
                new Label[2]{label72,label79 },
                new Label[2]{label73,label80 },
                new Label[2]{label74,label81 },
                new Label[2]{label75,label82 },
                new Label[2]{label76,label83 },
                new Label[2]{label77,label84 },
                new Label[2]{label78,label85 }
            };
            //SensingControl_SRU_LabelTOF    = new TextBox[8] { textBox1, textBox6, textBox7, textBox8, textBox9, textBox10, textBox11, textBox12 };
            //SensingControl_SRU_LabelStatus = new Label[8] { label26, label27, label28, label29, label30, label31, label41, label42 };
            //for (int i = 0; i < FunctionStatus[0].Length; i++)
            //{
            //    comboBox5.Items.Add(FunctionStatus[0][i]);
            //}
            //comboBox5.SelectedIndex = 0;
        }
        #endregion

        #region CAN接收线程
        /// <summary>
        /// CAN 接收线程函数
        /// </summary>
        public void CallToCANReceiveThread()
        {
            CAN_ReceiveTask();
        }
        /// <summary>
        /// CAN接收任务
        /// </summary>
        private void CAN_ReceiveTask()
        {
            while (true)
            {
                if(m_ZLGCAN.OpenStatus == 1)
                {
                    ZLGCAN.VCI_CAN_OBJ[] obj = new ZLGCAN.VCI_CAN_OBJ[1];
                    m_ZLGCAN.CAN_Receive(0,ref obj);
                    for(int i=0;i< obj.Length;i++)
                    {
                        Parse(obj[i]);
                    }
                }
                Thread.Sleep(10);
            }
        }
        #endregion

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
        /// PID参数设置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button3_Click(object sender, EventArgs e)
        {
            PID_ParameterConfigure();
        }

        /// <summary>
        /// PID revise parameter
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button8_Click(object sender, EventArgs e)
        {
            PID_ReviseParameterConfigure();
        }

        /// <summary>
        /// 目标速度设置事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4_Click(object sender, EventArgs e)
        {
            TargetSpeedParameterConfigure();
        }

        /// <summary>
        /// 长安对接测试
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button10_Click(object sender, EventArgs e)
        {
            //ChangAnInterfaceTest();
            ChangAnInterfaceCAN1();
            ChangAnInterfaceCAN2();
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
                            case 0x85:
                                m_Vehicle.EPS_Failed = Convert.ToBoolean(m_SerialCom.BinaryData[2] & 0x01);
                                m_Vehicle.ESPQDCACC = Convert.ToBoolean((m_SerialCom.BinaryData[2] >> 1) & 0x01);
                                m_Vehicle.EMSQECACC = Convert.ToBoolean((m_SerialCom.BinaryData[2] >> 2) & 0x01);

                                m_Vehicle.TargetAccelerationEnable = Convert.ToBoolean( m_SerialCom.BinaryData[3]       & 0x01);
                                m_Vehicle.TargetDecelerationEnable = Convert.ToBoolean((m_SerialCom.BinaryData[3] >> 1) & 0x01);
                                m_Vehicle.TorqueEnable             = Convert.ToBoolean((m_SerialCom.BinaryData[3] >> 2) & 0x01);
                                m_Vehicle.GearShiftEnable          = Convert.ToBoolean((m_SerialCom.BinaryData[3] >> 3) & 0x01);
                                m_Vehicle.SteeringAngleActive      = Convert.ToByte((m_SerialCom.BinaryData[3] >> 4) & 0x03);

                                m_Vehicle.SteeringAngleActual = BitConverter.ToInt16(m_SerialCom.BinaryData, 4);
                                m_Vehicle.SteeringAngleSpeed = BitConverter.ToUInt16(m_SerialCom.BinaryData, 6);
                                m_Vehicle.SteeringTorque = BitConverter.ToSingle(m_SerialCom.BinaryData, 8);
                                m_Vehicle.TargetAccelerationACC = BitConverter.ToSingle(m_SerialCom.BinaryData, 12);
                                m_Vehicle.Torque = BitConverter.ToUInt16(m_SerialCom.BinaryData, 16)*0.1;
                                break;

                            case 0x95:
                                m_Vehicle.VehicleSpeed = (double)BitConverter.ToSingle(m_SerialCom.BinaryData, 2);

                                m_Vehicle.WheelSpeedFrontLeftData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 6) * 0.1);
                                m_Vehicle.WheelSpeedFrontRightData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 8) * 0.1);
                                m_Vehicle.WheelSpeedRearLeftData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 10) * 0.1);
                                m_Vehicle.WheelSpeedRearRightData = (double)(BitConverter.ToInt16(m_SerialCom.BinaryData, 12) * 0.1);

                                m_Vehicle.WheelSpeedFrontLeftPulse = m_SerialCom.BinaryData[14];
                                m_Vehicle.WheelSpeedFrontRightPulse = m_SerialCom.BinaryData[15];
                                m_Vehicle.WheelSpeedRearLeftPulse = m_SerialCom.BinaryData[16];
                                m_Vehicle.WheelSpeedRearRightPulse = m_SerialCom.BinaryData[17];
                                m_Vehicle.WheelSpeedDirection = m_SerialCom.BinaryData[18];
                                if (m_Waveform.Visible)
                                {
                                    m_Waveform.VehicleSpeedPointAdd(setTargetVehicleSpeed,m_Vehicle.VehicleSpeed);
                                }
                                break;

                            case 0xA5:
                                WorkingModuleValue = m_SerialCom.BinaryData[2];
                                FunctionStatusValue = m_SerialCom.BinaryData[3];
                                break;

                            case 0x1A:
                            case 0x2A:
                            case 0x2B:
                            case 0x3A:
                            case 0x4A:
                            case 0x5A:
                                if (m_SerialCom.BinaryData[2] == 0x5A)
                                {
                                    AckId = m_SerialCom.BinaryData[0];
                                    AckCnt = 0;
                                    timer_ack.Enabled = true;
                                }
                                break;

                            default:
                                MessageBox.Show("异常ID", "提示");
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

        #region CAN 操作相关事件
        /// <summary>
        /// CAN连接
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button11_Click(object sender, EventArgs e)
        {
            m_ZLGCAN.CAN_Connect(0);
            m_ZLGCAN.CAN_Connect(1);
            button11.Text = m_ZLGCAN.OpenStatus == 0 ? "连接" : "断开";
        }
        /// <summary>
        /// 打开CAN设备
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button12_Click(object sender, EventArgs e)
        {
            m_ZLGCAN.CAN_Open(0);
            m_ZLGCAN.CAN_Open(1);
        }
        /// <summary>
        /// 关闭CAN设备
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button13_Click(object sender, EventArgs e)
        {
            m_ZLGCAN.CAN_Reset(0);
            m_ZLGCAN.CAN_Reset(1);
            m_ZLGCAN.CAN_Close();
        }
        #endregion

        #region 定时器事件
        private void timer_show_Tick(object sender, EventArgs e)
        {
            VehicleImformationShow();
            UltrasonicImformationShow();
            DataLog();
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
                switch(AckId)
                {
                    case 0x1A:// Vehicle Control
                        button2.BackColor = Color.Green;
                        break;

                    case 0x2A:// PID Parameter Configure
                        button3.BackColor = Color.Green;
                        break;

                    case 0x2B://PID Parameter Revise
                        button8.BackColor = Color.Green;
                        break;

                    case 0x3A:// Targte Vehicle Configure
                        button4.BackColor = Color.Green;
                        break;

                    case 0x4A:// Sysytem Working module Configure
                        button9.BackColor = Color.Green;
                        break;

                    case 0x5A:// chang an Test
                        button10.BackColor = Color.Green;
                        break;

                    default:
                        MessageBox.Show("异常应答ID", "提示");
                        break;
                }
            }
            else if(AckCnt >= 5)
            {
                switch (AckId)
                {
                    case 0x1A:// Vehicle Control
                        button2.BackColor = Color.Transparent;
                        break;

                    case 0x2A:// PID Parameter Configure
                        button3.BackColor = Color.Transparent;
                        break;

                    case 0x2B://PID Parameter Revise
                        button8.BackColor = Color.Transparent;
                        break;

                    case 0x3A:// Targte Vehicle Configure
                        button4.BackColor = Color.Transparent;
                        break;

                    case 0x4A:// Sysytem Working module Configure
                        button9.BackColor = Color.Transparent;
                        break;

                    case 0x5A:// chang an Test
                        button10.BackColor = Color.Transparent;
                        break;

                    default:
                        MessageBox.Show("异常应答ID", "提示");
                        break;
                }
                timer_ack.Enabled = false;
            }
            AckCnt++;  
        }

        #endregion

        #region 图形化显示
        private void button7_Click(object sender, EventArgs e)
        {
            if (m_Waveform.Visible)
            {
                m_Waveform.Hide();
                button7.Text = "波形图显示关闭";
            }
            else
            {
                m_Waveform.Show();
                button7.Text = "波形图显示开启";
            }
        }
        #endregion

        #region 数据保存
        /// <summary>
        /// 保存路径选择
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button5_Click(object sender, EventArgs e)
        {
            saveFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
            saveFileDialog1.InitialDirectory = "D:\\APA\\NXP_DataSet";
            saveFileDialog1.Title = "请选择要保存的文件路径";
            saveFileDialog1.FilterIndex = 2;
            saveFileDialog1.DefaultExt = "txt";
            saveFileDialog1.FileName = "Data.txt";
            saveFileDialog1.RestoreDirectory = true;
            saveFileDialog1.AddExtension = true;
            DialogResult dr = saveFileDialog1.ShowDialog();
            if (dr == DialogResult.OK && saveFileDialog1.FileName.Length > 0)
            {
                //获得文件路径
                localFilePath = saveFileDialog1.FileName.ToString();
                //获取文件路径，不带文件名
                FilePath = localFilePath.Substring(0, localFilePath.LastIndexOf("\\"));
                //获取文件名，不带路径
                fileNameExt = localFilePath.Substring(localFilePath.LastIndexOf("\\") + 1);
            }
        }

        /// <summary>
        /// 开始保存数据
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button6_Click(object sender, EventArgs e)
        {
            if (!DataSaveStatus)
            {
                //给文件名前加上时间
                newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox10.Text + "_" + fileNameExt;
                DataSave = new StreamWriter(FilePath + "\\" + newFileName, true, Encoding.ASCII);
                DataSaveStatus = true;
            }
            else
            {
                DataSave.Close();
                DataSaveStatus = false;
            }
            button6.Text = DataSaveStatus ? "取消保存" : "开始保存";
            button6.BackColor = DataSaveStatus ? Color.Green : Color.Red;
        }
        #endregion

        #region 模式切换事件
        /// <summary>
        /// based on the working module select the function
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void comboBox4_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                comboBox5.Items.Clear();
                for (int i = 0; i < FunctionStatus[comboBox4.SelectedIndex].Length; i++)
                {
                    comboBox5.Items.Add(FunctionStatus[comboBox4.SelectedIndex][i]);
                }
                comboBox5.SelectedIndex = 0;
            }
            catch
            {
                MessageBox.Show("模式索引越界！");
            }
        }

        /// <summary>
        /// Configure the system Working State and function status
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button9_Click(object sender, EventArgs e)
        {
            SystemModuleConfigure();
        }
        #endregion
        #endregion




    }
}
