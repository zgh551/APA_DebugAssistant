using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
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
        UInt32 VehicleSendCAN    = 0;
        UInt32 VehicleReceiveCAN = 1;
        UInt32 TerminalCAN       = 2;
        #endregion

        #region 车辆本身相关变量
        Vehicle m_Vehicle = new Vehicle();
        double setTargetVehicleSpeed = 0;
        string[] GearState = new string[8] { "No Request", "驻车", "倒车", "空挡", "前进", "无效", "保留", "保留" };
        string[] VehicleDirection = new string[4] { "前进","后退","停车","无效"};
        string[] SteeringAngleActiveStatus = new string[4] { "无请求", "请求控制", "控制激活", "无效" };

        const double FRONT_EDGE_TO_CENTER = 3.54;
        const double REAR_EDGE_TO_CENTER  = 0.905;
        const double LEFT_EDGE_TO_CENTER  = 0.9275;
        const double RIGHT_EDGE_TO_CENTER = 0.9275;

        private Polar FrontLeftDiagonal;
        private Polar FrontRightDiagonal;
        private Polar RearLeftDiagonal;
        private Polar RearRightDiagonal;
        #endregion

        #region 数据显示变量
        Waveform m_Waveform = new Waveform();

        Series UltrasonicDataShow = new Series();//超声显示

        Series TrackDataShow = new Series();//跟踪数据显示
        Series TrackEdgeDataShow = new Series();//跟踪车辆轮廓数据显示

        Series ParkingDataShow = new Series();//车位数据显示
        Series VehicleModuleShow = new Series();//车辆模型显示

        Series TurnningPointShow = new Series();//车辆转向角显示

        byte track_update_status;
        byte parking_update_status;
        byte vehicle_update_status;
        #endregion

        #region 超声参数
        public LIN_STP313_ReadData[] m_LIN_STP313_ReadData = new LIN_STP313_ReadData[4];
        public LIN_STP318_ReadData[] m_LIN_STP318_ReadData = new LIN_STP318_ReadData[8];

        public Ultrasonic_Data_Packet[] m_Ultrasonic_Data_Packet = new Ultrasonic_Data_Packet[12];
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

        #region ParkingMoudle
        string[] ParkingModule = new string[4] { "平行", "垂直", "斜向", "保留" };
        #endregion

        #region File Operation Relation Variable
        StreamWriter DataSave;
        string FilePath, localFilePath, newFileName, fileNameExt;
        bool DataSaveStatus = false;
        #endregion

        #region 超生波检车位数据保存相关变量
        StreamWriter ULtrasonicDataSave;
        StreamWriter ParkingDataSave;
        string u_FilePath, u_localFilePath, u_newFileName, u_fileNameExt;
        bool u_DataSaveStatus = false;
        #endregion

        #region Ultrasonic File Loading
        StreamReader UltrasonicDataLoad;
        string LoadFilePath;/* localFilePath, newFileName, fileNameExt;*/
        Int32 FileLineCount;
        //bool DataLoadStatus = false;
        #endregion

        #region 轨迹跟踪信息变量
        public struct Vector2d
        {
            public double X;
            public double Y;
        }
        public struct Polar
        {
            public double Length;
            public double Angle;
        }
        public struct LocationPoint
        {
            public Vector2d Position;
            public double Yaw;
        }
        public struct TurnPoint
        {
            public Vector2d Position;
            public double SteeringAngle;
        }

        LocationPoint TrackPoint;
        Vector2d[] VehicleEdgePoint = new Vector2d[4];
        private LocationPoint[] FrontTrial = new LocationPoint[10];
        private LocationPoint[] RearTrial = new LocationPoint[10];
        LocationPoint VehicleInitPosition = new LocationPoint();
        LocationPoint ParkingEnterPosition = new LocationPoint();

        private TurnPoint[] TurnningPointArrary = new TurnPoint[6];
        private byte TurnningPointNumber = 0;
        private byte TrialCnt = 0,TrialPoint = 0;
        #endregion

        #region 车位信息
        private double ParkingLength;
        private double ParkingWidth;
        private byte DetectorStatus;
        private double LatMarginMove;
        private double FrontMarginBoundary;
        private double RearMarginBoundary;

        string[] ParkingDetecterStatus = new string[4] { "待机","检测","成功","完成"};
        #endregion

        #region 泊车状态
        byte ParkingStatus = 0;
        Vector2d ParkingCenterPoint;

        string[] ParkingPlanningStatus = new string[4] { "待机", "规划", "控制", "完成" };
        #endregion

        #region Time Relation Function Interface and Variable
        [DllImport("winmm")]
        static extern uint timeGetTime();
        [DllImport("winmm")]
        static extern void timeBeginPeriod(int t);
        [DllImport("winmm")]
        static extern void timeEndPeriod(int t);
        uint LastTime,ErrTime;
        #endregion
        #endregion
        #region 函数
        #region 车辆参数配置函数(串口)
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

        #region PID参数设置(串口)
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

        #region PID修正参数设置(串口)
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

        #region 目标速度参数设置(串口)
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

        #region 系统模式设定(串口)
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

        #region 超声波数据注入
        /// <summary>
        /// 超声长距数据注入
        /// </summary>
        /// <param name="id_num"></param>
        /// <param name="u"></param>
        private void UltrasonicCAN(uint id_num,LIN_STP313_ReadData u)
        {
            uint id = 0x500 | id_num;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            dat[0] = (byte)( u.TOF1 & 0xff);
            dat[1] = (byte)((u.TOF1 >> 8) & 0xff);

            dat[2] = (byte)(u.TOF2 & 0xff);
            dat[3] = (byte)((u.TOF2 >> 8) & 0xff);

            dat[4] = u.Level;
            dat[5] = u.Width;
            dat[6] = u.status;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        
        /// <summary>
        /// 车辆速度信息
        /// </summary>
        /// <param name="v"></param>
        private void VehicleVelocityCAN(Vehicle v)
        {
            uint id = 0x520;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            dat[0] = 0;
            dat[1] = 0;
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = (byte)(( (UInt16)(v.WheelSpeedRearLeftData * 277.78))       & 0xFF);
            dat[5] = (byte)((((UInt16)(v.WheelSpeedRearLeftData * 277.78)) >> 8) & 0xFF);
            dat[6] = (byte)(( (UInt16)(v.WheelSpeedRearRightData * 277.78))       & 0xFF);
            dat[7] = (byte)((((UInt16)(v.WheelSpeedRearRightData * 277.78)) >> 8) & 0xFF);
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        /// <summary>
        /// 车辆状态
        /// </summary>
        /// <param name="v"></param>
        private void VehicleStatusCAN(Vehicle v)
        {
            uint id = 0x510;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            dat[0] = 0;
            dat[1] = v.SaveTime;
            dat[2] = (byte)(( (Int16)(v.SteeringAngleActual * 10))       & 0xFF);
            dat[3] = (byte)((((Int16)(v.SteeringAngleActual * 10)) >> 8) & 0xFF);
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        /// <summary>
        /// 注入文件解析
        /// </summary>
        /// <param name="s"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        private void FileDataParse(string [] s,ref LIN_STP313_ReadData [] u,ref Vehicle v)
        {
            for(int i=0;i<4;i++)
            {
                u[i].TOF1  = (UInt16)(Convert.ToSingle(s[6 + i]) * 58);
                u[i].Width =(byte)( Convert.ToSingle(s[10 + i]) / 16);
                u[i].TOF2  = (UInt16)(Convert.ToSingle(s[14 + i]) * 58);
                u[i].Level = (byte)(Convert.ToSingle(s[18 + i]) * 255 / 3.3);
            }
            v.SaveTime = Convert.ToByte(s[1]);
            v.WheelSpeedRearLeftData  = Convert.ToSingle(s[2]);
            v.WheelSpeedRearRightData = Convert.ToSingle(s[2]);
            v.SteeringAngleActual = Convert.ToSingle(s[5]);
        }
        #endregion

        #region 终端控制命令
        private void TerminalControlCommandCAN(byte cmd)
        {
            uint id = 0x530;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = cmd;
            dat[1] = 0;
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 超声波检车位
        private void ParkingDetectionCommandCAN()
        {
            uint id = 0x530;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = 0xA1;
            dat[1] = 0;
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 轨迹规划信息
        /// <summary>
        /// 规划开始控制命令
        /// </summary>
        private void PlanningCommandCAN(byte cmd)
        {
            uint id = 0x532;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = cmd;
            dat[1] = 0;        
            dat[2] = 0;
            dat[3] = 0;    
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        /// <summary>
        /// 车身初始位置信息
        /// </summary>
        private void VehicleInitPositionCAN()
        {
            uint id = 0x540;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存
            float parking_length, parking_width;
            double yaw;
            parking_length = Convert.ToSingle(textBox16.Text);
            parking_width = Convert.ToSingle(textBox17.Text);
            yaw = Convert.ToSingle(textBox15.Text) * Math.PI / 180;
            Data_Temp = BitConverter.GetBytes((Int16)( (parking_length + Convert.ToSingle(textBox13.Text) + RIGHT_EDGE_TO_CENTER * Math.Sin(yaw)) * 100));//车辆初始位置X轴向
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((Int16)( (Convert.ToSingle(textBox14.Text) + RIGHT_EDGE_TO_CENTER * Math.Cos(yaw)) * 100));//车辆初始位置Y轴向
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((Int16)(yaw * 100));//车辆初始偏航姿态角
            dat[4] = Data_Temp[0];
            dat[5] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox18.Text) * 100));//车辆侧移余量
            dat[6] = Data_Temp[0];
            dat[7] = Data_Temp[1];
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        /// <summary>
        /// 车位信息
        /// </summary>
        private void ParkingInformationCAN()
        {
            uint id = 0x541;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox16.Text) * 1000));//车位长
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox17.Text) * 1000));//车位宽
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox19.Text) * 1000));//车前余量
            dat[4] = Data_Temp[0];
            dat[5] = Data_Temp[1];
            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox20.Text) * 1000));//车后余量
            dat[6] = Data_Temp[0];
            dat[7] = Data_Temp[1];
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 嵌入式参数配置
        private void ParameterConfigureCAN()
        {
            uint id = 0x560;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = (byte)(Convert.ToSingle(textBox22.Text) * 10);
            dat[1] = (byte)(Convert.ToSingle(textBox23.Text) * 10);
            dat[2] = (byte)(Convert.ToSingle(textBox24.Text) * 100);
            dat[3] = (byte)(Convert.ToSingle(textBox25.Text) * 100);
            dat[4] = (byte)(Convert.ToSingle(textBox26.Text) * 100);
            dat[5] = (byte)(Convert.ToSingle(textBox27.Text) * 100);
            dat[6] = (byte)(Convert.ToSingle(textBox28.Text) * 100);
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
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
            uint id = 0x516;
            byte len = 8;
            byte CheckSum;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            dat[0] = (byte)(
                              (Convert.ToByte(checkBox6.Checked) << 3)// Velocity 
                            | (Convert.ToByte(checkBox3.Checked) << 5)// Torque 
                            | (Convert.ToByte(checkBox2.Checked) << 4)// AEB 
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
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        private void ChangAnInterfaceCAN2()
        {
            uint id = 0x517;
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
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 终端解码函数
        private delegate void TerminalParse(ZLGCAN.VCI_CAN_OBJ m_packet);
        private void UltrasonicParse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            TerminalParse m_sampling = new TerminalParse(Parse);
            this.Invoke(m_sampling, new object[] { m_packet });
        }
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
                    m_LIN_STP318_ReadData[m_packet.ID & 0x007].TOF = BitConverter.ToUInt16(tmp_dat, 0);
                    m_LIN_STP318_ReadData[m_packet.ID & 0x007].status = m_packet.Data[6];

                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance1 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].status = m_packet.Data[6];
                    break;

                case 0x408://传感器9
                case 0x409://传感器10
                case 0x40A://传感器11
                case 0x40B://传感器12
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF1 = BitConverter.ToUInt16(tmp_dat, 0);
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance1 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF2 = BitConverter.ToUInt16(tmp_dat, 0);
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance2 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].Level = m_packet.Data[4];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].Width = m_packet.Data[5];
                    m_LIN_STP313_ReadData[m_packet.ID & 0x003].status = m_packet.Data[6];
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Level = m_packet.Data[4] * 0.1f;
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Width = m_packet.Data[5];
                    //m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].status = m_packet.Data[6];
                    break;

                case 0x416:
                    if (m_packet.Data[1] == 0xA5)
                    {
                        AckId = m_packet.Data[0];
                        AckCnt = 0;
                        timer_ack.Enabled = true;
                    }
                    break;

                case 0x410:
                    m_Vehicle.EMSQECACC         = Convert.ToBoolean((m_packet.Data[0] >> 2) & 0x01);
                    m_Vehicle.ESPQDCACC         = Convert.ToBoolean((m_packet.Data[0] >> 1) & 0x01);
                    m_Vehicle.APA_EpasFailed    = Convert.ToBoolean( m_packet.Data[0]  & 0x01);

                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.SteeringAngleActual = BitConverter.ToInt16(tmp_dat,0) * 0.1;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.SteeringAngleSpeed = (UInt16)(BitConverter.ToUInt16(tmp_dat, 0) * 0.01);
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    m_Vehicle.SteeringTorque = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    break;

                case 0x411:
                    m_Vehicle.TargetAccelerationEnable = Convert.ToBoolean( m_packet.Data[0]       & 0x01);
                    m_Vehicle.TargetDecelerationEnable = Convert.ToBoolean((m_packet.Data[0] >> 1) & 0x01);
                    m_Vehicle.TorqueEnable             = Convert.ToBoolean((m_packet.Data[0] >> 2) & 0x01);
                    m_Vehicle.VelocityEnable           = Convert.ToBoolean((m_packet.Data[0] >> 3) & 0x01);
                    m_Vehicle.SteeringAngleActive      = Convert.ToByte   ((m_packet.Data[0] >> 4) & 0x03);
                    m_Vehicle.GearShiftEnable          = Convert.ToBoolean((m_packet.Data[0] >> 6) & 0x01);

                    m_Vehicle.GearShift = m_packet.Data[1];
                    if(4 == m_Vehicle.GearShift)
                    {
                        m_Vehicle.WheelSpeedRearRightDirection = 0;
                    }
                    else if (2 == m_Vehicle.GearShift)
                    {
                        m_Vehicle.WheelSpeedRearRightDirection = 1;
                    }
                    else
                    {
                        m_Vehicle.WheelSpeedRearRightDirection = 2;
                    }
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.TargetAccelerationACC = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.TargetDecelerationAEB = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    if(m_Vehicle.VelocityEnable)
                    {
                        m_Vehicle.WheelSpeedRearLeftData = BitConverter.ToUInt16(tmp_dat, 0) * 0.01;
                        m_Vehicle.WheelSpeedRearRightData = m_Vehicle.WheelSpeedRearLeftData;
                    }
                    else
                    {
                        m_Vehicle.WheelSpeedRearLeftData = 0;
                        m_Vehicle.WheelSpeedRearRightData = 0;
                        m_Vehicle.WheelSpeedRearRightDirection = 2;
                    }
                    break;

                case 0x440://反馈的车辆初始中心点位置
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    VehicleInitPosition.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    VehicleInitPosition.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    VehicleInitPosition.Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    DetectorStatus = m_packet.Data[6];
                    TrackPoint = VehicleInitPosition;
                    track_update_status = 0xa5;
                    break;

                case 0x441://车位信息
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    ParkingLength = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    ParkingWidth = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    FrontMarginBoundary = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    RearMarginBoundary = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    parking_update_status = 0xa5;
                    ParkingInformationDataLog();
                    break;

                case 0x442://跟踪轨迹
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    TrackPoint.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    TrackPoint.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    TrackPoint.Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    track_update_status = 0xa5;
                    break;

                case 0x443://前向实验记录
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    FrontTrial[m_packet.Data[6]].Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    FrontTrial[m_packet.Data[6]].Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    FrontTrial[m_packet.Data[6]].Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    break;

                case 0x444://后向实验记录
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    RearTrial[m_packet.Data[6]].Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    RearTrial[m_packet.Data[6]].Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    RearTrial[m_packet.Data[6]].Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    break;

                case 0x445://车位进库点位置
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    ParkingEnterPosition.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    ParkingEnterPosition.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    ParkingEnterPosition.Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    TrialCnt = m_packet.Data[6];
                    if(0x5A == m_packet.Data[7])
                    {
                        TrackListBox.Items.Add("车位满足泊车条件");
                        TrackListBox.Items.Add("尝试次数为：" + TrialCnt.ToString());
                    }
                    else
                    {
                        TrackListBox.Items.Add("请重新选择车位！");
                    }
                    vehicle_update_status = 0xa5;
                    break;

                case 0x446:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    TurnningPointNumber = m_packet.Data[6];
                    TurnningPointArrary[TurnningPointNumber].Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    TurnningPointArrary[TurnningPointNumber].Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    TurnningPointArrary[TurnningPointNumber].SteeringAngle = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    vehicle_update_status = 0xa6;
                    break;

                case 0x447:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    ParkingCenterPoint.X = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    ParkingCenterPoint.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    
                    TrackListBox.Items.Add("进库中心点：");
                    TrackListBox.Items.Add("X:" + ParkingCenterPoint.X.ToString());
                    TrackListBox.Items.Add("Y:" + ParkingCenterPoint.Y.ToString());
                    break;

                case 0x448:
                    ParkingStatus = m_packet.Data[0];
                    break;

                default:

                    break;
            }
        }
        #endregion

        #region 模拟车辆接收解码
        unsafe void VehicleParse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            byte[] tmp_dat = new byte[2] { 0, 0 };
            byte[] dat = new byte[8];
            switch (m_packet.ID)
            {
                case 0x6fe:
                    tmp_dat[1] = m_packet.Data[6];
                    tmp_dat[0] = m_packet.Data[7];
                    m_Vehicle.SteeringAngle = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    break;

                case 0x6ff:

                    break;

                case 0x6e9:

                    break;

                default:

                    break;
            }
        }
        #endregion

        #region 实际车辆信息
        unsafe void VehicleSendParse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            byte[] tmp_dat = new byte[2] { 0, 0 };
            byte[] dat = new byte[8];
            switch (m_packet.ID)
            {
                case 0x208:
                    m_Vehicle.WheelSpeedDirection = (byte)((m_packet.Data[0] >> 5) & 0x03);
                    m_Vehicle.WheelSpeedRearRightData = (UInt16)(((m_packet.Data[0] & 0x1F) << 8) | m_packet.Data[1]) * 0.015625;
                    m_Vehicle.WheelSpeedRearLeftData  = (UInt16)(((m_packet.Data[2] & 0x1F) << 8) | m_packet.Data[3]) * 0.015625;

                    m_Vehicle.VehicleSpeed = (m_Vehicle.WheelSpeedRearRightData + m_Vehicle.WheelSpeedRearLeftData) * 0.5;

                    m_Vehicle.VehicleSpeed = m_Vehicle.WheelSpeedDirection == 0 ?  m_Vehicle.VehicleSpeed :
                                             m_Vehicle.WheelSpeedDirection == 1 ? -m_Vehicle.VehicleSpeed : 0;
                    break;

                case 0x278:
                    m_Vehicle.LatAcc = m_packet.Data[2] * 0.1 - 12.7;
                    m_Vehicle.LonAcc = (m_packet.Data[3] << 2 | m_packet.Data[4] >> 6) * 0.03125 - 16;
                    m_Vehicle.YawRate = ((m_packet.Data[4] & 0x3f) << 8 | m_packet.Data[5]) * 0.01 - 81.91;
                    break;

                default:

                    break;
            }
        }
        #endregion

        #region 模拟车辆发送数据编码
        /// <summary>
        /// 模拟车辆发送速度信息
        /// </summary>
        private void EPB_VehicleSpeedSimulation()
        {
            uint id = 0x208;
            byte len = 8;
            UInt16 temp_speed;
            byte[] dat = new byte[8];

            temp_speed = (UInt16)(m_Vehicle.WheelSpeedRearRightData * 64);

            dat[0] = (byte)(m_Vehicle.WheelSpeedRearRightDirection << 5 | ((temp_speed >> 8) & 0x1f));
            dat[1] = (byte)(temp_speed & 0xff);
            dat[2] = dat[0];
            dat[3] = dat[1];
            dat[4] = dat[0];
            dat[5] = dat[1];
            dat[6] = dat[0];
            dat[7] = dat[1];
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
        }

        /// <summary>
        /// 模拟车辆发送转向角信息
        /// </summary>
        private void SAS_SteeringAngleSimulation()
        {
            uint id = 0x180;
            byte len = 8;
            byte[] dat = new byte[8];
            Int16 temp_steering;
            temp_steering = (Int16)(m_Vehicle.SteeringAngle * 10);

            dat[0] = (byte)(temp_steering >> 8);
            dat[1] = (byte)(temp_steering & 0xff);
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
        }

        /// <summary>
        /// 模拟车辆挡位信息
        /// </summary>
        private void TCU_GearSimulation()
        {
            uint id = 0x268;
            byte len = 8;
            byte[] dat = new byte[8];

            dat[0] = 0;
            dat[1] = (m_Vehicle.GearShift == 1) ? (byte)0x0A :
                     (m_Vehicle.GearShift == 2) ? (byte)0x09 :
                     (m_Vehicle.GearShift == 3) ? (byte)0x00 : (byte)0x01;
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
        }
        #endregion

        #region 窗口显示函数
        /// <summary>
        /// 车辆信息显示
        /// </summary>
        private void VehicleImformationShow()
        {
            try
            {
                label4.Text = m_SerialCom.ErrCount.ToString();//帧错计数

                label5.ForeColor = m_Vehicle.EPS_Failed ? Color.AliceBlue : Color.BlueViolet;
                label6.ForeColor = m_Vehicle.ESPQDCACC ? Color.AliceBlue : Color.BlueViolet;
                label7.ForeColor = m_Vehicle.EMSQECACC ? Color.AliceBlue : Color.BlueViolet;

                label52.ForeColor  = m_Vehicle.TargetAccelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                label53.ForeColor  = m_Vehicle.TargetDecelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                label54.ForeColor  = m_Vehicle.TorqueEnable             ? Color.DarkGoldenrod : Color.AliceBlue;
                label55.ForeColor  = m_Vehicle.GearShiftEnable          ? Color.DarkGoldenrod : Color.AliceBlue;
                label108.ForeColor = m_Vehicle.VelocityEnable           ? Color.DarkGoldenrod : Color.AliceBlue;

                label59.Text = SteeringAngleActiveStatus[m_Vehicle.SteeringAngleActive];
                label60.Text = m_Vehicle.TargetAccelerationACC.ToString();
                label61.Text = m_Vehicle.TargetDecelerationAEB.ToString();
                label107.Text = m_Vehicle.Torque.ToString();

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

        /// <summary>
        /// 超声波数据数值显示
        /// </summary>
        private void UltrasonicImformationShow()
        {
            for(int i =0;i<8;i++ )
            {
                m_Ultrasonic.DataMapping2Control_STP318(m_LIN_STP318_ReadData[i], ref SensingControl_SRU[i]);
                //m_Ultrasonic.DataMapping2Control_STP318Packet(m_Ultrasonic_Data_Packet[i], ref SensingControl_SRU[i]);
            }
            for (int i = 0; i < 4; i++)
            {
                m_Ultrasonic.DataMapping2Control_STP313(m_LIN_STP313_ReadData[i], ref SensingControl_LRU[i]);
            }
            //for (int i = 8; i < 12; i++)
            //{
            //    m_Ultrasonic.DataMapping2Control_STP313Packet(m_Ultrasonic_Data_Packet[i], ref SensingControl_LRU[i - 8]);
            //}
        }

        private void UltrasonicImformationFormShow()
        {
            if (checkBox8.Checked)
            {
                //UltrasonicDataShow.Points.AddY(m_Ultrasonic_Data_Packet[9].Distance1*100);
                UltrasonicDataShow.Points.AddY(m_LIN_STP313_ReadData[1].TOF1 / 58);
                while (UltrasonicDataShow.Points.Count > 500)
                {
                    UltrasonicDataShow.Points.RemoveAt(0);
                }
            }
        }
        /// <summary>
        /// 跟踪轨迹显示
        /// </summary>
        private void TrackInformationShow()
        {
            if(0xa5 == track_update_status)
            {
                track_update_status = 0x00;
                label120.Text = TrackPoint.Position.X.ToString();
                label121.Text = TrackPoint.Position.Y.ToString();
                label122.Text = TrackPoint.Yaw.ToString();

                TrackDataShow.Points.AddXY(TrackPoint.Position.X, TrackPoint.Position.Y);

                AxisRotation(TrackPoint, FrontLeftDiagonal, ref VehicleEdgePoint[0]);
                AxisRotation(TrackPoint, FrontRightDiagonal, ref VehicleEdgePoint[1]);
                AxisRotation(TrackPoint, RearLeftDiagonal, ref VehicleEdgePoint[3]);
                AxisRotation(TrackPoint, RearRightDiagonal, ref VehicleEdgePoint[2]);
                TrackEdgeDataShow.Points.Clear();
                for (int i = 0; i < 4; i++)
                {
                    TrackEdgeDataShow.Points.AddXY(VehicleEdgePoint[i].X, VehicleEdgePoint[i].Y);
                }
                TrackEdgeDataShow.Points.AddXY(VehicleEdgePoint[0].X, VehicleEdgePoint[0].Y);
            }
        }

        /// <summary>
        /// 车位显示
        /// </summary>
        private void ParkingShow()
        {
            if(0xa5 == parking_update_status)
            {
                parking_update_status = 0;
                ParkingDataShow.Points.Clear();
                ParkingDataShow.Points.AddXY(-5, 0);
                ParkingDataShow.Points.AddXY(0, 0);
                ParkingDataShow.Points.AddXY(0, -ParkingWidth);
                ParkingDataShow.Points.AddXY(ParkingLength, -ParkingWidth);
                ParkingDataShow.Points.AddXY(ParkingLength, 0);
                ParkingDataShow.Points.AddXY(18, 0);

                label124.Text = "X:" + VehicleInitPosition.Position.X.ToString() + "m";
                label125.Text = "Y:" + VehicleInitPosition.Position.Y.ToString() + "m";
                label126.Text = "Yaw:" + (VehicleInitPosition.Yaw * 57.3).ToString() + "°";

                label127.Text = "L:" + ParkingLength.ToString() + "m";
                label128.Text = "W:" + ParkingWidth.ToString() + "m";

                label129.Text = (DetectorStatus < 4 ? ParkingDetecterStatus[DetectorStatus] : "异常");
            }
        }

        /// <summary>
        /// 车辆车身信息
        /// </summary>
        /// <param name="c"></param>
        private void VehicleShow(LocationPoint c)
        {
            label123.Text = TrialCnt.ToString();
            VehicleModuleShow.Points.Clear();
            AxisRotation(c, FrontLeftDiagonal, ref VehicleEdgePoint[0]);
            AxisRotation(c, FrontRightDiagonal, ref VehicleEdgePoint[1]);
            AxisRotation(c, RearLeftDiagonal, ref VehicleEdgePoint[3]);
            AxisRotation(c, RearRightDiagonal, ref VehicleEdgePoint[2]);
            for (int i = 0; i < 4; i++)
            {
                VehicleModuleShow.Points.AddXY(VehicleEdgePoint[i].X, VehicleEdgePoint[i].Y);
            }
            VehicleModuleShow.Points.AddXY(VehicleEdgePoint[0].X, VehicleEdgePoint[0].Y);
        }

        private void TurnningAngleShiftShow()
        {
            if(vehicle_update_status == 0xa6)
            {
                vehicle_update_status = 0;
                TurnningPointShow.Points.Clear();
                for (int i = 0; i < (TurnningPointNumber + 1); i++)
                {
                    TurnningPointShow.Points.AddXY(TurnningPointArrary[i].Position.X,
                                                   TurnningPointArrary[i].Position.Y);
                    TrackListBox.Items.Add("第" + i.ToString() + "转向点:" );
                    TrackListBox.Items.Add("X:" + TurnningPointArrary[i].Position.X.ToString());
                    TrackListBox.Items.Add("Y:" + TurnningPointArrary[i].Position.Y.ToString());
                }
            }
            
        }
        private void ParkingControlShow()
        {
            label133.Text = ParkingStatus < 4 ? ParkingPlanningStatus[ParkingStatus]:"异常";
            label135.Text = "转向角:" + m_Vehicle.SteeringAngleActual.ToString();
            label136.Text = "轮速:" + (0.5 * (m_Vehicle.WheelSpeedRearLeftData + m_Vehicle.WheelSpeedRearRightData)).ToString();
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

        /// <summary>
        /// 检车位专用的数据保存
        /// </summary>
        private void ParkingDetectionDataLog()
        {
            ErrTime = timeGetTime() - LastTime;
            //数据保存
            if (u_DataSaveStatus)
            {
                ULtrasonicDataSave.Write(
                                "{0:D} " +
                                "{1:R16} {2:D} {3:R16} {4:R16} " +
                                "{5:R16} {6:R16} {7:R16} {8:R16} " +
                                "{9:D} {10:D} {11:D} {12:D} {13:D} " +
                                "{14:D} {15:D} {16:D} {17:D} {18:D} " +
                                "{19:D} {20:D} {21:D} {22:D} {23:D} " +
                                "{24:D} {25:D} {26:D} {27:D} {28:D} " +
                                "{29:D} {30:D} {31:D} {32:D} {33:D} " +
                                "\r\n",
                                ErrTime,
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
                                // Ultrasonic
                                m_LIN_STP313_ReadData[0].TOF1,
                                m_LIN_STP313_ReadData[0].TOF2,
                                m_LIN_STP313_ReadData[0].Level,
                                m_LIN_STP313_ReadData[0].Width,
                                m_LIN_STP313_ReadData[0].status,
                                m_LIN_STP313_ReadData[1].TOF1,
                                m_LIN_STP313_ReadData[1].TOF2,
                                m_LIN_STP313_ReadData[1].Level,
                                m_LIN_STP313_ReadData[1].Width,
                                m_LIN_STP313_ReadData[1].status,
                                m_LIN_STP313_ReadData[2].TOF1,
                                m_LIN_STP313_ReadData[2].TOF2,
                                m_LIN_STP313_ReadData[2].Level,
                                m_LIN_STP313_ReadData[2].Width,
                                m_LIN_STP313_ReadData[2].status,
                                m_LIN_STP313_ReadData[3].TOF1,
                                m_LIN_STP313_ReadData[3].TOF2,
                                m_LIN_STP313_ReadData[3].Level,
                                m_LIN_STP313_ReadData[3].Width,
                                m_LIN_STP313_ReadData[3].status
                );
            }
            LastTime = timeGetTime();
        }

        /// <summary>
        /// 数据包保存
        /// </summary>
        private void ParkingDetectionPacketDataLog()
        {
            ErrTime = timeGetTime() - LastTime;
            //数据保存
            if (u_DataSaveStatus)
            {
                ULtrasonicDataSave.Write(
                                "{0:D} " +
                                "{1:R16} {2:D} {3:R16} {4:R16} " +
                                "{5:R16} {6:R16} {7:R16} {8:R16} " +
                                "{9:D} {10:D} {11:D} {12:D} {13:D} " +
                                "{14:R16} {15:R16} {16:R16} {17:R16} {18:R16} " +
                                "{19:R16} {20:R16} {21:R16} {22:R16} {23:R16} " +
                                "{24:R16} {25:R16} {26:R16} {27:R16} {28:R16} " +
                                "{29:R16} {30:R16} {31:R16} {32:R16} {33:R16} " +
                                "\r\n",
                                ErrTime,
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
                                // Ultrasonic
                                m_Ultrasonic_Data_Packet[8].Distance1,
                                m_Ultrasonic_Data_Packet[8].Distance2,
                                m_Ultrasonic_Data_Packet[8].Level,
                                m_Ultrasonic_Data_Packet[8].Width,
                                m_Ultrasonic_Data_Packet[8].status,
                                m_Ultrasonic_Data_Packet[9].Distance1,
                                m_Ultrasonic_Data_Packet[9].Distance2,
                                m_Ultrasonic_Data_Packet[9].Level,
                                m_Ultrasonic_Data_Packet[9].Width,
                                m_Ultrasonic_Data_Packet[9].status,
                                m_Ultrasonic_Data_Packet[10].Distance1,
                                m_Ultrasonic_Data_Packet[10].Distance2,
                                m_Ultrasonic_Data_Packet[10].Level,
                                m_Ultrasonic_Data_Packet[10].Width,
                                m_Ultrasonic_Data_Packet[10].status,
                                m_Ultrasonic_Data_Packet[11].Distance1,
                                m_Ultrasonic_Data_Packet[11].Distance2,
                                m_Ultrasonic_Data_Packet[11].Level,
                                m_Ultrasonic_Data_Packet[11].Width,
                                m_Ultrasonic_Data_Packet[11].status
                );
            }
            LastTime = timeGetTime();
        }
        /// <summary>
        /// 库位信息保存
        /// </summary>
        private void ParkingInformationDataLog()
        {
            //数据保存
            if (u_DataSaveStatus)
            {
                ParkingDataSave.Write(
                                "{0:R16} {1:R16} " +
                                "{2:R16} {3:R16} {4:R16} "+                       
                                "\r\n",
                                ParkingLength,
                                ParkingWidth,
                                VehicleInitPosition.Position.X,
                                VehicleInitPosition.Position.Y,
                                VehicleInitPosition.Yaw
                );
            }
            
        }
        #endregion

        #region utils math

        private void AxisRotation(LocationPoint c, Polar p,ref Vector2d v)
        {
            v.X = c.Position.X + p.Length * Math.Cos(c.Yaw + p.Angle);
            v.Y = c.Position.Y + p.Length * Math.Sin(c.Yaw + p.Angle);
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
            ///超声波数据显示
            ultrasonic_chart.ChartAreas[0].AxisY.Maximum = 400;
            ultrasonic_chart.ChartAreas[0].AxisY.Minimum = 0;
            ultrasonic_chart.Series.Add(UltrasonicDataShow);
            UltrasonicDataShow.ChartType = SeriesChartType.Point;
            UltrasonicDataShow.BorderWidth = 5;
            UltrasonicDataShow.BorderDashStyle = ChartDashStyle.Dash;
            UltrasonicDataShow.Color = Color.Green;
            UltrasonicDataShow.IsVisibleInLegend = true;
            UltrasonicDataShow.LegendText = "超声波数据";

            /// 车辆泊车图形显示
            track_chart.ChartAreas[0].AxisX.Maximum = 12;
            track_chart.ChartAreas[0].AxisX.Minimum = -1;
            track_chart.ChartAreas[0].AxisY.Maximum = 5;
            track_chart.ChartAreas[0].AxisY.Minimum = -3;
            track_chart.ChartAreas[0].AxisX.Interval = 0.5;
            track_chart.ChartAreas[0].AxisY.Interval = 0.5;

            track_chart.Series.Add(TrackDataShow);
            TrackDataShow.ChartType = SeriesChartType.FastLine;
            TrackDataShow.BorderWidth = 2;
            TrackDataShow.BorderDashStyle = ChartDashStyle.Dash;
            TrackDataShow.Color = Color.Red;
            TrackDataShow.IsVisibleInLegend = true;
            TrackDataShow.LegendText = "车辆中心点";

            track_chart.Series.Add(ParkingDataShow);
            ParkingDataShow.ChartType = SeriesChartType.FastLine;
            ParkingDataShow.BorderWidth = 5;
            ParkingDataShow.BorderDashStyle = ChartDashStyle.Solid;
            ParkingDataShow.Color = Color.Green;
            ParkingDataShow.IsVisibleInLegend = true;
            ParkingDataShow.LegendText = "车位";

            
            track_chart.Series.Add(TrackEdgeDataShow);
            TrackEdgeDataShow.ChartType = SeriesChartType.FastLine;
            TrackEdgeDataShow.BorderWidth = 3;
            TrackEdgeDataShow.BorderDashStyle = ChartDashStyle.Solid;
            TrackEdgeDataShow.Color = Color.DarkRed;
            TrackEdgeDataShow.IsVisibleInLegend = true;
            TrackEdgeDataShow.LegendText = "车辆实时姿态";

            //track_chart.Series.Add(VehicleModuleShow);
            VehicleModuleShow.ChartType = SeriesChartType.FastLine;
            VehicleModuleShow.BorderWidth = 3;
            VehicleModuleShow.BorderDashStyle = ChartDashStyle.Solid;
            VehicleModuleShow.Color = Color.Blue;
            VehicleModuleShow.IsVisibleInLegend = true;
            VehicleModuleShow.LegendText = "车辆模型";
          
            track_chart.Series.Add(TurnningPointShow);
            TurnningPointShow.ChartType = SeriesChartType.FastPoint;
            TurnningPointShow.BorderWidth = 16;
            TurnningPointShow.BorderDashStyle = ChartDashStyle.Solid;
            TurnningPointShow.Color = Color.DarkGreen;
            TurnningPointShow.IsVisibleInLegend = true;
            TurnningPointShow.LegendText = "转向角切换点";
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

            for (int i = 0; i < 4; i++)
            {
                comboBox6.Items.Add(ParkingModule[i]);
            }
            comboBox6.SelectedIndex = 0;
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
            /// 车辆初始参数计算
            /// 
            FrontLeftDiagonal.Length = Math.Sqrt( Math.Pow(LEFT_EDGE_TO_CENTER, 2) + Math.Pow(FRONT_EDGE_TO_CENTER,2));
            FrontLeftDiagonal.Angle  = Math.Atan(LEFT_EDGE_TO_CENTER/ FRONT_EDGE_TO_CENTER);
            FrontRightDiagonal.Length = FrontLeftDiagonal.Length;
            FrontRightDiagonal.Angle = -FrontLeftDiagonal.Angle;


            RearLeftDiagonal.Length = -Math.Sqrt(Math.Pow(LEFT_EDGE_TO_CENTER, 2) + Math.Pow(REAR_EDGE_TO_CENTER, 2)); ;
            RearLeftDiagonal.Angle  = -Math.Atan(LEFT_EDGE_TO_CENTER / REAR_EDGE_TO_CENTER);

            RearRightDiagonal.Length =  RearLeftDiagonal.Length;
            RearRightDiagonal.Angle  = -RearLeftDiagonal.Angle;
            ///车位信息的初始化
            ParkingLength = Convert.ToDouble(textBox16.Text);
            ParkingWidth  = Convert.ToDouble(textBox17.Text);
            VehicleInitPosition.Position.X   = Convert.ToDouble(textBox13.Text) + ParkingLength + m_Vehicle.RearOverhangDistance;
            VehicleInitPosition.Position.Y   = Convert.ToDouble(textBox14.Text) + m_Vehicle.WheelAxisWidthHalf;
            VehicleInitPosition.Yaw = Convert.ToDouble(textBox15.Text);
            LatMarginMove       = Convert.ToDouble(textBox18.Text);
            FrontMarginBoundary = Convert.ToDouble(textBox19.Text);
            RearMarginBoundary  = Convert.ToDouble(textBox20.Text);

            ParkingShow();
            VehicleShow(VehicleInitPosition);
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
                    m_ZLGCAN.CAN_Receive(TerminalCAN, ref obj);
                    for(int i=0;i< obj.Length-1;i++)
                    {
                        UltrasonicParse(obj[i]);
                    }
                    m_ZLGCAN.CAN_Receive(VehicleReceiveCAN, ref obj);
                    for (int i = 0; i < obj.Length-1; i++)
                    {
                        VehicleParse(obj[i]);
                    }
                    m_ZLGCAN.CAN_Receive(VehicleSendCAN, ref obj);
                    for (int i = 0; i < obj.Length; i++)
                    {

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
            if(1 == m_ZLGCAN.ConnectStatus)
            {
                m_ZLGCAN.CAN_Close();
            }
            else
            {
                m_ZLGCAN.CAN_Connect();
            }
            button11.Text = m_ZLGCAN.ConnectStatus == 0 ? "连接" : "断开";
        }
        /// <summary>
        /// 打开CAN设备
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button12_Click(object sender, EventArgs e)
        {
            if (0 == m_ZLGCAN.CAN_Open(0) && 0 == m_ZLGCAN.CAN_Open(1) && 0 == m_ZLGCAN.CAN_Open(2))
            {
                m_ZLGCAN.OpenStatus = 1;
            }
            else
            {
                m_ZLGCAN.OpenStatus = 0;
            }
            
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
            m_ZLGCAN.CAN_Reset(2);
            m_ZLGCAN.OpenStatus = 0;
        }


        #endregion

        #region 定时器事件
        private void timer_show_Tick(object sender, EventArgs e)
        {
            if(checkBox7.Checked)
            {
                EPB_VehicleSpeedSimulation();
                SAS_SteeringAngleSimulation();
                TCU_GearSimulation();
            }
            ParkingControlShow();
            VehicleImformationShow();
            UltrasonicImformationShow();
            UltrasonicImformationFormShow();
            TrackInformationShow();
            ParkingShow();
            TurnningAngleShiftShow();
            if (0xa5 == vehicle_update_status)
            {
                vehicle_update_status = 0;
                VehicleShow(ParkingEnterPosition);
            }
            ParkingDetectionDataLog();
        }

        /// <summary>
        /// 超声波数据定时注入
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void InjectionUltrasonicTimer_Tick(object sender, EventArgs e)
        {
            string line;
            if((line = UltrasonicDataLoad.ReadLine() ) != null)
            {
                string[] s = line.Split(' ');

                FileDataParse(s,ref m_LIN_STP313_ReadData,ref m_Vehicle);
                
                UltrasonicDataShow.Points.AddY(m_LIN_STP313_ReadData[1].TOF1 / 58);
                //while (UltrasonicDataShow.Points.Count > 100)
                //{
                //    UltrasonicDataShow.Points.RemoveAt(0);
                //}
                UltrasonicCAN(9, m_LIN_STP313_ReadData[1]);
                VehicleVelocityCAN(m_Vehicle);
                VehicleStatusCAN(m_Vehicle);
                progressBar1.Value++;
            }
            else
            {
                UltrasonicDataLoad.Close();
                InjectionUltrasonicTimer.Enabled = false;
            }
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

        #region 检车位相关数据保存
        /// <summary>
        /// 超声检测车位的保存路径选择
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button26_Click(object sender, EventArgs e)
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
                u_localFilePath = saveFileDialog1.FileName.ToString();
                //获取文件路径，不带文件名
                u_FilePath = u_localFilePath.Substring(0, u_localFilePath.LastIndexOf("\\"));
                //获取文件名，不带路径
                u_fileNameExt = u_localFilePath.Substring(u_localFilePath.LastIndexOf("\\") + 1);
            }
        }

        /// <summary>
        /// 检车位数据保存
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button27_Click(object sender, EventArgs e)
        {
            if (!u_DataSaveStatus)
            {
                //给文件名前加上时间
                u_newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox21.Text + "_" + u_fileNameExt;
                ULtrasonicDataSave = new StreamWriter(u_FilePath + "\\" + u_newFileName, true, Encoding.ASCII);
                u_newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox21.Text + "_ParkingInf_" + u_fileNameExt;
                ParkingDataSave    = new StreamWriter(u_FilePath + "\\" + u_newFileName, true, Encoding.ASCII);
                u_DataSaveStatus   = true;
            }
            else
            {
                ULtrasonicDataSave.Close();
                ParkingDataSave.Close();
                u_DataSaveStatus = false;
            }
            button27.Text = u_DataSaveStatus ? "取消保存" : "开始保存";
            button27.BackColor = u_DataSaveStatus ? Color.Green : Color.Red;
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

        #region 超声波数据注入
        private void button14_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
            openFileDialog1.InitialDirectory = "D:";
            openFileDialog1.Title = "请选择加载文件路径";
            openFileDialog1.FilterIndex = 0;
            openFileDialog1.RestoreDirectory = true;
            //saveFileDialog1.DefaultExt = "txt";
            //saveFileDialog1.FileName = "Data.txt";
            //saveFileDialog1.AddExtension = true;
            DialogResult dr = openFileDialog1.ShowDialog();
            if (dr == DialogResult.OK && openFileDialog1.FileName.Length > 0)
            {
                //获得文件路径
                LoadFilePath = openFileDialog1.FileName.ToString();
                try
                {
                    using (StreamReader sr = new StreamReader(LoadFilePath, Encoding.ASCII))
                    {
                        FileLineCount = sr.ReadToEnd().Split('\n').Length;
                    }
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
                try
                {
                    UltrasonicDataLoad = new StreamReader(LoadFilePath, Encoding.ASCII);
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
                progressBar1.Maximum = FileLineCount-1;
                progressBar1.Value = 0;
                UltrasonicDataShow.Points.Clear();
                //CurrentLine = 0;
                InjectionUltrasonicTimer.Enabled = true;
                ////获取文件路径，不带文件名
                //FilePath = localFilePath.Substring(0, localFilePath.LastIndexOf("\\"));
                ////获取文件名，不带路径
                //fileNameExt = localFilePath.Substring(localFilePath.LastIndexOf("\\") + 1);


                //StreamReader UltrasonicDataLoad;
                //string LoadFilePath;/* localFilePath, newFileName, fileNameExt;*/
                //bool DataLoadStatus = false;
            }
        }

        /// <summary>
        /// 超声数据注入继续
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button15_Click(object sender, EventArgs e)
        {
            InjectionUltrasonicTimer.Enabled = true;
        }

        /// <summary>
        /// 超声波数据注入暂停
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button16_Click(object sender, EventArgs e)
        {
            InjectionUltrasonicTimer.Enabled = false;
        }

        private void button17_Click(object sender, EventArgs e)
        {
            UltrasonicDataShow.Points.Clear();
        }
        #endregion

        #region 规划状态显示控制
        /// <summary>
        /// 图像清零
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button18_Click(object sender, EventArgs e)
        {
            TrackDataShow.Points.Clear();
        }

        /// <summary>
        /// 切换上一帧
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button20_Click(object sender, EventArgs e)
        {
            if ( TrialPoint <= 0)
            {
                TrialPoint = 0;
            }
            else
            {
                TrialPoint--;
            }
            if(TrialCnt % 2 == 1)
            {
                VehicleShow(FrontTrial[TrialPoint]);
            }
            else
            {
                VehicleShow(RearTrial[TrialPoint]);
            }
        }

        /// <summary>
        /// 切换下一帧
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button21_Click(object sender, EventArgs e)
        {
            if (TrialPoint >= TrialCnt)
            {
                TrialPoint = TrialCnt;
            }
            else
            {
                TrialPoint++;
            }
            if (TrialCnt % 2 == 1)
            {
                VehicleShow(FrontTrial[TrialPoint]);
            }
            else
            {
                VehicleShow(RearTrial[TrialPoint]);
            }
        }
        #endregion

        #region 车位检车事件
        private void button23_Click(object sender, EventArgs e)
        {
            TerminalControlCommandCAN(0xA0);
            //ParkingDetectionCommandCAN();
        }

        private void button28_Click(object sender, EventArgs e)
        {
            TerminalControlCommandCAN(0xB0);
        }
        #endregion

        #region 泊车事件
        /// <summary>
        /// 泊车开始
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button19_Click(object sender, EventArgs e)
        {
            TrackListBox.Items.Clear();

            VehicleInitPositionCAN();
            ParkingInformationCAN();
            PlanningCommandCAN(0x50);
        }

        /// <summary>
        /// 泊车结束
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button24_Click(object sender, EventArgs e)
        {
            PlanningCommandCAN(0x80);
        }

        /// <summary>
        /// 泊车模式设置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button25_Click(object sender, EventArgs e)
        {
            if(0 == comboBox6.SelectedIndex)
            {
                /// 车辆泊车图形显示
                track_chart.ChartAreas[0].AxisX.Maximum = 12;
                track_chart.ChartAreas[0].AxisX.Minimum = -1;
                track_chart.ChartAreas[0].AxisY.Maximum = 5;
                track_chart.ChartAreas[0].AxisY.Minimum = -3;
                track_chart.ChartAreas[0].AxisX.Interval = 0.5;
                track_chart.ChartAreas[0].AxisY.Interval = 0.5;
                if(false == track_chart.Series.Contains(VehicleModuleShow))
                {
                    track_chart.Series.Add(VehicleModuleShow);
                }
                TerminalControlCommandCAN(0x10);
            }
            else if (1 == comboBox6.SelectedIndex)
            {
                /// 车辆泊车图形显示
                track_chart.ChartAreas[0].AxisX.Maximum = 18;
                track_chart.ChartAreas[0].AxisX.Minimum = -5;
                track_chart.ChartAreas[0].AxisY.Maximum = 9;
                track_chart.ChartAreas[0].AxisY.Minimum = -6;
                track_chart.ChartAreas[0].AxisX.Interval = 1;
                track_chart.ChartAreas[0].AxisY.Interval = 1;
                if (true == track_chart.Series.Contains(VehicleModuleShow))
                {
                    track_chart.Series.Remove(VehicleModuleShow);
                }
                TerminalControlCommandCAN(0x20);
            }
            else if (2 == comboBox6.SelectedIndex)
            {

            }
            else
            {

            }
        }
        #endregion

        #endregion
    }
}
