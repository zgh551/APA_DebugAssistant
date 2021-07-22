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


        #region BLDC相关变量
        private string[] BLDC_WorkState = new string[2] { "待机", "投放" };
        private string[] BLDC_MotorDirection = new string[4] { "不转", "正传", "反转", "异常"};

        private Single BLDC_TargetPosition, BLDC_ActualPosition, BLDC_TargetTurn, BLDC_ActualTurn;

        private Single BLDC_PhaseIA, BLDC_PhaseIB, BLDC_PhaseIC;
        private Single BLDC_VBUS, BLDC_VBUS_I;
        private Single BLDC_Position, BLDC_Velocity;
        private Single BLDC_Current_D, BLDC_Current_Q;
        #endregion

        #region CAN相关变量
        ZLGCAN m_ZLGCAN = new ZLGCAN();
        UInt32 VehicleSendCAN    = 0;
        UInt32 VehicleReceiveCAN = 1;
        UInt32 TerminalCAN       = 2;
        #endregion

        #region 车辆本身相关变量
        Vehicle m_Vehicle = new Vehicle();

        string[] GearStatus         = new string[8] { "No Request", "驻车", "倒车", "空挡", "前进", "无效", "保留", "保留" };
        string[] VehicleDirection   = new string[4] { "静止", "前进", "后退", "无效" };
        string[] EPB_SwitchPosition = new string[4] { "无请求", "锁住", "释放", "无效" };
        string[] APA_SystemState    = new string[2] { "NoReady", "Ready" };
        string[] DriverMode         = new string[2] { "手动模式", "自动模式" };

        //string[] ESC_ControlStatus = new string[4] { "unavailible", "standby", "active", "inactive" };
        //string[] EPS_ControlStatus = new string[4] { "unavailible", "availible", "active", "inactive" };
        //const double FRONT_EDGE_TO_CENTER = 3.54;
        //const double REAR_EDGE_TO_CENTER  = 0.905;
        //const double LEFT_EDGE_TO_CENTER  = 0.9275;
        //const double RIGHT_EDGE_TO_CENTER = 0.9275;

        const double FRONT_EDGE_TO_CENTER = 3.886;
        const double REAR_EDGE_TO_CENTER = 1.1;
        const double LEFT_EDGE_TO_CENTER = 0.9275;
        const double RIGHT_EDGE_TO_CENTER = 0.9275;

            private Polar FrontLeftDiagonal;
            private Polar FrontRightDiagonal;
            private Polar RearLeftDiagonal;
            private Polar RearRightDiagonal;
        #endregion

        #region 数据显示变量
        Color[] u_color = new Color[4];

        Waveform m_Waveform = new Waveform();
        MonitorForm m_MonitorForm = new MonitorForm();
        AccelarateForm m_AccelarateForm = new AccelarateForm();

        Series UltrasonicDataShow = new Series();//超声显示
        Series UltrasonicVehicle  = new Series(); //超声车身

        Series UltrasonicBodyDirectLocation     = new Series();//超声障碍物直接定位(车体坐标系)
        Series UltrasonicBodyTriangleLocation   = new Series();//超声障碍物三角定位(车体坐标系)
        Series UltrasonicGroundDirectLocation   = new Series();//超声障碍物直接定位(地面坐标系)
        Series UltrasonicGroundTriangleLocation = new Series();//超声障碍物三角定位(地面坐标系)

        Series TrackDataShow     = new Series();//跟踪数据显示
        Series TrackEdgeDataShow = new Series();//跟踪车辆轮廓数据显示

        Series ParkingDataShow   = new Series();//车位数据显示
        Series VehicleModuleShow = new Series();//车辆模型显示

        Series TurnningPointShow = new Series();//车辆转向角显示

        /// <summary>
        /// 关于超声波进行障碍物定位的数据显示
        /// 这些数据涉及前向库位的长距传感器的数据显示，短距车辆后侧的数据显示，和最终库位坐标点的显示
        /// </summary>
        Series UltrasonicObstacleLocation_LRU_DataShow = new Series();//超声障碍物定位长距传感器数据显示
        Series UltrasonicObstacleLocation_SRU_DataShow = new Series();//超声障碍物定位短距传感器数据显示
        Series UltrasonicObstacleLocationParkingPointDataShow = new Series();//超声障碍物定位检测库位点数据显示

        Series LeftFitLine_DataShow = new Series();//库位左边界拟合直线
        Series RightFitLine_DataShow = new Series();//库位右边界拟合直线
        Series CenterFitLine_DataShow = new Series();//库位左边界拟合直线

        Series ObstacleDistance_DataShow = new Series();//障碍物距离显示
        //Series[] test_show = new Series[2] { new Series(),new Series()};
        byte track_update_status;
        byte parking_update_status;
        byte vehicle_update_status;

        /// <summary>
        /// BLDC 3相电流显示
        /// </summary>
        Series BLDC_Phase_IA_Show = new Series();//A相电流
        Series BLDC_Phase_IB_Show = new Series();//B相电流
        Series BLDC_Phase_IC_Show = new Series();//C相电流

        Series BLDC_Current_D_Show = new Series();//D电流
        Series BLDC_Current_Q_Show = new Series();//Q电流

        Series BLDC_VBUS_Show = new Series();//电机总线电压

        Series BLDC_PositionShow = new Series();//电机位置
        Series BLDC_VelocityShow = new Series();//电机转速
        #endregion

        #region 超声参数
        public LIN_STP313_ReadData[] m_LIN_STP313_ReadData = new LIN_STP313_ReadData[4];
        public LIN_STP318_ReadData[] m_LIN_STP318_ReadData = new LIN_STP318_ReadData[8];
        public LIN_STP318_ReadData[] m_LIN_STP318_Location_ReadData = new LIN_STP318_ReadData[12];

        public Ultrasonic_Data_Packet[] m_Ultrasonic_Data_Packet = new Ultrasonic_Data_Packet[12];
        public Ultrasonic_Data_Packet[] m_Ultrasonic_Location_Data_Packet = new Ultrasonic_Data_Packet[12];//超声定位的原始数据

        public UltrasonicAxis[] m_BodyDirectLocation = new UltrasonicAxis[12];
        public UltrasonicAxis[] m_BodyTriangleLocation = new UltrasonicAxis[12];

        public UltrasonicAxis[] m_GroundDirectLocation   = new UltrasonicAxis[12];
        public UltrasonicAxis[] m_GroundTriangleLocation = new UltrasonicAxis[12];
        
        //LRU STP313 传感器 控件显示
        Label[][] SensingControl_LRU = new Label[4][];
        //SRU STP318 传感器 控件显示
        Label[][] SensingControl_SRU = new Label[8][];

        Label[][] SensingLocation_SRU = new Label[4][];

        Ultrasonic m_Ultrasonic = new Ultrasonic();

        const byte UltrasonicDataType = 2;
        #endregion

        #region PID参数
        float Kp, Ki, Kd, Threshold;
        byte ControlStateFlag;
        #endregion

        #region 目标轨迹参数
        Vector2d TargetTrack;
        Single Sliding_x1, Sliding_x2, SlidingVariable;
        #endregion

        #region System Working State
        string[] WorkingModule = new string[5] { "调试", "标定", "测试", "正常", "超声配置模式"};
        string[][] FunctionStatus = new string[5][]
        {
            new string [4] { "直接控制", "速度控制", "长安车控制", "超声波收发"},
            new string [3] { "速度标定", "脉冲标定", "超声波标定"},
            new string [4] { "车位检测", "侧方停车", "垂直停车", "斜向停车" },
            new string [5] { "APA_1.0", "APA_2.0", "APA_3.0", "APA_4.0", "APA_5.0"},
            new string [2] { "检车位模式", "正常模式"}
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
        public struct FitLine
        {
            public double Angle;
            public double Offset;
        }

        StreamWriter ULtrasonicDataSave;
        StreamWriter BodyTriangleDataSave;
        StreamWriter ParkingDataSave;
        string u_FilePath, u_localFilePath, u_newFileName, u_fileNameExt;
        bool u_DataSaveStatus = false;
        #endregion

        #region 轨迹规划数据保存相关变量
        StreamWriter PlanningDataSave;
        bool PlanningDataSaveStatus = false;
        #endregion

        #region Ultrasonic File Loading
        StreamReader UltrasonicDataLoad;
        string LoadFilePath;/* localFilePath, newFileName, fileNameExt;*/
        Int32 FileLineCount;
        //bool DataLoadStatus = false;
        #endregion

        #region UltrasonicObstacleLocation
        StreamReader UltrasonicLocationData;//注入数据的读取
        StreamWriter LocationMapDataSave;//原始数据的采集存储
        StreamWriter LocationResultDataSave;//定位结果保存
        string UltrasonicLocationLoadFilePath;
        Int32 UltrasonicLocationFileLineCount;

        byte LocationOpenFileFlag = 0;
        byte InjectionAckFlag = 0;

        bool LocationDataSaveStatus = false;

        LocationPoint FrontParkingEdge;
        LocationPoint RearParkingEdge;
        LocationPoint ParkingCenter;//库位中心的新坐标 
        FitLine left_fit_line;
        FitLine right_fit_line;
        FitLine center_fit_line;
        bool FitLineShowFlag = false;

        byte UltrasonicLocationStatus;
        string[] u_location_status = new string[4] { "Ready", "Push", "Calculate", "Finish" };
        Byte FrontParkingPositionCnt;
        Byte RearParkingPositionCnt;
        Byte LeftParkingPositionCnt;
        Byte RightParkingPositionCnt;

        // 避障信息
        public struct ObstacleDistancePacket
        {
            public Single Distance;
            public byte region;
            public byte status;
        }

        ObstacleDistancePacket FrontObstacle,RearObstacle;
        string[] region_status = new string[5] { "左侧", "左中","中间","右中", "右侧"};
        string[] valid_status = new string[5] { "正常", "盲区", "超探", "噪声", "无效" };
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
        double ActualTurnRadius;
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
        uint TestLastTime, TestErrTime;
        uint ParkingLastTime, ParkingErrTime;
        uint UltrasonicLastTime, UltrasonicErrTime;
        uint PlanLastTime, PlanErrTime;
        uint LocationMapLastTime, LocationMapErrTime;
        #endregion

        //System.Drawing.Imaging.ColorPalette GreyColorPalette = null;
        //System.Drawing.Bitmap newBitmap = null;
        #endregion
        #region 函数
        #region PID设置(CAN)
        private void PID_ParameterConfigureCAN()
        {
            uint id = 0x550;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox6.Text));//P
            dat[0] = Data_Temp[3];
            dat[1] = Data_Temp[2];
            dat[2] = Data_Temp[1];
            dat[3] = Data_Temp[0];
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox7.Text));//I
            dat[4] = Data_Temp[3];
            dat[5] = Data_Temp[2];
            dat[6] = Data_Temp[1];
            dat[7] = Data_Temp[0];
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);

            id = 0x551;
            len = 8;
            dat = new byte[8];
            Data_Temp = new byte[8];//动态分配内存
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox8.Text));//D
            dat[0] = Data_Temp[3];
            dat[1] = Data_Temp[2];
            dat[2] = Data_Temp[1];
            dat[3] = Data_Temp[0];
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox35.Text));//OUTPUT LIM
            dat[4] = Data_Temp[3];
            dat[5] = Data_Temp[2];
            dat[6] = Data_Temp[1];
            dat[7] = Data_Temp[0];
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);

            id = 0x552;
            len = 8;
            dat = new byte[8];
            Data_Temp = new byte[8];//动态分配内存
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox11.Text));//Threshold
            dat[0] = Data_Temp[3];
            dat[1] = Data_Temp[2];
            dat[2] = Data_Temp[1];
            dat[3] = Data_Temp[0];
            Data_Temp = BitConverter.GetBytes(Convert.ToSingle(textBox34.Text));//I Lim
            dat[4] = Data_Temp[3];
            dat[5] = Data_Temp[2];
            dat[6] = Data_Temp[1];
            dat[7] = Data_Temp[0];
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
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
        /// 超声波定位信息数据发送
        /// </summary>
        /// <param name="id_num"></param>
        /// <param name="u"></param>
        private void UltrasonicLocationCAN(uint id_num, UltrasonicAxis u)
        {
            uint id = 0x500 | id_num;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存
            UInt16 u16_data_temp;

            u16_data_temp = (UInt16)(u.x * 100);
            Data_Temp = BitConverter.GetBytes(u16_data_temp);
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];

            u16_data_temp = (UInt16)(u.y * 100);
            Data_Temp = BitConverter.GetBytes(u16_data_temp);
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];

            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = (byte)u.state;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        private void UltrasonicPacketCAN(uint id_num, Ultrasonic_Data_Packet u)
        {
            uint id = 0x500 | id_num;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存
            UInt16 u16_data_temp;

            u16_data_temp = (UInt16)(u.Distance1 * 100);
            Data_Temp = BitConverter.GetBytes(u16_data_temp);
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];

            u16_data_temp = (UInt16)(u.Distance2 * 100);
            Data_Temp = BitConverter.GetBytes(u16_data_temp);
            dat[2] = Data_Temp[0];
            dat[3] = Data_Temp[1];

            dat[4] = (byte)(u.Level * 10);
            dat[5] = (byte)u.Width;
            dat[6] = (byte)u.status;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        /// <summary>
        /// 注入文件解析，适用于检车位
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

        private void FileDataParse(string[] s, ref Ultrasonic_Data_Packet[] u, ref UltrasonicAxis[] dir_lc, ref UltrasonicAxis[] triangle_lc)
        {
            for (int i = 0; i < 4; i++)
            {
                u[8 + i].Distance1 = Convert.ToSingle(s[40 + 5 * i]);
                u[8 + i].Distance2 = Convert.ToSingle(s[41 + 5 * i]);
                u[8 + i].Level     = Convert.ToSingle(s[42 + 5 * i]);
                u[8 + i].Width     = Convert.ToSingle(s[43 + 5 * i]);
                u[8 + i].status    = Convert.ToByte  (s[44 + 5 * i]);
            }
            for (int i = 0; i < 4; i++)
            {
                dir_lc[8 + i].x = Convert.ToSingle(s[25 + 3 * i]);
                dir_lc[8 + i].y = Convert.ToSingle(s[26 + 3 * i]);
                dir_lc[8 + i].state = (UltrasonicStatus)Convert.ToByte(s[27 + 3 * i]);
            }
            for (int i = 0; i < 8; i++)
            {
                triangle_lc[i].x     = Convert.ToSingle(s[1 + 3 * i]);
                triangle_lc[i].y     = Convert.ToSingle(s[2 + 3 * i]);
                triangle_lc[i].state = (UltrasonicStatus)Convert.ToByte(s[3 + 3 * i]);
            }
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

        private void TerminalWorkModeCommandCAN(byte wk,byte fc)
        {
            uint id = 0x531;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = wk;
            dat[1] = fc;
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

        #region 超声定位控制命令
        private void UltrasonicLocationCommandCAN(byte cmd)
        {
            uint id = 0x533;
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
            dat[7] = Convert.ToByte(textBox30.Text) ;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 长安对接接口
        /// <summary>
        /// 用于长安车调试的接口控制函数
        /// </summary>
        //private void ChangAnInterfaceCAN()
        //{
        //    uint id = 0x516;
        //    byte len = 8;
        //    byte CheckSum;
        //    byte[] dat = new byte[8];
        //    byte[] Data_Temp = new byte[8];//动态分配内存

        //    dat[0] = (byte)(
        //                      (Convert.ToByte(checkBox6.Checked) << 3)// Velocity 
        //                    | (Convert.ToByte(checkBox3.Checked) << 5)// Torque 
        //                    | (Convert.ToByte(checkBox2.Checked) << 4)// AEB 
        //                    | (Convert.ToByte(checkBox1.Checked) << 2)// ACC
        //                    | (Convert.ToByte(checkBox4.Checked) << 1)// Steering Angle
        //                    | Convert.ToByte(checkBox5.Checked)      // Gear enable
        //                    );
        //    dat[1] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位

        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
        //    dat[2] = Data_Temp[0];
        //    dat[3] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox5.Text) * 100));//转向角速度
        //    dat[4] = Data_Temp[0];
        //    dat[5] = Data_Temp[1];

        //    dat[6] = 0;
        //    CheckSum = 0;
        //    for (int i = 0; i < 7; i++)
        //    {
        //        CheckSum += dat[i];
        //    }
        //    CheckSum ^= 0xFF;
        //    dat[7] = CheckSum;
        //    m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);

        //    id = 0x517;

        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox1.Text) * 1000));//ACC加速度
        //    dat[0] = Data_Temp[0];
        //    dat[1] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox2.Text) * 1000));//AEB减速度
        //    dat[2] = Data_Temp[0];
        //    dat[3] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox12.Text) * 1000));//车辆速度
        //    dat[4] = Data_Temp[0];
        //    dat[5] = Data_Temp[1];

        //    dat[6] = (byte)(Convert.ToSingle(textBox3.Text) * 0.5);// 扭矩
        //    CheckSum = 0;
        //    for (int i = 0; i < 7; i++)
        //    {
        //        CheckSum += dat[i];
        //    }
        //    CheckSum ^= 0xFF;
        //    dat[7] = CheckSum;
        //    m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        //}
        #endregion

        #region 通用对接接口
        private void VehicleControlCAN_BR1()
        {
            uint id = 0x411;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
            dat[0] = Data_Temp[1];
            dat[1] = Data_Temp[0];
            Data_Temp = BitConverter.GetBytes((UInt16)Convert.ToSingle(textBox31.Text));//刹停距离
            dat[2] = Data_Temp[1];
            dat[3] = Data_Temp[0];
  
            dat[4] = (Byte)(Convert.ToSingle(textBox32.Text) * 10);//刹停速度
            dat[5] = Convert.ToByte(comboBox3.SelectedIndex);      // 挡位

            dat[6] = (Byte)(Convert.ToByte(checkBox4.Checked) << 6);
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        private void VehicleControlCAN_BR2()
        {
            uint id = 0x412;
            byte len = 8;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox3.Text) * 100));//扭矩值
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];

            dat[2] = (Byte)(Convert.ToByte(checkBox3.Checked) << 7);// torque enable
            dat[3] = (Byte)(Convert.ToSingle(textBox1.Text) * 20);//acc value

            dat[4] = (Byte)(Convert.ToByte(checkBox1.Checked) << 4);//ACC enable
            dat[5] = 0;

            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        private void ControlInterfaceCAN()
        {
            uint id = 0x518;
            byte len = 8;
            byte CheckSum;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存

            Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
            dat[0] = Data_Temp[0];
            dat[1] = Data_Temp[1];
            dat[2] = (byte)(Convert.ToSingle(textBox5.Text) * 0.25);
            dat[3] = (byte)(Convert.ToSingle(textBox32.Text) * 100);//刹停速度
            Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox31.Text)*1000));//刹停距离
            dat[4] = Data_Temp[0];
            dat[5] = Data_Temp[1];

            dat[6] = (byte)((Convert.ToByte(checkBox11.Checked) << 4) | Convert.ToByte(comboBox3.SelectedIndex));// 挡位

            CheckSum = 0;
            for (int i = 0; i < 7; i++)
            {
                CheckSum += dat[i];
            }
            CheckSum ^= 0xFF;
            dat[7] = CheckSum;

            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }

        private void EmergencyInterfaceCAN()
        {
            uint id = 0x51B;
            byte len = 8;
            byte CheckSum;
            byte[] dat = new byte[8];
            byte[] Data_Temp = new byte[8];//动态分配内存
            float f_temp_data;
            f_temp_data = Convert.ToSingle(textBox9.Text);//制动行程
            f_temp_data = f_temp_data > 100.0f ? 100.0f : f_temp_data < 0.0f ? 0.0f : f_temp_data;
            dat[0] = (byte)(f_temp_data * 2.5f);
            dat[1] = (byte)Convert.ToByte(checkBox6.Checked);
            dat[2] = (byte)Convert.ToByte(checkBox14.Checked);
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
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
        #endregion

        #region 东风接口
        /// <summary>
        /// 用于东风车调试的接口控制函数
        /// </summary>
        //private void DongFengInterfaceCAN()
        //{
        //    uint id = 0x519;
        //    byte len = 8;
        //    byte CheckSum;
        //    byte[] dat = new byte[8];
        //    byte[] Data_Temp = new byte[8];//动态分配内存

        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox4.Text) * 10));//转向角
        //    dat[0] = Data_Temp[0];
        //    dat[1] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox5.Text) * 100));//转向角速度
        //    dat[2] = Data_Temp[0];
        //    dat[3] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox3.Text) * 1));//扭矩
        //    dat[4] = Data_Temp[0];
        //    dat[5] = Data_Temp[1];

        //    dat[6] = (byte)(
        //                      (Convert.ToByte(checkBox6.Checked) << 3)// Velocity 
        //                    | (Convert.ToByte(checkBox3.Checked) << 5)// Torque 
        //                    | (Convert.ToByte(checkBox2.Checked) << 4)// AEB 
        //                    | (Convert.ToByte(checkBox1.Checked) << 2)// ACC
        //                    | (Convert.ToByte(checkBox4.Checked) << 1)// Steering Angle
        //                    |  Convert.ToByte(checkBox5.Checked)      // Gear enable
        //                    );
        //    CheckSum = 0;
        //    for (int i = 0; i < 7; i++)
        //    {
        //        CheckSum += dat[i];
        //    }
        //    CheckSum ^= 0xFF;
        //    dat[7] = CheckSum;
        //    m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);

        //    id = 0x51A;
        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox1.Text) * 1000));//ACC加速度
        //    dat[0] = Data_Temp[0];
        //    dat[1] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((Int16)(Convert.ToSingle(textBox2.Text) * 1000));//AEB减速度
        //    dat[2] = Data_Temp[0];
        //    dat[3] = Data_Temp[1];

        //    Data_Temp = BitConverter.GetBytes((UInt16)(Convert.ToSingle(textBox12.Text) * 1000));//车辆速度
        //    dat[4] = Data_Temp[0];
        //    dat[5] = Data_Temp[1];

        //    dat[6] = Convert.ToByte(comboBox3.SelectedIndex);// 挡位
        //    CheckSum = 0;
        //    for (int i = 0; i < 7; i++)
        //    {
        //        CheckSum += dat[i];
        //    }
        //    CheckSum ^= 0xFF;
        //    dat[7] = CheckSum;
        //    m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        //}
        #endregion

        #region 应答接口
        private void AckCommandCAN()
        {
            uint id = 0x512;
            byte len = 8;
            byte[] dat = new byte[8];
            dat[0] = 0x5A;
            dat[1] = 0xA5;
            dat[2] = 0;
            dat[3] = 0;
            dat[4] = 0;
            dat[5] = 0;
            dat[6] = 0;
            dat[7] = 0;
            m_ZLGCAN.CAN_Send(TerminalCAN, id, len, dat);
        }
        #endregion

        #region 终端解码函数
        private delegate void TerminalParse(ZLGCAN.VCI_CAN_OBJ m_packet);
        private void UltrasonicParse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            TerminalParse m_sampling = new TerminalParse(MCU_Parse);
            this.Invoke(m_sampling, new object[] { m_packet });
        }
        /// <summary>
        /// Chang An vehicle imformation receive
        /// </summary>
        /// <param name="m_packet"></param>
        /// <param name="m_vehicle"></param>
        unsafe void MCU_Parse(ZLGCAN.VCI_CAN_OBJ m_packet)
        {
            byte[] tmp_dat = new byte[4] { 0, 0, 0, 0 };
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
                    if(1 == UltrasonicDataType)
                    {
                        m_LIN_STP318_ReadData[m_packet.ID & 0x007].TOF = BitConverter.ToUInt16(tmp_dat, 0);
                        m_LIN_STP318_ReadData[m_packet.ID & 0x007].status = m_packet.Data[6];
                    }
                    else if (2 == UltrasonicDataType)
                    {
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance1 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].status = m_packet.Data[6];
                    }
                    break;

                case 0x408://传感器9
                case 0x409://传感器10
                case 0x40A://传感器11
                case 0x40B://传感器12
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    if (1 == UltrasonicDataType)
                    {
                        m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF1 = BitConverter.ToUInt16(tmp_dat, 0);
                    }
                    else if (2 == UltrasonicDataType)
                    {
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance1 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                    }                    
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    if (1 == UltrasonicDataType)
                    {
                        m_LIN_STP313_ReadData[m_packet.ID & 0x003].TOF2 = BitConverter.ToUInt16(tmp_dat, 0);
                        m_LIN_STP313_ReadData[m_packet.ID & 0x003].Level = m_packet.Data[4];
                        m_LIN_STP313_ReadData[m_packet.ID & 0x003].Width = m_packet.Data[5];
                        m_LIN_STP313_ReadData[m_packet.ID & 0x003].status = m_packet.Data[6];
                    }
                    else if(2 == UltrasonicDataType)
                    {
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Distance2 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Level = m_packet.Data[4] * 0.1f;
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].Width = m_packet.Data[5];
                        m_Ultrasonic_Data_Packet[m_packet.ID & 0x00f].status = m_packet.Data[6];
                    }
                    break;

                case 0x470:
                case 0x471:
                case 0x472:
                case 0x473:
                case 0x474:
                case 0x475:
                case 0x476:
                case 0x477:
                case 0x478:
                case 0x479:
                case 0x47A:
                case 0x47B:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    if (1 == UltrasonicDataType)
                    {
                        m_LIN_STP318_Location_ReadData[m_packet.ID & 0x00F].TOF = BitConverter.ToUInt16(tmp_dat, 0);
                        m_LIN_STP318_Location_ReadData[m_packet.ID & 0x00F].status = m_packet.Data[6];
                    }
                    else if (2 == UltrasonicDataType)
                    {
                        m_Ultrasonic_Location_Data_Packet[m_packet.ID & 0x00f].Distance1 = BitConverter.ToUInt16(tmp_dat, 0) * 0.01f;
                        m_Ultrasonic_Location_Data_Packet[m_packet.ID & 0x00f].status    = m_packet.Data[6];
                    }
                    break;

                case 0x480:
                case 0x481:
                case 0x482:
                case 0x483:
                case 0x484:
                case 0x485:
                case 0x486:
                case 0x487:

                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_BodyTriangleLocation[m_packet.ID & 0x00f].x = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_BodyTriangleLocation[m_packet.ID & 0x00f].y = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    m_BodyTriangleLocation[m_packet.ID & 0x00f].state = (UltrasonicStatus)m_packet.Data[7];
                    break;

                case 0x488:
                case 0x489:
                case 0x48A:
                case 0x48B:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_BodyDirectLocation[m_packet.ID & 0x00f].x = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_BodyDirectLocation[m_packet.ID & 0x00f].y = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    m_BodyDirectLocation[m_packet.ID & 0x00f].state = (UltrasonicStatus)m_packet.Data[7];
                    break;
                    
                // 地面坐标系
                case 0x490:
                case 0x491:
                case 0x492:
                case 0x493:
                case 0x494:
                case 0x495:
                case 0x496:
                case 0x497:

                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_GroundTriangleLocation[m_packet.ID & 0x00f].x = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_GroundTriangleLocation[m_packet.ID & 0x00f].y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    m_GroundTriangleLocation[m_packet.ID & 0x00f].state = (UltrasonicStatus)m_packet.Data[7];
                    break;

                case 0x498:
                case 0x499:
                case 0x49A:
                case 0x49B:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_GroundDirectLocation[m_packet.ID & 0x00f].x = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_GroundDirectLocation[m_packet.ID & 0x00f].y = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    m_GroundDirectLocation[m_packet.ID & 0x00f].state = (UltrasonicStatus)m_packet.Data[7];
                    break;
                    
                case 0x416:
                    if (m_packet.Data[1] == 0xA5)
                    {
                        AckId = m_packet.Data[0];
                        AckCnt = 0;
                        AckCommandCAN();
                        InjectionAckFlag = 0xAA;
                        //InjectionUltrasonicTimer.Enabled = true;
                        timer_ack.Enabled = true;
                    }
                    break;

                case 0x410:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.VehicleSpeed = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.SteeringAngleActual = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.SteeringAngleSpeed = (UInt16)(BitConverter.ToUInt16(tmp_dat, 0) * 0.01);
                    m_Vehicle.ActualGearShift = Convert.ToByte(m_packet.Data[6] & 0x0f);
                    m_Vehicle.WheelSpeedDirection   = Convert.ToByte( m_packet.Data[7]       & 0x03);
                    m_Vehicle.SystemReadySts        = Convert.ToByte((m_packet.Data[7] >> 2) & 0x01);
                    m_Vehicle.AutoDriverModeSts     = Convert.ToByte((m_packet.Data[7] >> 2) & 0x01);
                    break;

                case 0x411:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.WheelSpeedFrontLeftData = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.WheelSpeedFrontRightData = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.WheelSpeedRearLeftData = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    m_Vehicle.WheelSpeedRearRightData = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    break;

                case 0x412:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.WheelSpeedFrontLeftPulse = BitConverter.ToUInt16(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.WheelSpeedFrontRightPulse = BitConverter.ToUInt16(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.WheelSpeedRearLeftPulse = BitConverter.ToUInt16(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    m_Vehicle.WheelSpeedRearRightPulse = BitConverter.ToUInt16(tmp_dat, 0);
                    break;

                case 0x413:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    tmp_dat[2] = m_packet.Data[2];
                    tmp_dat[3] = m_packet.Data[3];
                    m_Vehicle.WheelSpeedRearLeftPulseSum = BitConverter.ToInt32(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    tmp_dat[2] = m_packet.Data[6];
                    tmp_dat[3] = m_packet.Data[7];
                    m_Vehicle.WheelSpeedRearRightPulseSum = BitConverter.ToInt32(tmp_dat, 0);
                    break;

                case 0x414:
                    m_Vehicle.TargetAccelerationEnable = Convert.ToBoolean( m_packet.Data[0]       & 0x01);
                    m_Vehicle.TargetDecelerationEnable = Convert.ToBoolean((m_packet.Data[0] >> 1) & 0x01);
                    m_Vehicle.TorqueEnable             = Convert.ToBoolean((m_packet.Data[0] >> 2) & 0x01);
                    m_Vehicle.VelocityEnable           = Convert.ToBoolean((m_packet.Data[0] >> 3) & 0x01);
                    m_Vehicle.SteeringAngleActive      = Convert.ToByte   ((m_packet.Data[0] >> 4) & 0x03);
                    m_Vehicle.GearShiftEnable          = Convert.ToBoolean((m_packet.Data[0] >> 6) & 0x01);

                    m_Vehicle.TargetGearShift = m_packet.Data[1];
                    if (checkBox7.Checked)
                    {
                        if (4 == m_Vehicle.TargetGearShift)
                        {
                            m_Vehicle.WheelSpeedRearRightDirection = 0;
                        }
                        else if (2 == m_Vehicle.TargetGearShift)
                        {
                            m_Vehicle.WheelSpeedRearRightDirection = 1;
                        }
                        else
                        {
                            m_Vehicle.WheelSpeedRearRightDirection = 2;
                        }
                    }
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.ActualAccelerationACC = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.Torque = BitConverter.ToUInt16(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    m_Vehicle.TargetVehicleSpeed = BitConverter.ToUInt16(tmp_dat, 0) * 0.01;
                    if (checkBox7.Checked)
                    {
                        if (0 == m_Vehicle.TargetVehicleSpeed)
                        {
                            m_Vehicle.WheelSpeedRearRightDirection = 2;
                        }
                        if (m_Vehicle.VelocityEnable)
                        {
                            m_Vehicle.WheelSpeedRearLeftData  = m_Vehicle.TargetVehicleSpeed;
                            m_Vehicle.WheelSpeedRearRightData = m_Vehicle.TargetVehicleSpeed;
                        }
                        else
                        {
                            m_Vehicle.WheelSpeedRearLeftData  = 0;
                            m_Vehicle.WheelSpeedRearRightData = 0;
                        }
                    }
                    break;
                case 0x415:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.TargetDistance = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.TargetAccelerationACC = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.SteeringAngleTarget = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    break;

                case 0x417:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.LonAcc = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.LatAcc = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    m_Vehicle.YawRate = BitConverter.ToInt16(tmp_dat, 0) * 0.01;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    m_Vehicle.Temperature = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    break;

                case 0x418:
                    m_Vehicle.EPS_Status = Convert.ToByte( m_packet.Data[0]       & 0x01);
                    m_Vehicle.ESC_Status = Convert.ToByte((m_packet.Data[0] >> 1) & 0x01);
                    m_Vehicle.EPB_Status = Convert.ToByte((m_packet.Data[0] >> 2) & 0x01);
                    m_Vehicle.VCU_Status = Convert.ToByte((m_packet.Data[0] >> 3) & 0x01);
                    m_Vehicle.SAS_Status = Convert.ToByte((m_packet.Data[0] >> 4) & 0x01);
                    m_Vehicle.TCU_Status = Convert.ToByte((m_packet.Data[0] >> 5) & 0x01);
                    m_Vehicle.EMS_Status = Convert.ToByte((m_packet.Data[0] >> 6) & 0x01);

                    m_Vehicle.DriverDoorSts     = Convert.ToByte( m_packet.Data[1]       & 0x01);
                    m_Vehicle.PassangerDoorSts  = Convert.ToByte((m_packet.Data[1] >> 1) & 0x01);
                    m_Vehicle.TrunkSts          = Convert.ToByte((m_packet.Data[1] >> 2) & 0x01);

                    m_Vehicle.TurnLightLeftSts  = Convert.ToByte( m_packet.Data[2]       & 0x01);
                    m_Vehicle.TurnLightRightSts = Convert.ToByte((m_packet.Data[2] >> 1) & 0x01);

                    m_Vehicle.DriverSeatBeltSwitchSts = Convert.ToByte(m_packet.Data[3] & 0x01);

                    m_Vehicle.EPB_SwitchPosition = Convert.ToByte(m_packet.Data[4] & 0x03);
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
                    //tmp_dat[0] = m_packet.Data[6];
                    //tmp_dat[1] = m_packet.Data[7];
                    //m_Vehicle.VehicleSpeed = BitConverter.ToUInt16(tmp_dat, 0) * 0.001;
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

                case 0x449:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];        
                    FrontParkingEdge.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    FrontParkingEdge.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    RearParkingEdge.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    RearParkingEdge.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    LocationlistBox.Items.Add("库位定位完成");
                    LocationlistBox.Items.Add("前端:");
                    LocationlistBox.Items.Add("x=" + FrontParkingEdge.Position.X.ToString());
                    LocationlistBox.Items.Add("y=" + FrontParkingEdge.Position.Y.ToString());
                    LocationlistBox.Items.Add("后端:");
                    LocationlistBox.Items.Add("x=" + RearParkingEdge.Position.X.ToString());
                    LocationlistBox.Items.Add("y=" + RearParkingEdge.Position.Y.ToString());
                    break;

                case 0x44A:
                    UltrasonicLocationStatus = m_packet.Data[0];
                    FrontParkingPositionCnt = m_packet.Data[4];
                    RearParkingPositionCnt  = m_packet.Data[5];           
                    LeftParkingPositionCnt = m_packet.Data[6];
                    RightParkingPositionCnt = m_packet.Data[7];
                    break;

                case 0x44B:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    ParkingCenter.Position.X = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    ParkingCenter.Position.Y = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    ParkingCenter.Yaw = BitConverter.ToInt16(tmp_dat, 0) * 0.0001;
                    center_fit_line.Angle = ParkingCenter.Yaw;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    center_fit_line.Offset = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    LocationlistBox.Items.Add("库位中心信息:");
                    LocationlistBox.Items.Add("x:"   + ParkingCenter.Position.X.ToString());
                    LocationlistBox.Items.Add("y:"   + ParkingCenter.Position.Y.ToString());
                    LocationlistBox.Items.Add("yaw:" + ParkingCenter.Yaw.ToString());
                    LocationlistBox.Items.Add("库位中心拟合直线:");
                    LocationlistBox.Items.Add("A:" + Math.Tan(center_fit_line.Angle).ToString());
                    LocationlistBox.Items.Add("B:" + center_fit_line.Offset.ToString());
                    LocationResultInfDatalog();
                    break;

                case 0x44C:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    left_fit_line.Angle = BitConverter.ToInt16(tmp_dat, 0) * 0.0001;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    left_fit_line.Offset = BitConverter.ToInt16(tmp_dat, 0) * 0.001;
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    right_fit_line.Angle = BitConverter.ToInt16(tmp_dat, 0) * 0.0001;
                    tmp_dat[0] = m_packet.Data[6];
                    tmp_dat[1] = m_packet.Data[7];
                    right_fit_line.Offset = BitConverter.ToInt16(tmp_dat, 0) * 0.001;

                    LocationlistBox.Items.Add("左库位拟合直线:");
                    LocationlistBox.Items.Add("A:" + Math.Tan(left_fit_line.Angle).ToString());
                    LocationlistBox.Items.Add("B:" + left_fit_line.Offset.ToString());

                    LocationlistBox.Items.Add("右库位拟合直线:");
                    LocationlistBox.Items.Add("A:" + Math.Tan(right_fit_line.Angle).ToString());
                    LocationlistBox.Items.Add("B:" + right_fit_line.Offset.ToString());
                    FitLineShowFlag = true;
                    LeftFitLine_DataShow.Points.Clear();
                    RightFitLine_DataShow.Points.Clear();
                    CenterFitLine_DataShow.Points.Clear();
                    break;

                case 0x44D:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    FrontObstacle.Distance = BitConverter.ToUInt16(tmp_dat, 0) * 0.0001f;
                    FrontObstacle.region   = m_packet.Data[2];
                    FrontObstacle.status   = m_packet.Data[3];

                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    RearObstacle.Distance = BitConverter.ToUInt16(tmp_dat, 0) * 0.0001f;
                    RearObstacle.region = m_packet.Data[6];
                    RearObstacle.status = m_packet.Data[7];

                    break;

                case 0x44E:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    m_Vehicle.PulseUpdateVelocity = BitConverter.ToUInt16(tmp_dat, 0) * 0.0001f;
                    tmp_dat[0] = m_packet.Data[2];
                    tmp_dat[1] = m_packet.Data[3];
                    m_Vehicle.AccUpdateVelocity = BitConverter.ToUInt16(tmp_dat, 0) * 0.0001f;
                    break;

                case 0x4A0:
                    ControlStateFlag = m_packet.Data[0];
                    break;

                case 0x4B0:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    tmp_dat[2] = m_packet.Data[2];
                    tmp_dat[3] = m_packet.Data[3];
                    TargetTrack.X = BitConverter.ToSingle(tmp_dat,0);
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    tmp_dat[2] = m_packet.Data[6];
                    tmp_dat[3] = m_packet.Data[7];
                    TargetTrack.Y = BitConverter.ToSingle(tmp_dat, 0);
                    break;

                case 0x4B1:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    tmp_dat[2] = m_packet.Data[2];
                    tmp_dat[3] = m_packet.Data[3];
                    Sliding_x1 = BitConverter.ToSingle(tmp_dat, 0);
                    tmp_dat[0] = m_packet.Data[4];
                    tmp_dat[1] = m_packet.Data[5];
                    tmp_dat[2] = m_packet.Data[6];
                    tmp_dat[3] = m_packet.Data[7];
                    Sliding_x2 = BitConverter.ToSingle(tmp_dat, 0);
                    break;

                case 0x4B2:
                    tmp_dat[0] = m_packet.Data[0];
                    tmp_dat[1] = m_packet.Data[1];
                    tmp_dat[2] = m_packet.Data[2];
                    tmp_dat[3] = m_packet.Data[3];
                    SlidingVariable = BitConverter.ToSingle(tmp_dat, 0);
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
                    m_Vehicle.SteeringAngleTarget = BitConverter.ToInt16(tmp_dat, 0) * 0.1;
                    //m_Vehicle.SteeringAngleActual = m_Vehicle.SteeringAngleTarget;
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

                    //m_Vehicle.VehicleSpeed = (m_Vehicle.WheelSpeedRearRightData + m_Vehicle.WheelSpeedRearLeftData) * 0.5;

                    //m_Vehicle.VehicleSpeed = m_Vehicle.WheelSpeedDirection == 0 ?  m_Vehicle.VehicleSpeed :
                    //                         m_Vehicle.WheelSpeedDirection == 1 ? -m_Vehicle.VehicleSpeed : 0;
                    break;

                case 0x278:
                    m_Vehicle.LatAcc = m_packet.Data[2] * 0.1 - 12.7;
                    m_Vehicle.LonAcc = (m_packet.Data[3] << 2 | m_packet.Data[4] >> 6) * 0.03125 - 16;
                    m_Vehicle.YawRate = ((m_packet.Data[4] & 0x3f) << 8 | m_packet.Data[5]) * 0.01 - 81.91;
                    break;

                case 0x180://SAS
                    //m_Vehicle.SteeringAngleActual = (double)(((Int16)((dat[0] << 8) | dat[1])) * 0.1);
                    m_Vehicle.SteeringAngleSpeed = (UInt16)(dat[2] * 4);
                    break;

                // 东风汽车的数据解码
                case 0xA3://speed
                    m_Vehicle.WheelSpeedRearLeftData   = (UInt16)(((m_packet.Data[4] & 0x7f) << 8) | m_packet.Data[5]) * 0.002778;
                    m_Vehicle.WheelSpeedRearRightData  = (UInt16)(((m_packet.Data[6] & 0x7f) << 8) | m_packet.Data[7]) * 0.002778;
                    break;

                case 0xA6:
                    m_Vehicle.WheelSpeedRearLeftPulse  = (UInt16)(((m_packet.Data[2] & 0x0f) << 6) | ((m_packet.Data[3] >> 2) & 0x3f));
                    m_Vehicle.WheelSpeedRearRightPulse = (UInt16)(((m_packet.Data[3] & 0x03) << 8) |   m_packet.Data[4]              );
                    m_Vehicle.LonAcc = (UInt16)(((m_packet.Data[5] & 0x07) << 8) | m_packet.Data[6]) * 0.03 - 15.36;
                    break;

                case 0x122:
                    m_Vehicle.LatAcc  = (UInt16)(((m_packet.Data[0] & 0x0f) << 8) | m_packet.Data[1]) * 0.1 - 204.8;
                    m_Vehicle.YawRate = (((m_packet.Data[2] & 0x07) << 8) | m_packet.Data[3]) * 0.03 - 15.36;
                    break;

                // 长安车标定数据采集
                case 0x27B:
                    m_Vehicle.LonAcc = ((UInt16)((m_packet.Data[3] << 2) | (m_packet.Data[4] >> 6))) * 0.03125 - 16;
                    m_Vehicle.LatAcc = m_packet.Data[2] * 0.1f - 12.7f;
                    m_Vehicle.YawRate = (((m_packet.Data[4] & 0x3f) << 8) | m_packet.Data[5]) * 0.01 - 81.91;
                    break;
                case 0x20B:// wheel speed
                    m_Vehicle.WheelSpeedRearRightData = ((UInt16)(((m_packet.Data[0] & 0x1F) << 8) | m_packet.Data[1])) * 0.015625;
                    m_Vehicle.WheelSpeedRearLeftData = ((UInt16)(((m_packet.Data[2] & 0x1F) << 8) | m_packet.Data[3])) * 0.015625;
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
            temp_steering = (Int16)(m_Vehicle.SteeringAngleTarget * 10);

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
            dat[1] = (m_Vehicle.TargetGearShift == 1) ? (byte)0x0A :
                     (m_Vehicle.TargetGearShift == 2) ? (byte)0x09 :
                     (m_Vehicle.TargetGearShift == 3) ? (byte)0x00 : (byte)0x01;
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
                this.Invoke((EventHandler)(delegate
                {
                    label191.ForeColor = m_Vehicle.DriverDoorSts == 0 ? Color.AliceBlue : Color.DarkViolet;//主驾驶门
                    label192.ForeColor = m_Vehicle.PassangerDoorSts == 0 ? Color.AliceBlue : Color.DarkViolet; //副驾驶门
                    label195.ForeColor = m_Vehicle.TrunkSts == 0 ? Color.AliceBlue : Color.DarkViolet; //后备箱
                    label196.ForeColor = m_Vehicle.TurnLightLeftSts == 0 ? Color.AliceBlue : Color.Lime; //左转灯
                    label197.ForeColor = m_Vehicle.TurnLightRightSts == 0 ? Color.AliceBlue : Color.Lime; //右转灯
                    label198.ForeColor = m_Vehicle.DriverSeatBeltSwitchSts == 0 ? Color.AliceBlue : Color.Red; //主驾驶安全带

                    //执行器状态显示
                    label5.ForeColor = m_Vehicle.EPS_Status == 0 ? Color.AliceBlue : Color.BlueViolet;
                    label6.ForeColor = m_Vehicle.ESC_Status == 0 ? Color.AliceBlue : Color.BlueViolet;
                    label7.ForeColor = m_Vehicle.EPB_Status == 0 ? Color.AliceBlue : Color.BlueViolet;

                    //控制状态显示
                    label52.ForeColor = m_Vehicle.TargetAccelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                    label53.ForeColor = m_Vehicle.TargetDecelerationEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                    label54.ForeColor = m_Vehicle.TorqueEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                    label55.ForeColor = m_Vehicle.GearShiftEnable ? Color.DarkGoldenrod : Color.AliceBlue;
                    label108.ForeColor = m_Vehicle.VelocityEnable ? Color.DarkGoldenrod : Color.AliceBlue;

                    //控制信息
                    label59.Text = GearStatus[m_Vehicle.TargetGearShift];           //目标挡位
                    label60.Text = m_Vehicle.ActualAccelerationACC.ToString("F3");  //控制ACC
                    label61.Text = m_Vehicle.Torque.ToString("F3");                 //扭矩
                    label107.Text = m_Vehicle.TargetVehicleSpeed.ToString("F3");    //目标速度

                    //反馈车辆状态
                    label181.Text = GearStatus[m_Vehicle.ActualGearShift];//实际挡位位置
                    label182.Text = VehicleDirection[m_Vehicle.WheelSpeedDirection];//速度方向
                    label183.Text = EPB_SwitchPosition[m_Vehicle.EPB_SwitchPosition];//EPB挡位位置状态
                    label187.Text = APA_SystemState[m_Vehicle.SystemReadySts];//系统状态
                    label194.Text = DriverMode[m_Vehicle.AutoDriverModeSts];// 驾驶模式
                    label189.Text = m_Vehicle.Temperature.ToString("F1");// 环境温度显示

                    //转向角
                    label11.Text = m_Vehicle.SteeringAngleActual.ToString("F3");//转向角
                    label12.Text = m_Vehicle.TargetAccelerationACC.ToString("F3");//转向角速度

                    //轮速
                    label20.Text = m_Vehicle.VehicleSpeed.ToString("F3");               //后轴中心速度
                    label21.Text = m_Vehicle.WheelSpeedFrontLeftData.ToString("F3");    //左前轮速
                    label22.Text = m_Vehicle.WheelSpeedFrontRightData.ToString("F3");   //右前轮速
                    label23.Text = m_Vehicle.WheelSpeedRearLeftData.ToString("F3");     //左后轮速
                    label24.Text = m_Vehicle.WheelSpeedRearRightData.ToString("F3");    //右后轮速
                    //轮脉冲
                    label36.Text = m_Vehicle.WheelSpeedFrontLeftPulse.ToString("F3");   //左前脉冲
                    label37.Text = m_Vehicle.WheelSpeedFrontRightPulse.ToString("F3");  //右前脉冲
                    label38.Text = m_Vehicle.WheelSpeedRearLeftPulse.ToString("F3");    //左后脉冲
                    label39.Text = m_Vehicle.WheelSpeedRearRightPulse.ToString("F3");   //右后脉冲
                    label14.Text = m_Vehicle.WheelSpeedRearLeftPulseSum.ToString();//左后脉冲总数
                    label40.Text = m_Vehicle.WheelSpeedRearRightPulseSum.ToString();//右后脉冲总数
                    //传感器数据
                    label176.Text = m_Vehicle.LonAcc.ToString("F3");//lon
                    label177.Text = m_Vehicle.LatAcc.ToString("F3");//lat
                    label178.Text = m_Vehicle.YawRate.ToString("F3");//yaw_rate
                }));
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
            this.Invoke((EventHandler)(delegate
            {
                for (int i = 0; i < 8; i++)
                {
                    if (1 == UltrasonicDataType)
                    {
                        m_Ultrasonic.DataMapping2Control_STP318(m_LIN_STP318_ReadData[i], ref SensingControl_SRU[i]);
                    }
                    else if (2 == UltrasonicDataType)
                    {
                        m_Ultrasonic.DataMapping2Control_STP318Packet(m_Ultrasonic_Data_Packet[i], ref SensingControl_SRU[i]);
                    }
                }
                for (int i = 0; i < 12; i++)
                {
                    if (1 == UltrasonicDataType)
                    {
                        m_Ultrasonic.DataMapping2Control_STP318(m_LIN_STP318_Location_ReadData[i], ref SensingLocation_SRU[i / 3][i % 3]);
                    }
                    else if (2 == UltrasonicDataType)
                    {
                        m_Ultrasonic.DataMapping2Control_STP318Packet(m_Ultrasonic_Location_Data_Packet[i], ref SensingLocation_SRU[i / 3][i % 3]);
                    }
                }

                if (1 == UltrasonicDataType)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        m_Ultrasonic.DataMapping2Control_STP313(m_LIN_STP313_ReadData[i], ref SensingControl_LRU[i]);
                    }
                }
                else if (2 == UltrasonicDataType)
                {
                    for (int i = 8; i < 12; i++)
                    {
                        m_Ultrasonic.DataMapping2Control_STP313Packet(m_Ultrasonic_Data_Packet[i], ref SensingControl_LRU[i - 8]);
                    }
                }
            }));
        }

        private void UltrasonicImformationFormShow()
        {
            this.Invoke((EventHandler)(delegate
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
            }));
        }

        /// <summary>
        /// 三角定位坐标显示
        /// </summary>
        private void UltrasonicLocationFormShow()
        {
            this.Invoke((EventHandler)(delegate
            {
                UltrasonicBodyTriangleLocation.Points.Clear();
                for (int i = 0; i < 8; i++)
                {
                    if (UltrasonicStatus.Normal == m_BodyTriangleLocation[i].state)
                    {
                        UltrasonicBodyTriangleLocation.Points.AddXY(m_BodyTriangleLocation[i].x, m_BodyTriangleLocation[i].y);
                    }
                }
            }));
        }

        /// <summary>
        /// 直接定位坐标显示
        /// </summary>
        private void UltrasonicLocationDirectFormShow()
        {
            this.Invoke((EventHandler)(delegate
            {
                UltrasonicBodyDirectLocation.Points.Clear();
                for (int i = 8; i < 12; i++)
                {
                    if (UltrasonicStatus.Normal == m_BodyDirectLocation[i].state)
                    {
                        UltrasonicBodyDirectLocation.Points.AddXY(m_BodyDirectLocation[i].x, m_BodyDirectLocation[i].y);
                    }
                }
            }));
        }

        /// <summary>
        /// 三角定位地面坐标显示
        /// </summary>
        private void UltrasonicGroundLocationTriangleFormShow()
        {
            this.Invoke((EventHandler)(delegate
            {
                for (int i = 0; i < 8; i++)
                {
                    if (UltrasonicStatus.Normal == m_GroundTriangleLocation[i].state)
                    {
                        UltrasonicGroundTriangleLocation.Points.AddXY(m_GroundTriangleLocation[i].x, m_GroundTriangleLocation[i].y);
                    }
                }
            }));
        }

        /// <summary>
        /// 直接定位地面坐标显示
        /// </summary>
        private void UltrasonicGroundLocationDirectFormShow()
        {
            this.Invoke((EventHandler)(delegate
            {
                for (int i = 8; i < 12; i++)
                {
                    if (UltrasonicStatus.Normal == m_GroundDirectLocation[i].state)
                    {
                        UltrasonicGroundDirectLocation.Points.AddXY(m_GroundDirectLocation[i].x, m_GroundDirectLocation[i].y);
                    }
                }
            }));
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
                //label146.Text = m_Vehicle.VehicleSpeed.ToString();
                //label146.Text = m_Vehicle.TargetDistance.ToString();

                if ((0 != TrackPoint.Position.X) && ( 0 != TrackPoint.Position.Y))
                {
                    TrackDataShow.Points.AddXY(TrackPoint.Position.X, TrackPoint.Position.Y);
                }
        
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
            label135.Text = "转向角:" + m_Vehicle.SteeringAngleActual.ToString("F1");
            label136.Text = "轮速:" + (0.5 * (m_Vehicle.WheelSpeedRearLeftData + m_Vehicle.WheelSpeedRearRightData)).ToString("F2");
        }
        private void UltrasonicLocationShow()
        {
            double x, y;
            label165.Text = FrontParkingEdge.Position.X.ToString();
            label166.Text = FrontParkingEdge.Position.Y.ToString();
            label167.Text = RearParkingEdge.Position.X.ToString();
            label168.Text = RearParkingEdge.Position.Y.ToString();
            if(UltrasonicLocationStatus < 4)
            {
                label26.Text = u_location_status[UltrasonicLocationStatus];
            }
            // 超声数据推送
            label27.Text = FrontParkingPositionCnt.ToString();
            label28.Text = RearParkingPositionCnt.ToString();
            label170.Text = LeftParkingPositionCnt.ToString();
            label172.Text = RightParkingPositionCnt.ToString();

            // 障碍物距离避障信息
            label1.Text = FrontObstacle.Distance.ToString();
            label3.Text = region_status[FrontObstacle.region];
            label4.Text = valid_status[FrontObstacle.status];

            label2.Text = RearObstacle.Distance.ToString();
            label43.Text = region_status[RearObstacle.region];
            label50.Text = valid_status[RearObstacle.status];

            //ObstacleDistance_DataShow.Points.AddY(FrontObstacle.Distance);

            if (FitLineShowFlag)
            {
                if (checkBox13.Checked)
                {
                    x = -8;
                    y = Math.Tan(left_fit_line.Angle) * x + left_fit_line.Offset;
                    LeftFitLine_DataShow.Points.AddXY(x,y);
                    x = 4;
                    y = Math.Tan(left_fit_line.Angle) * x + left_fit_line.Offset;
                    LeftFitLine_DataShow.Points.AddXY(x, y);

                    x = -8;
                    y = Math.Tan(right_fit_line.Angle) * x + right_fit_line.Offset;
                    RightFitLine_DataShow.Points.AddXY(x, y);
                    x = 4;
                    y = Math.Tan(right_fit_line.Angle) * x + right_fit_line.Offset;
                    RightFitLine_DataShow.Points.AddXY(x, y);

                    x = -8;
                    y = Math.Tan(center_fit_line.Angle) * x + center_fit_line.Offset;
                    CenterFitLine_DataShow.Points.AddXY(x, y);
                    x = 4;
                    y = Math.Tan(center_fit_line.Angle) * x + center_fit_line.Offset;
                    CenterFitLine_DataShow.Points.AddXY(x, y);
                }


                if(checkBox12.Checked)
                {
                    UltrasonicObstacleLocationParkingPointDataShow.Points.AddXY(FrontParkingEdge.Position.X, FrontParkingEdge.Position.Y);
                    UltrasonicObstacleLocationParkingPointDataShow.Points.AddXY(RearParkingEdge.Position.X, RearParkingEdge.Position.Y);
                }

                FitLineShowFlag = false;
            }

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
        private void VelocityControlDataLog()
        {
            TestErrTime = timeGetTime() - TestLastTime;
            //数据保存
            if (DataSaveStatus)
            {
                DataSave.Write( "{0:D} " +
                                "{1:R} {2:R} " +
                                "{3:D} {4:D} " +
                                "{5:R} " +
                                "{6:R} {7:R} {8:R} " +
                                "{9:R} {10:R} " +
                                "{11:R} {12:R} " +
                                "{13:R} {14:R} " +
                                "{15:R} {16:R} {17:D} " +   // Steering Angle
                                "{18:R} {19:R} {20:R} " +   // Track
                                "{21:R} {22:R} {23:R} {24:R} {25:R} " +
                                "\r\n",
                                TestErrTime,
                                /// WheelSpeed
                                m_Vehicle.WheelSpeedRearLeftData,
                                m_Vehicle.WheelSpeedRearRightData,
                                // wheel pulse 
                                m_Vehicle.WheelSpeedRearLeftPulseSum,
                                m_Vehicle.WheelSpeedRearRightPulseSum,
                                // middle speed
                                m_Vehicle.VehicleSpeed,
                                // lon acc
                                m_Vehicle.LonAcc,
                                m_Vehicle.LatAcc,
                                m_Vehicle.YawRate,
                                // 嵌入式更新速度
                                m_Vehicle.PulseUpdateVelocity,
                                m_Vehicle.AccUpdateVelocity,
                                //控制扭矩和减速度
                                m_Vehicle.ActualAccelerationACC,
                                m_Vehicle.Torque,
                                m_Vehicle.TargetVehicleSpeed,
                                m_Vehicle.TargetDistance,
                                /// Steering Angle 
                                m_Vehicle.SteeringAngleTarget,
                                m_Vehicle.SteeringAngleActual,
                                m_Vehicle.SteeringAngleSpeed,
                                /// 跟踪位置信息
                                TrackPoint.Position.X,
                                TrackPoint.Position.Y,
                                TrackPoint.Yaw,
                                /// 滑模变量
                                TargetTrack.X,
                                TargetTrack.Y,
                                Sliding_x1,
                                Sliding_x2,
                                SlidingVariable
                );
            }
            TestLastTime = timeGetTime();
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
        /// 检车位专用的数据保存，与之前格式统一
        /// </summary>
        private void ParkingDetectionDataLogOld()
        {
            ErrTime = timeGetTime() - LastTime;
            //数据保存
            if (u_DataSaveStatus)
            {
                ULtrasonicDataSave.Write(
                                "{0:D} {1:D} " +
                                "{2:R16} {3:R16} " +
                                "{4:D} {5:R16} " +
                                "{6:R16} {7:R16} {8:R16} {9:R16} " +
                                "{10:R16} {11:R16} {12:R16} {13:R16} " +
                                "{14:R16} {15:R16} {16:R16} {17:R16} " +
                                "{18:R16} {19:R16} {20:R16} {21:R16} " +
                                "\r\n",
                                LastTime, ErrTime,
                                /// Vehicle Speed
                                m_Vehicle.VehicleSpeed,
                                m_Vehicle.VehicleSpeed,
                                /// Steering Angle 
                                m_Vehicle.SteeringAngleSpeed,
                                m_Vehicle.SteeringAngleActual,
                                // Ultrasonic
                                m_LIN_STP313_ReadData[0].TOF1 / 58.0,
                                m_LIN_STP313_ReadData[1].TOF1 / 58.0,
                                m_LIN_STP313_ReadData[2].TOF1 / 58.0,
                                m_LIN_STP313_ReadData[3].TOF1 / 58.0,
                                m_LIN_STP313_ReadData[0].Width * 16.0,
                                m_LIN_STP313_ReadData[1].Width * 16.0,
                                m_LIN_STP313_ReadData[2].Width * 16.0,
                                m_LIN_STP313_ReadData[3].Width * 16.0,
                                m_LIN_STP313_ReadData[0].TOF2 / 58.0,
                                m_LIN_STP313_ReadData[1].TOF2 / 58.0,
                                m_LIN_STP313_ReadData[2].TOF2 / 58.0,
                                m_LIN_STP313_ReadData[3].TOF2 / 58.0,
                                m_LIN_STP313_ReadData[0].Level * 3.3 / 255,
                                m_LIN_STP313_ReadData[1].Level * 3.3 / 255,
                                m_LIN_STP313_ReadData[2].Level * 3.3 / 255,
                                m_LIN_STP313_ReadData[3].Level * 3.3 / 255
                );
            }
            LastTime = timeGetTime();
        }

        /// <summary>
        /// 数据包保存
        /// </summary>
        private void ParkingDetectionPacketDataLog()
        {
            ParkingErrTime = timeGetTime() - ParkingLastTime;
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
            ParkingLastTime = timeGetTime();
        }

        /// <summary>
        /// 数据包保存
        /// </summary>
        private void UltrasonicPacketDataLog()
        {
            UltrasonicErrTime = timeGetTime() - UltrasonicLastTime;
            //数据保存
            if (u_DataSaveStatus)
            {
                ULtrasonicDataSave.Write(
                                "{0:D} " +
                                "{1:R16} {2:D} {3:R16} {4:D} " +
                                "{5:R16} {6:D} {7:R16} {8:D} " +
                                "{9:R16} {10:D} {11:R16} {12:D} " +
                                "{13:R16} {14:D} {15:R16} {16:D} " +
                                "{17:R16} {18:R16} {19:R16} {20:R16} {21:D} " +
                                "{22:R16} {23:R16} {24:R16} {25:R16} {26:D} " +
                                "{27:R16} {28:R16} {29:R16} {30:R16} {31:D} " +
                                "{32:R16} {33:R16} {34:R16} {35:R16} {36:D} " +
                                "{37:R16} {38:R16} {39:R16} " +
                                "{40:R16} {41:R16} {42:R16} {43:R16} " +
                                "\r\n",
                                UltrasonicErrTime,
                                // Ultrasonic
                                m_Ultrasonic_Data_Packet[0].Distance1,
                                m_Ultrasonic_Data_Packet[0].status,
                                m_Ultrasonic_Data_Packet[1].Distance1,
                                m_Ultrasonic_Data_Packet[1].status,
                                m_Ultrasonic_Data_Packet[2].Distance1,
                                m_Ultrasonic_Data_Packet[2].status,
                                m_Ultrasonic_Data_Packet[3].Distance1,
                                m_Ultrasonic_Data_Packet[3].status,
                                m_Ultrasonic_Data_Packet[4].Distance1,
                                m_Ultrasonic_Data_Packet[4].status,
                                m_Ultrasonic_Data_Packet[5].Distance1,
                                m_Ultrasonic_Data_Packet[5].status,
                                m_Ultrasonic_Data_Packet[6].Distance1,
                                m_Ultrasonic_Data_Packet[6].status,
                                m_Ultrasonic_Data_Packet[7].Distance1,
                                m_Ultrasonic_Data_Packet[7].status,
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
                                m_Ultrasonic_Data_Packet[11].status,

                                /// 跟踪位置信息
                                TrackPoint.Position.X,
                                TrackPoint.Position.Y,
                                TrackPoint.Yaw,
                                /// Steering Angle 
                                m_Vehicle.SteeringAngleActual,
                                m_Vehicle.VehicleSpeed,
                                /// WheelSpeed
                                m_Vehicle.WheelSpeedRearLeftData,
                                m_Vehicle.WheelSpeedRearRightData
                );
            }
            UltrasonicLastTime = timeGetTime();
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

        private void BodyTriangleLocationDataLog()
        {
            //数据保存
            if (u_DataSaveStatus)
            {
                BodyTriangleDataSave.Write(
                    "{0:D} {1:R16} {2:R16} " +
                    "{3:D} {4:R16} {5:R16} " +
                    "{6:D} {7:R16} {8:R16} " +
                    "{9:D} {10:R16} {11:R16} " +
                    "{12:D} {13:R16} {14:R16} " +
                    "{15:D} {16:R16} {17:R16} " +
                    "{18:D} {19:R16} {20:R16} " +
                    "{21:D} {22:R16} {23:R16} " +
                    "\r\n",
                    m_BodyTriangleLocation[0].state,
                    m_BodyTriangleLocation[0].x,
                    m_BodyTriangleLocation[0].y,
                    m_BodyTriangleLocation[1].state,
                    m_BodyTriangleLocation[1].x,
                    m_BodyTriangleLocation[1].y,
                    m_BodyTriangleLocation[2].state,
                    m_BodyTriangleLocation[2].x,
                    m_BodyTriangleLocation[2].y,
                    m_BodyTriangleLocation[3].state,
                    m_BodyTriangleLocation[3].x,
                    m_BodyTriangleLocation[3].y,
                    m_BodyTriangleLocation[4].state,
                    m_BodyTriangleLocation[4].x,
                    m_BodyTriangleLocation[4].y,
                    m_BodyTriangleLocation[5].state,
                    m_BodyTriangleLocation[5].x,
                    m_BodyTriangleLocation[5].y,
                    m_BodyTriangleLocation[6].state,
                    m_BodyTriangleLocation[6].x,
                    m_BodyTriangleLocation[6].y,
                    m_BodyTriangleLocation[7].state,
                    m_BodyTriangleLocation[7].x,
                    m_BodyTriangleLocation[7].y
                );
            }
        }
        
        private void PlanningInfDataLog()
        {
            PlanErrTime = timeGetTime() - PlanLastTime;
            //数据保存
            if (PlanningDataSaveStatus)
            {
                PlanningDataSave.Write(
                                "{0:D} " +
                                "{1:R} {2:R} {3:D} {4:R} " +
                                "{5:R} {6:R} " +
                                "{7:R} {8:R} " +
                                "{9:R} {10:R} {11:R} " +
                                "{12:R} {13:R} {14:R} " +
                                "{15:R} {16:R} " +
                                "{17:R} {18:R} {19:R} " +
                                "\r\n",
                                PlanErrTime,
                                /// Steering Angle 
                                m_Vehicle.SteeringAngleTarget,
                                m_Vehicle.SteeringAngleActual,
                                m_Vehicle.SteeringAngleSpeed,
                                m_Vehicle.SteeringTorque,
                                /// Vehicle Speed
                                m_Vehicle.TargetVehicleSpeed,
                                m_Vehicle.VehicleSpeed,
                                /// WheelSpeed
                                m_Vehicle.WheelSpeedRearLeftData,
                                m_Vehicle.WheelSpeedRearRightData,
                                /// 跟踪位置信息
                                TrackPoint.Position.X,
                                TrackPoint.Position.Y,
                                TrackPoint.Yaw,
                                /// 车辆姿态信息
                                m_Vehicle.LatAcc,
                                m_Vehicle.LonAcc,
                                m_Vehicle.YawRate,
                                TargetTrack.X,
                                TargetTrack.Y,
                                Sliding_x1,
                                Sliding_x2,
                                SlidingVariable
                );
            }
            PlanLastTime = timeGetTime();
        }

        private void LocationMapInfDataLog()
        {
            LocationMapErrTime = timeGetTime() - LocationMapLastTime;
            //数据保存
            if (LocationDataSaveStatus)
            {
                LocationMapDataSave.Write(
                                "{0:D} " +
                                "{1:R} {2:R} {3:D} " +
                                "{4:R} {5:R} {6:D} " +
                                "{7:R} {8:R} {9:D} " +
                                "{10:R} {11:R} {12:D} " +
                                "{13:R} {14:R} {15:D} " +
                                "{16:R} {17:R} {18:D} " +
                                "{19:R} {20:R} {21:D} " +
                                "{22:R} {23:R} {24:D} " +
                                "{25:R} {26:R} {27:D} " +
                                "{28:R} {29:R} {30:D} " +
                                "{31:R} {32:R} {33:D} " +
                                "{34:R} {35:R} {36:D} " +
                                "{37:R} {38:R} {39:R} " +
                                "{40:R16} {41:R16} {42:R16} {43:R16} {44:D} " +
                                "{45:R16} {46:R16} {47:R16} {48:R16} {49:D} " +
                                "{50:R16} {51:R16} {52:R16} {53:R16} {54:D} " +
                                "{55:R16} {56:R16} {57:R16} {58:R16} {59:D} " +
                                "\r\n",
                                LocationMapErrTime,
                                m_GroundTriangleLocation[0].x,
                                m_GroundTriangleLocation[0].y,
                                m_GroundTriangleLocation[0].state,
                                m_GroundTriangleLocation[1].x,
                                m_GroundTriangleLocation[1].y,
                                m_GroundTriangleLocation[1].state,
                                m_GroundTriangleLocation[2].x,
                                m_GroundTriangleLocation[2].y,
                                m_GroundTriangleLocation[2].state,
                                m_GroundTriangleLocation[3].x,
                                m_GroundTriangleLocation[3].y,
                                m_GroundTriangleLocation[3].state,
                                m_GroundTriangleLocation[4].x,
                                m_GroundTriangleLocation[4].y,
                                m_GroundTriangleLocation[4].state,
                                m_GroundTriangleLocation[5].x,
                                m_GroundTriangleLocation[5].y,
                                m_GroundTriangleLocation[6].state,
                                m_GroundTriangleLocation[6].x,
                                m_GroundTriangleLocation[6].y,
                                m_GroundTriangleLocation[6].state,
                                m_GroundTriangleLocation[7].x,
                                m_GroundTriangleLocation[7].y,
                                m_GroundTriangleLocation[7].state,
                                m_GroundDirectLocation[8].x,
                                m_GroundDirectLocation[8].y,
                                m_GroundDirectLocation[8].state,
                                m_GroundDirectLocation[9].x,
                                m_GroundDirectLocation[9].y,
                                m_GroundDirectLocation[9].state,
                                m_GroundDirectLocation[10].x,
                                m_GroundDirectLocation[10].y,
                                m_GroundDirectLocation[10].state,
                                m_GroundDirectLocation[11].x,
                                m_GroundDirectLocation[11].y,
                                m_GroundDirectLocation[11].state,
                                /// 跟踪位置信息
                                TrackPoint.Position.X,
                                TrackPoint.Position.Y,
                                TrackPoint.Yaw,
                                // 长距原始
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
            LocationMapLastTime = timeGetTime();
        }
        
        private void LocationResultInfDatalog()
        {
            if (LocationDataSaveStatus)
            {
                for(int i=0;i< LocationlistBox.Items.Count;i++)
                {
                    LocationResultDataSave.Write(LocationlistBox.Items[i].ToString());
                }  
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

        #region 串口发送接口
        void BLDC_DeliverySend()
        {
            byte[] Data = new byte[100];//动态分配内存
            byte CRC_Sum = 0;
            byte N = 2;
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    CRC_Sum = 0;
                    Data[0] = 0x55; //识别标志1
                    Data[1] = 0x77; //识别标志2
                    Data[2] = (byte)(3 * N + 3);    //数据长度
                    Data[3] = 2;    //数据标志 
                    Data[4] = 1;    //编号
                    Data[5] = N;

                    Data[6] = Convert.ToByte(textBox12.Text);
                    Data[7] = (byte)(Convert.ToSingle(textBox40.Text) * 100);
                    Data[8] = (byte)(Convert.ToSingle(textBox41.Text) * 20);

                    Data[9] = Convert.ToByte(textBox12.Text);
                    Data[10] = (byte)(Convert.ToSingle(textBox40.Text) * 100);
                    Data[11] = (byte)(Convert.ToSingle(textBox41.Text) * 20);

                    for (int i = 0; i < Data[2]; i++)
                    {
                        CRC_Sum = (byte)(CRC_Sum + Data[i + 3]);
                    }
                    Data[Data[2] + 3] = CRC_Sum;

                    serialPort1.Write(Data, 0, Data[2] + 4);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
        }

        void BLDC_DeliveryMaxSend()
        {
            byte[] Data = new byte[100];//动态分配内存
            byte CRC_Sum = 0;
            byte N = 2;
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    
                    Data[0] = 0x55; //识别标志1
                    Data[1] = 0x77; //识别标志2
                    Data[2] = 93;    //数据长度
                    Data[3] = 2;    //数据标志 
                    Data[4] = 1;    //编号
                    Data[5] = N;

                    //Data[6] = Convert.ToByte(textBox12.Text);
                    //Data[7] = (byte)(Convert.ToSingle(textBox40.Text) * 100);
                    //Data[8] = (byte)(Convert.ToSingle(textBox41.Text) * 20);

                    //Data[9] = Convert.ToByte(textBox12.Text);
                    //Data[10] = (byte)(Convert.ToSingle(textBox40.Text) * 100);
                    //Data[11] = (byte)(Convert.ToSingle(textBox41.Text) * 20);

                    for (int i = 0; i < 30; i++)
                    {
                        Data[3 * i + 6] = Convert.ToByte(textBox12.Text);
                        Data[3 * i + 7] = (byte)(Convert.ToSingle(textBox40.Text) * 100);
                        Data[3 * i + 8] = (byte)(Convert.ToSingle(textBox41.Text) * 20);
                    }
                    Data[95] = 0xff;
                    CRC_Sum = Data[2];
                    for (int i = 0; i < Data[2]; i++)
                    {
                        CRC_Sum = (byte)(CRC_Sum + Data[i + 3]);
                    }
                    Data[96] = CRC_Sum;

                    serialPort1.Write(Data, 0, Data[2] + 4);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
        }

        void BLDC_ResetSend()
        {
            byte[] Data = new byte[10];//动态分配内存
            byte CRC_Sum = 0;
            byte N = 1;
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    //CRC_Sum = 0;
                    Data[0] = 0x55; //识别标志1
                    Data[1] = 0x77; //识别标志2
                    Data[2] = 4;    //数据长度
                    Data[3] = 3;    //数据标志 
                    Data[4] = Convert.ToByte(textBox42.Text);
                    Data[5] = Convert.ToByte(textBox43.Text);
                    Data[6] = 0;

                    CRC_Sum = Data[2];
                    for (int i = 0; i < Data[2]; i++)
                    {
                        CRC_Sum = (byte)(CRC_Sum + Data[i + 3]);
                    }
                    Data[Data[2] + 3] = CRC_Sum;

                    serialPort1.Write(Data, 0, Data[2] + 4);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
        }

        void BLDC_DQ_Send()
        {
            byte[] Data = new byte[10];//动态分配内存
            byte CRC_Sum = 0;
            byte[] Data_Temp = new byte[4];//动态分配内存
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    //CRC_Sum = 0;
                    Data[0] = 0x55; //识别标志1
                    Data[1] = 0x77; //识别标志2
                    Data[2] = 6;    //数据长度
                    Data[3] = 5;    //数据标志 
                    Data_Temp = BitConverter.GetBytes( Convert.ToInt16( Convert.ToSingle(textBox36.Text) * 100 ));// V_D
                    Data[4] = Data_Temp[0];
                    Data[5] = Data_Temp[1];
                    Data_Temp = BitConverter.GetBytes( Convert.ToInt16(Convert.ToSingle(textBox37.Text) * 100 ));// V_Q
                    Data[6] = Data_Temp[0];
                    Data[7] = Data_Temp[1];
                    Data[8] = 0;
                    CRC_Sum = Data[2];
                    for (int i = 0; i < Data[2]; i++)
                    {
                        CRC_Sum = (byte)(CRC_Sum + Data[i + 3]);
                    }
                    Data[Data[2] + 3] = CRC_Sum;

                    serialPort1.Write(Data, 0, Data[2] + 4);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
        }

        void BLDC_Speed_Send()
        {
            byte[] Data = new byte[10];//动态分配内存
            byte CRC_Sum = 0;
            byte[] Data_Temp = new byte[4];//动态分配内存
            //发送指令
            if (serialPort1.IsOpen)
            {
                try
                {
                    //CRC_Sum = 0;
                    Data[0] = 0x55; //识别标志1
                    Data[1] = 0x77; //识别标志2
                    Data[2] = 6;    //数据长度
                    Data[3] = 6;    //数据标志 
                    Data_Temp = BitConverter.GetBytes(Convert.ToInt16(Convert.ToSingle(textBox38.Text) * 100));// Target Speed
                    Data[4] = Data_Temp[0];
                    Data[5] = Data_Temp[1];
                    Data_Temp = BitConverter.GetBytes(Convert.ToInt16(Convert.ToSingle(textBox39.Text) * 100));// Target Position
                    Data[6] = Data_Temp[0];
                    Data[7] = Data_Temp[1];
                    Data[8] = 0;
                    CRC_Sum = Data[2];
                    for (int i = 0; i < Data[2]; i++)
                    {
                        CRC_Sum = (byte)(CRC_Sum + Data[i + 3]);
                    }
                    Data[Data[2] + 3] = CRC_Sum;

                    serialPort1.Write(Data, 0, Data[2] + 4);
                }
                catch
                {
                    MessageBox.Show("数据类型错误，请检查所发数据类型", "错误提示");
                }
            }
        }
        #endregion

        #endregion
        #region 事件
        #region 初始窗口事件
        public Form1()
        {
            InitializeComponent();
            //newBitmap.Palette = GreyColorPalette;
            //newBitmap = new Bitmap(300, 600);
            //newBitmap.SetPixel(1,1,Color.Black);
            //newBitmap.SetPixel(1, 2, Color.Black);
            //newBitmap.SetPixel(2, 2, Color.Black);
            //newBitmap.SetPixel(2, 1, Color.Black);
            //pictureBox1.Image = newBitmap;

            #region 超声波数据显示Chart
            ///超声波数据显示
            //ultrasonic_chart.ChartAreas[0].AxisX.Maximum = 1.5;
            //ultrasonic_chart.ChartAreas[0].AxisX.Minimum = 0;
            //ultrasonic_chart.ChartAreas[0].AxisY.Maximum = 1;
            //ultrasonic_chart.ChartAreas[0].AxisY.Minimum = -1;
            //ultrasonic_chart.ChartAreas[0].AxisX.Interval = 0.01;
            //ultrasonic_chart.ChartAreas[0].AxisY.Interval = 0.1;

            ultrasonic_chart.ChartAreas[0].AxisX.Maximum = 14;
            ultrasonic_chart.ChartAreas[0].AxisX.Minimum = -10;
            ultrasonic_chart.ChartAreas[0].AxisY.Maximum = 6;
            ultrasonic_chart.ChartAreas[0].AxisY.Minimum = -6;
            ultrasonic_chart.ChartAreas[0].AxisX.Interval = 1;
            ultrasonic_chart.ChartAreas[0].AxisY.Interval = 1;

            ultrasonic_chart.Series.Add(UltrasonicDataShow);
            ultrasonic_chart.Series.Add(UltrasonicVehicle); //超声车身
            ultrasonic_chart.Series.Add(UltrasonicBodyDirectLocation);//超声障碍物定位(车体坐标系)
            ultrasonic_chart.Series.Add(UltrasonicBodyTriangleLocation);//超声障碍物定位(车体坐标系)
            

            UltrasonicDataShow.ChartType = SeriesChartType.Point;
            UltrasonicDataShow.BorderWidth = 5;
            UltrasonicDataShow.BorderDashStyle = ChartDashStyle.Dash;
            UltrasonicDataShow.Color = Color.Green;
            UltrasonicDataShow.IsVisibleInLegend = true;
            UltrasonicDataShow.LegendText = "超声波数据";

            UltrasonicVehicle.ChartType = SeriesChartType.FastLine;
            UltrasonicVehicle.BorderWidth = 5;
            UltrasonicVehicle.BorderDashStyle = ChartDashStyle.Solid;
            UltrasonicVehicle.Color = Color.DarkRed;
            UltrasonicVehicle.IsVisibleInLegend = true;
            UltrasonicVehicle.LegendText = "车身模型";

            UltrasonicBodyTriangleLocation.ChartType = SeriesChartType.Point;
            UltrasonicBodyTriangleLocation.BorderWidth = 3;
            UltrasonicBodyTriangleLocation.BorderDashStyle = ChartDashStyle.Dash;
            UltrasonicBodyTriangleLocation.Color = Color.Green;
            UltrasonicBodyTriangleLocation.IsVisibleInLegend = true;
            UltrasonicBodyTriangleLocation.LegendText = "障碍物三角定位";
            //UltrasonicBodyTriangleLocation.IsValueShownAsLabel = true;
            //UltrasonicLocation.


            UltrasonicBodyDirectLocation.ChartType = SeriesChartType.Point;
            UltrasonicBodyDirectLocation.BorderWidth = 3;
            UltrasonicBodyDirectLocation.BorderDashStyle = ChartDashStyle.Dash;
            UltrasonicBodyDirectLocation.Color = Color.Red;
            UltrasonicBodyDirectLocation.IsVisibleInLegend = true;
            UltrasonicBodyDirectLocation.LegendText = "障碍物直接定位";
            //UltrasonicBodyDirectLocation.IsValueShownAsLabel = true;
            #endregion

            #region 车辆跟踪Chart显示
            /// 车辆泊车图形显示
            track_chart.ChartAreas[0].AxisX.Maximum = 80;
            track_chart.ChartAreas[0].AxisX.Minimum = -80;
            track_chart.ChartAreas[0].AxisY.Maximum = 40;
            track_chart.ChartAreas[0].AxisY.Minimum = -40;
            //track_chart.ChartAreas[0].AxisX.Interval = 0.5;
            //track_chart.ChartAreas[0].AxisY.Interval = 0.5;

            //track_chart.ChartAreas[0].AxisX.Maximum = 20;
            //track_chart.ChartAreas[0].AxisX.Minimum = -20;
            //track_chart.ChartAreas[0].AxisY.Maximum = 10;
            //track_chart.ChartAreas[0].AxisY.Minimum = -10;

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

            
            track_chart.Series.Add(UltrasonicGroundDirectLocation);
            UltrasonicGroundDirectLocation.ChartType = SeriesChartType.FastPoint;
            //UltrasonicGroundDirectLocation.BorderWidth = 2;
            UltrasonicGroundDirectLocation.BorderDashStyle = ChartDashStyle.Solid;
            UltrasonicGroundDirectLocation.Color = Color.DarkRed;
            UltrasonicGroundDirectLocation.IsVisibleInLegend = true;
            UltrasonicGroundDirectLocation.LegendText = "地面坐标系直接定位";

            track_chart.Series.Add(UltrasonicGroundTriangleLocation);
            UltrasonicGroundTriangleLocation.ChartType = SeriesChartType.FastPoint;
            //UltrasonicGroundTriangleLocation.BorderWidth = 2;
            UltrasonicGroundTriangleLocation.BorderDashStyle = ChartDashStyle.Solid;
            UltrasonicGroundTriangleLocation.Color = Color.DarkBlue;
            UltrasonicGroundTriangleLocation.IsVisibleInLegend = true;
            UltrasonicGroundTriangleLocation.LegendText = "地面坐标系三角定位";
            #endregion

            #region 超声波障碍物定位Chart
            //UltrasonicLocationChart.ChartAreas[0].AxisX.Maximum = 1;
            //UltrasonicLocationChart.ChartAreas[0].AxisX.Minimum = -10;
            //UltrasonicLocationChart.ChartAreas[0].AxisY.Maximum = 9;
            //UltrasonicLocationChart.ChartAreas[0].AxisY.Minimum = -9;
            //UltrasonicLocationChart.ChartAreas[0].AxisX.MaximumAutoSize = 100;
            //超声障碍物定位长距传感器数据显示
            UltrasonicLocationChart.Series.Add(UltrasonicObstacleLocation_LRU_DataShow);
            UltrasonicObstacleLocation_LRU_DataShow.ChartType = SeriesChartType.FastPoint;
            UltrasonicObstacleLocation_LRU_DataShow.BorderWidth = 2;
            UltrasonicObstacleLocation_LRU_DataShow.MarkerSize = 3;
            UltrasonicObstacleLocation_LRU_DataShow.BorderDashStyle = ChartDashStyle.Solid;
            UltrasonicObstacleLocation_LRU_DataShow.Color = Color.DarkBlue;
            UltrasonicObstacleLocation_LRU_DataShow.IsVisibleInLegend = true;
            UltrasonicObstacleLocation_LRU_DataShow.LegendText = "长距";

            //超声障碍物定位短距传感器数据显示
            UltrasonicLocationChart.Series.Add(UltrasonicObstacleLocation_SRU_DataShow);
            UltrasonicObstacleLocation_SRU_DataShow.ChartType = SeriesChartType.FastPoint;
            UltrasonicObstacleLocation_SRU_DataShow.BorderWidth = 2;
            UltrasonicObstacleLocation_SRU_DataShow.MarkerSize = 3;
            UltrasonicObstacleLocation_SRU_DataShow.BorderDashStyle = ChartDashStyle.Solid;
            UltrasonicObstacleLocation_SRU_DataShow.Color = Color.DarkGreen;
            UltrasonicObstacleLocation_SRU_DataShow.IsVisibleInLegend = true;
            UltrasonicObstacleLocation_SRU_DataShow.LegendText = "短距";
            //超声障碍物定位检测库位点数据显示
            UltrasonicLocationChart.Series.Add(UltrasonicObstacleLocationParkingPointDataShow);
            UltrasonicObstacleLocationParkingPointDataShow.ChartType = SeriesChartType.FastPoint;
            UltrasonicObstacleLocationParkingPointDataShow.BorderWidth = 9;
            UltrasonicObstacleLocationParkingPointDataShow.MarkerSize = 9;
            UltrasonicObstacleLocationParkingPointDataShow.BorderDashStyle = ChartDashStyle.NotSet;
            UltrasonicObstacleLocationParkingPointDataShow.Color = Color.DarkRed;
            UltrasonicObstacleLocationParkingPointDataShow.IsVisibleInLegend = true;
            UltrasonicObstacleLocationParkingPointDataShow.LegendText = "库位";

            UltrasonicLocationChart.Series.Add(CenterFitLine_DataShow);
            CenterFitLine_DataShow.ChartType = SeriesChartType.FastLine;
            CenterFitLine_DataShow.BorderWidth = 2;
            CenterFitLine_DataShow.MarkerSize = 3;
            CenterFitLine_DataShow.BorderDashStyle = ChartDashStyle.NotSet;
            CenterFitLine_DataShow.Color = Color.YellowGreen;
            CenterFitLine_DataShow.IsVisibleInLegend = true;
            CenterFitLine_DataShow.LegendText = "中心线";
            //库位左边界拟合直线
            UltrasonicLocationChart.Series.Add(LeftFitLine_DataShow);
            LeftFitLine_DataShow.ChartType = SeriesChartType.FastLine;
            LeftFitLine_DataShow.BorderWidth = 2;
            LeftFitLine_DataShow.MarkerSize = 3;
            LeftFitLine_DataShow.BorderDashStyle = ChartDashStyle.NotSet;
            LeftFitLine_DataShow.Color = Color.Yellow;
            LeftFitLine_DataShow.IsVisibleInLegend = true;
            LeftFitLine_DataShow.LegendText = "左边线";
            //库位右边界拟合直线
            UltrasonicLocationChart.Series.Add(RightFitLine_DataShow);
            RightFitLine_DataShow.ChartType = SeriesChartType.FastLine;
            RightFitLine_DataShow.BorderWidth = 2;
            RightFitLine_DataShow.MarkerSize = 3;
            RightFitLine_DataShow.BorderDashStyle = ChartDashStyle.NotSet;
            RightFitLine_DataShow.Color = Color.Green;
            RightFitLine_DataShow.IsVisibleInLegend = true;
            RightFitLine_DataShow.LegendText = "右边线";

            UltrasonicLocationChart.Series.Add(ObstacleDistance_DataShow);
            ObstacleDistance_DataShow.ChartType = SeriesChartType.FastLine;
            ObstacleDistance_DataShow.BorderWidth = 2;
            ObstacleDistance_DataShow.MarkerSize = 3;
            ObstacleDistance_DataShow.BorderDashStyle = ChartDashStyle.Solid;
            ObstacleDistance_DataShow.Color = Color.BurlyWood;
            ObstacleDistance_DataShow.IsVisibleInLegend = true;
            ObstacleDistance_DataShow.LegendText = "障碍物距离";

            #endregion


            #region 串口初始化配置
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(serialPortDataReceived);
            serialPort1.Encoding = Encoding.GetEncoding("GB2312");
            m_SerialCom.SearchAndAddSerialToComboBox(serialPort1, comboBox1);
            m_SerialCom.AddBaudRate(comboBox2);
            #endregion

            #region BLDC波形显示 chart

            //BLDC_chart.ChartAreas[0].AxisX.Maximum = 1;
            //BLDC_chart.ChartAreas[0].AxisX.Minimum = -10;


            BLDC_chart.Series.Add(BLDC_Phase_IA_Show);
            BLDC_Phase_IA_Show.ChartType = SeriesChartType.FastLine;
            BLDC_Phase_IA_Show.BorderWidth = 2;
            BLDC_Phase_IA_Show.MarkerSize = 3;
            BLDC_Phase_IA_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_Phase_IA_Show.Color = Color.DarkRed;
            BLDC_Phase_IA_Show.IsVisibleInLegend = true;
            BLDC_Phase_IA_Show.LegendText = "A相电流";

            BLDC_chart.Series.Add(BLDC_Phase_IB_Show);
            BLDC_Phase_IB_Show.ChartType = SeriesChartType.FastLine;
            BLDC_Phase_IB_Show.BorderWidth = 2;
            BLDC_Phase_IB_Show.MarkerSize = 3;
            BLDC_Phase_IB_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_Phase_IB_Show.Color = Color.YellowGreen;
            BLDC_Phase_IB_Show.IsVisibleInLegend = true;
            BLDC_Phase_IB_Show.LegendText = "B相电流";

            BLDC_chart.Series.Add(BLDC_Phase_IC_Show);
            BLDC_Phase_IC_Show.ChartType = SeriesChartType.FastLine;
            BLDC_Phase_IC_Show.BorderWidth = 2;
            BLDC_Phase_IC_Show.MarkerSize = 3;
            BLDC_Phase_IC_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_Phase_IC_Show.Color = Color.DarkBlue;
            BLDC_Phase_IC_Show.IsVisibleInLegend = true;
            BLDC_Phase_IC_Show.LegendText = "C相电流";

            BLDC_chart.Series.Add(BLDC_VBUS_Show);
            BLDC_VBUS_Show.ChartType = SeriesChartType.FastLine;
            BLDC_VBUS_Show.BorderWidth = 1;
            BLDC_VBUS_Show.MarkerSize = 2;
            BLDC_VBUS_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_VBUS_Show.Color = Color.Black;
            BLDC_VBUS_Show.IsVisibleInLegend = true;
            BLDC_VBUS_Show.LegendText = "电机电压";

            BLDC_chart.Series.Add(BLDC_PositionShow);
            BLDC_PositionShow.ChartType = SeriesChartType.FastLine;
            BLDC_PositionShow.BorderWidth = 1;
            BLDC_PositionShow.MarkerSize = 2;
            BLDC_PositionShow.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_PositionShow.Color = Color.DarkOrange;
            BLDC_PositionShow.IsVisibleInLegend = true;
            BLDC_PositionShow.LegendText = "电机位置";

            BLDC_chart.Series.Add(BLDC_VelocityShow);
            BLDC_VelocityShow.ChartType = SeriesChartType.FastLine;
            BLDC_VelocityShow.BorderWidth = 1;
            BLDC_VelocityShow.MarkerSize = 2;
            BLDC_VelocityShow.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_VelocityShow.Color = Color.DarkGreen;
            BLDC_VelocityShow.IsVisibleInLegend = true;
            BLDC_VelocityShow.LegendText = "电机转速";

            BLDC_chart.Series.Add(BLDC_Current_D_Show);
            BLDC_Current_D_Show.ChartType = SeriesChartType.FastLine;
            BLDC_Current_D_Show.BorderWidth = 2;
            BLDC_Current_D_Show.MarkerSize = 3;
            BLDC_Current_D_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_Current_D_Show.Color = Color.DarkMagenta;
            BLDC_Current_D_Show.IsVisibleInLegend = true;
            BLDC_Current_D_Show.LegendText = "D电流";

            BLDC_chart.Series.Add(BLDC_Current_Q_Show);
            BLDC_Current_Q_Show.ChartType = SeriesChartType.FastLine;
            BLDC_Current_Q_Show.BorderWidth = 2;
            BLDC_Current_Q_Show.MarkerSize = 3;
            BLDC_Current_Q_Show.BorderDashStyle = ChartDashStyle.NotSet;
            BLDC_Current_Q_Show.Color = Color.GreenYellow;
            BLDC_Current_Q_Show.IsVisibleInLegend = true;
            BLDC_Current_Q_Show.LegendText = "Q电流";
            #endregion

            // add the thread of the can receive
            //ThreadStart CANTreadChild = new ThreadStart(CallToCANReceiveThread);
            //Thread m_CanReceiveChildThread = new Thread(CANTreadChild);
            //m_CanReceiveChildThread.Priority = ThreadPriority.Normal;
            //m_CanReceiveChildThread.IsBackground = true;
            //m_CanReceiveChildThread.Start();

            // add the thread of the winform show
            //ThreadStart FormShowTreadChild = new ThreadStart(FormShowThread);
            //Thread m_FormShowChildThread = new Thread(FormShowTreadChild);
            //m_FormShowChildThread.Priority = ThreadPriority.Normal;
            //m_FormShowChildThread.IsBackground = true;
            //m_FormShowChildThread.Start();
        }
        private void Form1_Load(object sender, EventArgs e)
        {
            label191.ForeColor = Color.AliceBlue;   //主驾驶门
            label192.ForeColor = Color.AliceBlue;   //副驾驶门
            label195.ForeColor = Color.AliceBlue;   //后备箱
            label196.ForeColor = Color.AliceBlue;   //左转灯
            label197.ForeColor = Color.AliceBlue;   //右转灯
            label198.ForeColor = Color.AliceBlue;   //主驾驶安全带

            label5.ForeColor = Color.AliceBlue;
            label6.ForeColor = Color.AliceBlue;
            label7.ForeColor = Color.AliceBlue;

            label52.ForeColor = Color.DarkGoldenrod ;
            label53.ForeColor = Color.DarkGoldenrod ;
            label54.ForeColor = Color.DarkGoldenrod ;
            label55.ForeColor = Color.DarkGoldenrod ;
            label108.ForeColor = Color.DarkGoldenrod;

            timer_show.Enabled = true;

            for (int i = 0; i < 8; i++)
            {
                comboBox3.Items.Add(GearStatus[i]);
            }
            comboBox3.SelectedIndex = 1;

            for(int i = 0; i < 5;i++)
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

            SensingLocation_SRU = new Label[4][]
            {
                new Label[3]{ label150, label151,label152 },
                new Label[3]{ label153, label154,label155 },
                new Label[3]{ label156, label157,label158 },
                new Label[3]{ label159, label160,label161 }
            };

            u_color = new Color[4] { Color.Red,Color.Green,Color.Green,Color.Red};
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

            LocationPoint p = new LocationPoint();
            p.Position.X = 0;
            p.Position.Y = 0;
            p.Yaw = 0;

            UltrasonicVehicle.Points.Clear();
            AxisRotation(p, FrontLeftDiagonal, ref VehicleEdgePoint[0]);
            AxisRotation(p, FrontRightDiagonal, ref VehicleEdgePoint[1]);
            AxisRotation(p, RearLeftDiagonal, ref VehicleEdgePoint[3]);
            AxisRotation(p, RearRightDiagonal, ref VehicleEdgePoint[2]);
            for (int i = 0; i < 4; i++)
            {
                UltrasonicVehicle.Points.AddXY(VehicleEdgePoint[i].X, VehicleEdgePoint[i].Y);
            }
            UltrasonicVehicle.Points.AddXY(VehicleEdgePoint[0].X, VehicleEdgePoint[0].Y);
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
                    //m_ZLGCAN.CAN_Receive(VehicleReceiveCAN, ref obj);
                    //for (int i = 0; i < obj.Length-1; i++)
                    //{
                    //    VehicleParse(obj[i]);
                    //}
                    //m_ZLGCAN.CAN_Receive(VehicleSendCAN, ref obj);
                    //for (int i = 0; i < obj.Length-1; i++)
                    //{
                    //    VehicleSendParse(obj[i]);
                    //}
                }
                //Thread.Sleep(50);
            }
        }
        #endregion

        #region 串口操作事件
        private void serialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (m_SerialCom.Closing) return;//如果正在关闭，忽略操作，直接返回，尽快的完成串口监听线程的一次循环  
            try
            {
                m_SerialCom.Listening = true;//设置标记，说明我已经开始处理数据，一会儿要使用系统UI的。
                int n = serialPort1.BytesToRead;//先记录下来，避免某种原因，人为的原因，操作几次之间时间长，缓存不一致  
                byte[] data = new byte[n];
                //received_count += n;//增加接收计数  
                serialPort1.Read(data, 0, n);//读取缓冲数据 

                m_SerialCom.ReceiveDataProcess(data);

                if (m_SerialCom.DataCatched)
                {
                    //更新界面  
                    this.Invoke((EventHandler)(delegate
                    {
                        if (m_SerialCom.BinaryData[0] == 0x85)
                        {
                            BLDC_VBUS = m_SerialCom.BinaryData[1] * 0.2f;
                            BLDC_VBUS_I = m_SerialCom.BinaryData[2] * 0.1f;
                            BLDC_Position = BitConverter.ToUInt16(m_SerialCom.BinaryData, 3) * 2 * 57.3f;
                            //BLDC_Position = BitConverter.ToUInt16(m_SerialCom.BinaryData, 3) * 0.01f;

                            BLDC_Velocity = BitConverter.ToInt16(m_SerialCom.BinaryData, 5) * 0.1f;
                            //BLDC_Velocity = BitConverter.ToInt16(m_SerialCom.BinaryData, 5) * 2 * 57.3f;

                            label208.Text = BLDC_VBUS_I.ToString("F3");

                            if (checkBox15.Checked)
                            {
                                BLDC_VBUS_Show.Points.AddY(BLDC_VBUS);
                                while (BLDC_VBUS_Show.Points.Count > 50)
                                {
                                    BLDC_VBUS_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label207.Text = BLDC_VBUS.ToString("F3");
                            }

                            if (checkBox20.Checked)
                            {
                                BLDC_PositionShow.Points.AddY(BLDC_Position);
                                while (BLDC_PositionShow.Points.Count > 50)
                                {
                                    BLDC_PositionShow.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label209.Text = BLDC_Position.ToString("F3");
                            }

                            if (checkBox21.Checked)
                            {
                                BLDC_VelocityShow.Points.AddY(BLDC_Velocity);
                                while (BLDC_VelocityShow.Points.Count > 50)
                                {
                                    BLDC_VelocityShow.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label210.Text = BLDC_Velocity.ToString("F3");
                            }
                            //label211.Text = BLDC_WorkState[(m_SerialCom.BinaryData[7] >> 7) & 0x01];
                            //label212.Text = BLDC_MotorDirection[(m_SerialCom.BinaryData[7] >> 5) & 0x03];
                            label212.Text = m_SerialCom.BinaryData[7].ToString("X");
                        }
                        else if (m_SerialCom.BinaryData[0] == 0x86)
                        {
                            BLDC_TargetPosition = BitConverter.ToUInt16(m_SerialCom.BinaryData, 1) * 2;
                            BLDC_ActualPosition = BitConverter.ToUInt16(m_SerialCom.BinaryData, 3) * 2;
                            BLDC_TargetTurn = m_SerialCom.BinaryData[5];
                            BLDC_ActualTurn = m_SerialCom.BinaryData[6];

                            label244.Text = BLDC_TargetPosition.ToString();
                            label245.Text = BLDC_ActualPosition.ToString();
                            label248.Text = BLDC_TargetTurn.ToString();
                            label246.Text = BLDC_ActualTurn.ToString();
                        }
                        else if (m_SerialCom.BinaryData[0] == 0x91) // 三相电流
                        {
                            BLDC_PhaseIA = BitConverter.ToInt16(m_SerialCom.BinaryData, 1) * 0.001f;
                            BLDC_PhaseIB = BitConverter.ToInt16(m_SerialCom.BinaryData, 3) * 0.001f;
                            BLDC_PhaseIC = BitConverter.ToInt16(m_SerialCom.BinaryData, 5) * 0.001f;
                                                  
                            if (checkBox16.Checked)
                            {
                                BLDC_Phase_IA_Show.Points.AddY(BLDC_PhaseIA); //A相电流
                                while (BLDC_Phase_IA_Show.Points.Count > 500)
                                {
                                    BLDC_Phase_IA_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label219.Text = BLDC_PhaseIA.ToString("F3");
                            }

                            if (checkBox17.Checked)
                            {
                                BLDC_Phase_IB_Show.Points.AddY(BLDC_PhaseIB); //B相电流
                                while (BLDC_Phase_IB_Show.Points.Count > 500)
                                {
                                    BLDC_Phase_IB_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label220.Text = BLDC_PhaseIB.ToString("F3");
                            }

                            if (checkBox18.Checked)
                            {
                                BLDC_Phase_IC_Show.Points.AddY(BLDC_PhaseIC); //C相电流
                                while (BLDC_Phase_IC_Show.Points.Count > 500)
                                {
                                    BLDC_Phase_IC_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label221.Text = BLDC_PhaseIC.ToString("F3");
                            }

                            //label211.Text = m_SerialCom.BinaryData[7].ToString("X");
                        }
                        else if (m_SerialCom.BinaryData[0] == 0x92) // 转换电流DQ
                        {
                            BLDC_Current_D = BitConverter.ToInt16(m_SerialCom.BinaryData, 1) * 0.001f;
                            BLDC_Current_Q = BitConverter.ToInt16(m_SerialCom.BinaryData, 3) * 0.001f;

                            if (checkBox22.Checked)
                            {
                                BLDC_Current_D_Show.Points.AddY(BLDC_Current_D);
                                while (BLDC_Current_D_Show.Points.Count > 500)
                                {
                                    BLDC_Current_D_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label233.Text = BLDC_Current_D.ToString("F3");
                            }

                            if (checkBox23.Checked)
                            {
                                BLDC_Current_Q_Show.Points.AddY(BLDC_Current_Q);
                                while (BLDC_Current_Q_Show.Points.Count > 500)
                                {
                                    BLDC_Current_Q_Show.Points.RemoveAt(0);
                                }
                            }
                            else
                            {
                                label234.Text = BLDC_Current_Q.ToString("F3");
                            }
                        }
                    }));
                }
            }
            finally
            {
                m_SerialCom.Listening = false;//我用完了，ui可以关闭串口了。     
            }// end try              
        }
        /**
         * 串口打开 关闭事件
         */
        private void button46_Click(object sender, EventArgs e)
        {
            m_SerialCom.OpenAndCloseSerial(serialPort1, button46, comboBox1.Text, Convert.ToInt32(comboBox2.Text));
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

        /// <summary>
        /// PID参数设置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button3_Click(object sender, EventArgs e)
        {
            PID_ParameterConfigureCAN();
        }

        /// <summary>
        /// 长安对接测试
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button10_Click(object sender, EventArgs e)
        {
            //ChangAnInterfaceCAN();

            ControlInterfaceCAN();
            EmergencyInterfaceCAN();

            //DongFengInterfaceCAN();
        }
        #endregion

        #region 定时器事件
        private void timer_show_Tick(object sender, EventArgs e)
        {
            //if (m_ZLGCAN.OpenStatus == 1)
            //{
            //    ZLGCAN.VCI_CAN_OBJ[] obj = new ZLGCAN.VCI_CAN_OBJ[1];
            //    m_ZLGCAN.CAN_Receive(TerminalCAN, ref obj);
            //    for (int i = 0; i < obj.Length - 1; i++)
            //    {
            //        UltrasonicParse(obj[i]);
            //    }

            //    //m_ZLGCAN.CAN_Receive(VehicleReceiveCAN, ref obj);
            //    //for (int i = 0; i < obj.Length-1; i++)
            //    //{
            //    //    VehicleParse(obj[i]);
            //    //}
            //    //m_ZLGCAN.CAN_Receive(VehicleSendCAN, ref obj);
            //    //for (int i = 0; i < obj.Length - 1; i++)
            //    //{
            //    //    VehicleSendParse(obj[i]);
            //    //}
            //}
            //if (0 == tabControl1.SelectedIndex)
            //{
            //    VehicleImformationShow();
            //}
            //else if(1 == tabControl1.SelectedIndex)
            //{
            //    if(checkBox9.Checked)
            //    {
            //        UltrasonicImformationShow();
            //        //UltrasonicImformationFormShow();//原始数据波形显示
            //    }
            //    if(checkBox10.Checked)
            //    {
            //        UltrasonicLocationFormShow();
            //        UltrasonicLocationDirectFormShow();
            //    }
            //    //BoRuiInterfaceCAN1();
            //}
            //else if (2 == tabControl1.SelectedIndex)
            //{
            //    if (checkBox7.Checked)
            //    {
            //        EPB_VehicleSpeedSimulation();
            //        SAS_SteeringAngleSimulation();
            //        TCU_GearSimulation();
            //    }
            //    if (checkBox8.Checked)
            //    {
            //        UltrasonicGroundLocationTriangleFormShow();
            //        UltrasonicGroundLocationDirectFormShow();
            //    }
            //    ParkingControlShow();
            //    TrackInformationShow();
            //    ParkingShow();
            //    TurnningAngleShiftShow();
            //}
            //else if (4 == tabControl1.SelectedIndex)
            //{
            //    UltrasonicLocationShow();
            //}
            
            //if (0xa5 == vehicle_update_status)
            //{
            //    vehicle_update_status = 0;
            //    VehicleShow(ParkingEnterPosition);
            //}
            //if (m_MonitorForm.Visible)
            //{
            //    m_MonitorForm.SteeringAnglePointAdd(m_Vehicle.SteeringAngleTarget, m_Vehicle.SteeringAngleActual);
            //}
            //if (m_Waveform.Visible)
            //{
            //    m_Waveform.VehicleSpeedPointAdd(m_Vehicle.TargetVehicleSpeed, m_Vehicle.VehicleSpeed);
            //}
            //if(m_AccelarateForm.Visible)
            //{
            //    m_AccelarateForm.VehicleAcceleratePointAdd(m_Vehicle.TargetAccelerationACC, m_Vehicle.ActualAccelerationACC, m_Vehicle.LonAcc);
            //}
            ////ParkingDetectionDataLog();
            ////ParkingDetectionDataLogOld();
            ////PlanningInfDataLog();

            //VelocityControlDataLog();
            //LocationMapInfDataLog();

            //UltrasonicPacketDataLog();
            //BodyTriangleLocationDataLog();
        }

        /// <summary>
        /// 超声波数据定时注入
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void InjectionUltrasonicTimer_Tick(object sender, EventArgs e)
        {
            string line;
            if(1 == tabControl1.SelectedIndex)//原始超声波数据的注入
            {
                if ((line = UltrasonicDataLoad.ReadLine() ) != null)
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
            else if(4 == tabControl1.SelectedIndex)//超声定位数据的注入
            {
                if(0xA5 == LocationOpenFileFlag)//文件是否打开状态
                {
                    if(InjectionAckFlag == 0xAA)
                    {
                        if ((line = UltrasonicLocationData.ReadLine()) != null)
                        {
                            // 数据解码
                            string[] s = line.Split(' ');
                            FileDataParse(s, ref m_Ultrasonic_Data_Packet, ref m_GroundDirectLocation, ref m_GroundTriangleLocation);
                            //有效数据显示
                            if (checkBox13.Checked)
                            {
                                if (0 == m_GroundDirectLocation[10].state)
                                {
                                    UltrasonicObstacleLocation_LRU_DataShow.Points.AddXY(m_GroundDirectLocation[10].x, m_GroundDirectLocation[10].y);
                                }
                            }
                            if (0 == m_GroundDirectLocation[11].state)
                            {
                                UltrasonicObstacleLocation_LRU_DataShow.Points.AddXY(m_GroundDirectLocation[11].x, m_GroundDirectLocation[11].y);
                            }
                            if (checkBox12.Checked)
                            {
                                if (0 == m_GroundTriangleLocation[5].state)
                                {
                                    UltrasonicObstacleLocation_SRU_DataShow.Points.AddXY(m_GroundTriangleLocation[5].x, m_GroundTriangleLocation[5].y);
                                }
                                if (0 == m_GroundTriangleLocation[6].state)
                                {
                                    UltrasonicObstacleLocation_SRU_DataShow.Points.AddXY(m_GroundTriangleLocation[6].x, m_GroundTriangleLocation[6].y);
                                }
                            }

                            progressBar2.Value++;
                            //数据发送
                            for (uint i = 8; i < 12; i++)
                            {
                                UltrasonicPacketCAN(i, m_Ultrasonic_Data_Packet[i]);
                            }
                            UltrasonicLocationCAN(12, m_GroundTriangleLocation[5]);
                            UltrasonicLocationCAN(13, m_GroundTriangleLocation[6]);
                            UltrasonicLocationCAN(14, m_GroundDirectLocation[10]);
                            UltrasonicLocationCAN(15, m_GroundDirectLocation[11]);
                            //InjectionAckFlag = 0;
                        }
                        else
                        {
                            UltrasonicLocationData.Close();
                            LocationOpenFileFlag = 0;
                            InjectionUltrasonicTimer.Enabled = false;
                        }
                    }
                    else
                    {
                        //数据发送
                        for (uint i = 8; i < 12; i++)
                        {
                            UltrasonicPacketCAN(i, m_Ultrasonic_Data_Packet[i]);
                        }
                        UltrasonicLocationCAN(12, m_GroundTriangleLocation[5]);
                        UltrasonicLocationCAN(13, m_GroundTriangleLocation[6]);
                        UltrasonicLocationCAN(14, m_GroundDirectLocation[10]);
                        UltrasonicLocationCAN(15, m_GroundDirectLocation[11]);
                    }
                }
            }
        }

        /// <summary>
        /// configure ack single
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer_ack_Tick(object sender, EventArgs e)
        {
            if (AckCnt == 0)
            {
                switch (AckId)
                {
                    case 0x1A:// Vehicle Control
                        //button2.BackColor = Color.Green;
                        break;

                    case 0x2A:// PID Parameter Configure
                        button3.BackColor = Color.Green;
                        break;

                    case 0x2B://PID Parameter Revise
                        button8.BackColor = Color.Green;
                        break;

                    case 0x3A:// Targte Vehicle Configure
                        //button4.BackColor = Color.Green;
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
            else if (AckCnt >= 5)
            {
                switch (AckId)
                {
                    case 0x1A:// Vehicle Control
                        //button2.BackColor = Color.Transparent;
                        break;

                    case 0x2A:// PID Parameter Configure
                        button3.BackColor = Color.Transparent;
                        break;

                    case 0x2B://PID Parameter Revise
                        button8.BackColor = Color.Transparent;
                        break;

                    case 0x3A:// Targte Vehicle Configure
                        //button4.BackColor = Color.Transparent;
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
                button7.Text = "波形图显示开启";
            }
            else
            {
                m_Waveform.Show();
                button7.Text = "波形图显示关闭";
            }
        }

        /// <summary>
        /// 车辆ACC显示
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button8_Click(object sender, EventArgs e)
        {
            if (m_AccelarateForm.Visible)
            {
                m_AccelarateForm.Hide();
                button8.Text = "加速度波形显示开启";
            }
            else
            {
                m_AccelarateForm.Show();
                button8.Text = "加速度波形显示关闭";
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
                u_newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox21.Text + "_Ultrasonic_" + u_fileNameExt;
                ULtrasonicDataSave = new StreamWriter(u_FilePath + "\\" + u_newFileName, true, Encoding.ASCII);
                u_newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox21.Text + "_BodyTriangle_" + u_fileNameExt;
                BodyTriangleDataSave = new StreamWriter(u_FilePath + "\\" + u_newFileName, true, Encoding.ASCII);
                u_DataSaveStatus   = true;
            }
            else
            {
                ULtrasonicDataSave.Close();
                BodyTriangleDataSave.Close();
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
        #endregion

        #region 超声波数据注入
        private void button14_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
            openFileDialog1.InitialDirectory = "D:/APA/NXP_DataSet";
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

        #region 超声波障碍物定位相关事件
        /// <summary>
        /// 选择超声波定位数据的注入文件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button34_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
            openFileDialog1.InitialDirectory = "D:/APA/NXP_DataSet";
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
                UltrasonicLocationLoadFilePath = openFileDialog1.FileName.ToString();
                try
                {
                    using (StreamReader sr = new StreamReader(UltrasonicLocationLoadFilePath, Encoding.ASCII))
                    {
                        UltrasonicLocationFileLineCount = sr.ReadToEnd().Split('\n').Length;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
                try
                {
                    UltrasonicLocationData = new StreamReader(UltrasonicLocationLoadFilePath, Encoding.ASCII);
                    LocationOpenFileFlag = 0xA5;
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
                progressBar2.Maximum = UltrasonicLocationFileLineCount - 1;
                progressBar2.Value = 0;
                UltrasonicObstacleLocation_LRU_DataShow.Points.Clear();
                UltrasonicObstacleLocation_SRU_DataShow.Points.Clear();
                UltrasonicObstacleLocationParkingPointDataShow.Points.Clear();
                LeftFitLine_DataShow.Points.Clear();
                RightFitLine_DataShow.Points.Clear();
                CenterFitLine_DataShow.Points.Clear();


            }
        }



        /// <summary>
        /// 控制命令：开始定位数据的采集
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button39_Click(object sender, EventArgs e)
        {
            UltrasonicLocationCommandCAN(0x50);
        }

        /// <summary>
        /// 清除图像显示信息
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button38_Click(object sender, EventArgs e)
        {
            progressBar2.Value = 0;
            UltrasonicObstacleLocation_LRU_DataShow.Points.Clear();
            UltrasonicObstacleLocation_SRU_DataShow.Points.Clear();
            UltrasonicObstacleLocationParkingPointDataShow.Points.Clear();
            LeftFitLine_DataShow.Points.Clear();
            RightFitLine_DataShow.Points.Clear();
            CenterFitLine_DataShow.Points.Clear();
        }

        /// <summary>
        /// 控制命令：车辆停止命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button40_Click(object sender, EventArgs e)
        {
            UltrasonicLocationCommandCAN(0x60);
        }

        /// <summary>
        /// 清除列表内容
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button43_Click(object sender, EventArgs e)
        {
            LocationlistBox.Items.Clear();
        }

        /// <summary>
        /// 发送入库后数据采集命令，该命令用于库位中心定位的数据采集
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button44_Click(object sender, EventArgs e)
        {
            UltrasonicLocationCommandCAN(0x55);
        }

        /// <summary>
        /// 启动数据保存
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button45_Click(object sender, EventArgs e)
        {
            if (!LocationDataSaveStatus)
            {
                //给文件名前加上时间
                newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox33.Text + "_Location_" + fileNameExt;
                LocationMapDataSave = new StreamWriter(FilePath + "\\" + newFileName, true, Encoding.ASCII);
                newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox33.Text + "_LocationResult_" + fileNameExt;
                LocationResultDataSave = new StreamWriter(FilePath + "\\" + newFileName, true, Encoding.ASCII);
                LocationDataSaveStatus = true;
            }
            else
            {
                LocationMapDataSave.Close();
                LocationResultDataSave.Close();
                LocationDataSaveStatus = false;
            }
            button45.Text       = LocationDataSaveStatus ? "取消保存" : "开始保存";
            button45.BackColor  = LocationDataSaveStatus ? Color.Green : Color.Red;
        }



        /// <summary>
        /// 曲线生成事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4_Click(object sender, EventArgs e)
        {
            TerminalControlCommandCAN(0xD0);
        }

        private void button9_Click(object sender, EventArgs e)
        {
            TerminalWorkModeCommandCAN((byte)comboBox4.SelectedIndex, (byte)comboBox5.SelectedIndex);
        }

        /// <summary>
        /// 保存路径选择
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button37_Click(object sender, EventArgs e)
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

        private void button2_Click(object sender, EventArgs e)
        {
            if(timer_show.Enabled)
            {
                timer_show.Stop();
            }
            else
            {
                timer_show.Start();
            }
        }

        /// <summary>
        /// 控制命令：使能障碍物计算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button42_Click(object sender, EventArgs e)
        {
            UltrasonicLocationCommandCAN(0x70);
        }

        /// <summary>
        /// 障碍物定位计算
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button41_Click(object sender, EventArgs e)
        {
            TerminalControlCommandCAN(0xC0);
        }

        /// <summary>
        /// 注入开始
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button35_Click(object sender, EventArgs e)
        {
            InjectionUltrasonicTimer.Enabled = true;
            InjectionAckFlag = 0xAA;
        }

        /// <summary>
        /// 注入暂停和继续按钮
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button36_Click(object sender, EventArgs e)
        {
            if (InjectionUltrasonicTimer.Enabled)
            {
                InjectionUltrasonicTimer.Enabled = false;
                button36.Text = "继续注入";
            }
            else
            {
                InjectionUltrasonicTimer.Enabled = true;
                button36.Text = "暂停注入";
            }
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
            UltrasonicGroundDirectLocation.Points.Clear();
            UltrasonicGroundTriangleLocation.Points.Clear();
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

        /// <summary>
        /// 转向角显示窗口开启或关闭事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button30_Click(object sender, EventArgs e)
        {
            if (m_MonitorForm.Visible)
            {
                m_MonitorForm.Hide();
                button30.Text = "监控数据开启";
            }
            else
            {
                m_MonitorForm.Show();
                button30.Text = "监控数据关闭";
            }
        }

        /// <summary>
        /// 速度显示开启或关闭
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button31_Click(object sender, EventArgs e)
        {
            if (m_Waveform.Visible)
            {
                m_Waveform.Hide();
                button31.Text = "速度显示开启";
            }
            else
            {
                m_Waveform.Show();
                button31.Text = "速度显示关闭";
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

        #region 参数配置设置
        /// <summary>
        /// 嵌入式全局参数配置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button29_Click(object sender, EventArgs e)
        {
            ParameterConfigureCAN();
        }
        #endregion

        #region 轨迹规划相关事件
        /// <summary>
        /// 轨迹规划相关数据保存
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button32_Click(object sender, EventArgs e)
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
        /// 轨迹规划数据保存按钮
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button33_Click(object sender, EventArgs e)
        {
            if (!PlanningDataSaveStatus)
            {
                //给文件名前加上时间
                newFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + textBox29.Text + "_Planning_" + fileNameExt;
                PlanningDataSave = new StreamWriter(FilePath + "\\" + newFileName, true, Encoding.ASCII);
                PlanningDataSaveStatus = true;
            }
            else
            {
                PlanningDataSave.Close();
                PlanningDataSaveStatus = false;
            }
            button33.Text = PlanningDataSaveStatus ? "取消保存" : "开始保存";
            button33.BackColor = PlanningDataSaveStatus ? Color.Green : Color.Red;
        }
        #endregion

        #region BLDC显示控制
        /// <summary>
        /// VBUS电压
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox15_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox15.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum = 30;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = 0;
            }
            else
            {
                BLDC_VBUS_Show.Points.Clear();
            }
        }

        /// <summary>
        /// 电机位置控制
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox20_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox20.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum = 360;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = 0;
            }
            else
            {
                BLDC_PositionShow.Points.Clear();
            }
        }

        /// <summary>
        /// 角速度
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox21_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox21.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum = 200;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -200;
            }
            else
            {
                BLDC_VelocityShow.Points.Clear();
            }
        }

        /// <summary>
        /// A相电流
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox16_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox16.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum =  2;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -2;
            }
            else
            {
                BLDC_Phase_IA_Show.Points.Clear();
            }
        }

        /// <summary>
        /// B相电流
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox17_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox17.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum =  2;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -2;
            }
            else
            {
                BLDC_Phase_IB_Show.Points.Clear();
            }
        }

        /// <summary>
        /// C相电流
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox18_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox18.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum =  2;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -2;
            }
            else
            {
                BLDC_Phase_IC_Show.Points.Clear();
            }
        }

        /// <summary>
        /// D电流
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox22_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox22.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum = 1;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -1;
            }
            else
            {
                BLDC_Current_D_Show.Points.Clear();
            }
        }

        /// <summary>
        /// Q电流
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox23_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox23.Checked)
            {
                BLDC_chart.ChartAreas[0].AxisY.Maximum =  1;
                BLDC_chart.ChartAreas[0].AxisY.Minimum = -1;
            }
            else
            {
                BLDC_Current_Q_Show.Points.Clear();
            }
        }
        #endregion

        #region BLDC控制
        /// <summary>
        /// 电机自检指令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button48_Click(object sender, EventArgs e)
        {

        }

        /// <summary>
        /// 复位指令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button49_Click(object sender, EventArgs e)
        {
            BLDC_ResetSend();
        }

        /// <summary>
        /// 投放指令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button47_Click(object sender, EventArgs e)
        {
            //BLDC_DeliverySend();
            BLDC_DeliveryMaxSend();
        }

        /// <summary>
        /// 近似零位指令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button50_Click(object sender, EventArgs e)
        {

        }

        /// <summary>
        /// 电机电压控制
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button51_Click(object sender, EventArgs e)
        {
            BLDC_DQ_Send();
        }

        /// <summary>
        /// 速度控制和位置控制
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button52_Click(object sender, EventArgs e)
        {
            BLDC_Speed_Send();
        }
        #endregion
        private void button1_Click(object sender, EventArgs e)
        {
            uint id = 0x7C2;
            byte len = 8;
            byte[] dat = new byte[8];

            //label186.Text = "开始标定";
            dat[0] = 0x02;
            dat[1] = 0x10;
            dat[2] = 0x03;
            dat[3] = 0x00;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x04;
            dat[1] = 0x31;
            dat[2] = 0x01;
            dat[3] = 0xF0;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x04;
            dat[1] = 0x31;
            dat[2] = 0x03;
            dat[3] = 0xF0;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x04;
            dat[1] = 0x31;
            dat[2] = 0x02;
            dat[3] = 0xF0;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x02;
            dat[1] = 0x10;
            dat[2] = 0x01;
            dat[3] = 0x00;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x03;
            dat[1] = 0x19;
            dat[2] = 0x02;
            dat[3] = 0x09;
            dat[4] = 0x00;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            Thread.Sleep(500);

            dat[0] = 0x04;
            dat[1] = 0x14;
            dat[2] = 0xFF;
            dat[3] = 0xFF;
            dat[4] = 0xFF;
            dat[5] = 0x00;
            dat[6] = 0x00;
            dat[7] = 0x00;
            m_ZLGCAN.CAN_Send(VehicleSendCAN, id, len, dat);
            //label186.Text = "标定结束";
        }
        #endregion


    }
}
