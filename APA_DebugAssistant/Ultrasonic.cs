using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace APA_DebugAssistant
{
    public class Ultrasonic
    {
        const double Temperature = 15;
        const double adjacent_tof_threshold = 9860;
        const double tof1_tof2_threshold = 9860;

        private LIN_STP313_ReadData[] last_valid_data;
        private UltrasonicAxis[] last_valid_point;

        /*** LIN Device Data Struct ***/
        public struct LIN_STP318_ReadData
        {
            public UInt16 TOF;
            public byte status;
        }

        public struct LIN_STP313_ReadData
        {
            public UInt16 TOF1;
            public byte Level;
            public byte Width;
            public UInt16 TOF2;
            public byte status;
        }

        public struct LocationPoint
        {
            public double x;
            public double y;
            public double yaw;
        }

        public enum UltrasonicStatus
        {
            Normal,
            BlindZone,
            OverDetection
        }
        public struct UltrasonicAxis
        {
            public double x;
            public double y;
            public UltrasonicStatus state;
        }

        public struct Distance
        {
            public double distance;
            public UltrasonicStatus state;
        }

        public struct ParkingEdgeAxis
        {
            public UltrasonicAxis OutEdge;
            public UltrasonicAxis InnerEdge;
            public Int16 state;
        }

        //public struct ParkingInformation
        //{
        //    public double Length;
        //    public double Width;
        //    public LocationPoint ParkingPoint;
        //}
        //Axis UltrasonicSensing = new Axis();
        enum ParkingStatus
        {
            WaitDownEdge,
            WaitUpEdge,
            ClauclateParkingState
        };
        #region LIN Device Configure relation varibale
        private ParkingStatus m_parking_left_status = new ParkingStatus() { };
        private ParkingStatus m_parking_right_status = new ParkingStatus() { };
        //Lin设备相关参数
        public Int32 DevNum;

        public Int32[] DevHandles = new Int32[20];
        private Byte LINIndex = 0;

        private string[] SensingStatus = new string[5] { "Blockage", "Noise Error", "Hardware Fault", "Communication Error", "Proximity State" };

        private LocationPoint[] UltrasonicVehicleLocation = new LocationPoint[12];
        private double[] BaseAngle = new double[6];
        private double[] BaseLength = new double[6];

        public byte[] LRU_SensingRead_ID = new byte[2] { 0x1f, 0x5E };
        public byte[] SRU_SensingRead_ID = new byte[4] { 0xCf, 0x8E, 0x0D, 0x4C };


        public byte[,] STP318SensingReadNum_SR = new byte[2, 4] {
                                                                { 1,1,1,1 },
                                                                { 1,1,1,1 }
                                                            };

        public byte[,] STP318SensingReadNum_DR = new byte[2, 4] {
                                                                { 3,1,1,3 },
                                                                { 1,3,3,1 }
                                                            };
        //public byte[,,] STP318SensingReadStatus = new byte[4, 4, 2] {
        //                                                                { {0,0},{1,0},{2,0},{7,1} },
        //                                                                { {3,0},{4,1},{5,1},{6,1} },
        //                                                                { {0,0},{5,1},{6,1},{7,1} },
        //                                                                { {1,0},{2,0},{3,0},{4,1} }
        //                                                            };
        public byte[][][][] STP318SensingRead_SR = new byte[4][][][];
        public byte[][][][] STP318SensingRead_DR = new byte[4][][][];

        public byte[,,] SensingSendStatus_DR = new byte[4, 2, 2]{
                                                            { { 0x02, 0x07 },{ 0x08, 0x08 } },//第一次[2->tx ;123->rx][8->tx;8->rx]
                                                            { { 0x08, 0x08 },{ 0x02, 0x07 } },//第二次[4->tx ;4->rx][6->tx;567->rx]
                                                            { { 0x01, 0x01 },{ 0x04, 0x0E } },//第三次[1->tx ;1->rx][7->tx;678->rx]
                                                            { { 0x04, 0x0E },{ 0x01, 0x01 } } //第四次[3->tx ;234->rx][5->tx;5->rx]
                                                            };
        public byte[,,] SensingSendStatus_SR = new byte[4, 2, 2]{
                                                            { { 0x02, 0x02 },{ 0x08, 0x08 } },//第一次[2->tx ;2->rx][8->tx;8->rx]
                                                            { { 0x08, 0x08 },{ 0x02, 0x02 } },//第二次[4->tx ;4->rx][6->tx;6->rx]
                                                            { { 0x01, 0x01 },{ 0x04, 0x04 } },//第三次[1->tx ;1->rx][7->tx;7->rx]
                                                            { { 0x04, 0x04 },{ 0x01, 0x01 } } //第四次[3->tx ;3->rx][5->tx;5->rx]
                                                            };
        #endregion

        #region Unity function
        public double CalculateVehicleUltrasonicDistance(LocationPoint vehicle, UltrasonicAxis ultrasonic)
        {
            return Math.Sqrt(Math.Pow(vehicle.x - ultrasonic.x, 2) + Math.Pow(vehicle.y - ultrasonic.y, 2));
        }

        public double CalculateUltrasonicDistance(UltrasonicAxis ultrasonic1, UltrasonicAxis ultrasonic2)
        {
            return Math.Sqrt(Math.Pow(ultrasonic1.x - ultrasonic2.x, 2) + Math.Pow(ultrasonic1.y - ultrasonic2.y, 2));
        }

        public double CalculateSlope(UltrasonicAxis ultrasonic1, UltrasonicAxis ultrasonic2)
        {
            return Math.Atan2(ultrasonic2.y - ultrasonic1.y , ultrasonic2.x - ultrasonic1.x);
        }
        
        public bool CheckVectorAngleSymbol(LocationPoint vehicle1, LocationPoint vehicle2, UltrasonicAxis ultrasonic)
        {
            LocationPoint vector1, vector2;
            vector1.x = ultrasonic.x - vehicle1.x;
            vector1.y = ultrasonic.y - vehicle1.y;
            vector2.x = vehicle2.x - vehicle1.x;
            vector2.y = vehicle2.y - vehicle1.y;
            double vector_s1 = vector1.x * vector2.x + vector1.y * vector2.y;

            vector1.x = ultrasonic.x - vehicle2.x;
            vector1.y = ultrasonic.y - vehicle2.y;
            vector2.x = vehicle1.x - vehicle2.x;
            vector2.y = vehicle1.y - vehicle2.y;
            double vector_s2 = vector1.x * vector2.x + vector1.y * vector2.y;

            return (vector_s1 >= 0 && vector_s2 >= 0) ? true : false;
        }
        #endregion

        public Ultrasonic()
        {
            LINIndex = 0;
            last_valid_data = new LIN_STP313_ReadData[4];
            last_valid_point = new UltrasonicAxis[4];
            //last_valid_tof = new double[4] { adjacent_tof_threshold, adjacent_tof_threshold, adjacent_tof_threshold, adjacent_tof_threshold };
            ///单发单收
            //第一次
            STP318SensingRead_SR[0] = new byte[2][][];
            STP318SensingRead_SR[0][0] = new byte[2][];//device 0
            STP318SensingRead_SR[0][0][0] = new byte[1] { 1 };//sensing array
            STP318SensingRead_SR[0][0][1] = new byte[1] { 1 };//ID
            STP318SensingRead_SR[0][1] = new byte[2][];//device 1
            STP318SensingRead_SR[0][1][0] = new byte[1] { 7 };
            STP318SensingRead_SR[0][1][1] = new byte[1] { 3 };
            //第二次
            STP318SensingRead_SR[1] = new byte[2][][];
            STP318SensingRead_SR[1][0] = new byte[2][];//device 0
            STP318SensingRead_SR[1][0][0] = new byte[1] { 3 };//sensing array
            STP318SensingRead_SR[1][0][1] = new byte[1] { 3 };//ID
            STP318SensingRead_SR[1][1] = new byte[2][];//device 1
            STP318SensingRead_SR[1][1][0] = new byte[1] { 5 };//sensing array
            STP318SensingRead_SR[1][1][1] = new byte[1] { 1 };//ID
            //第三次
            STP318SensingRead_SR[2] = new byte[2][][];
            STP318SensingRead_SR[2][0] = new byte[2][];//device 0
            STP318SensingRead_SR[2][0][0] = new byte[1] { 0 };//sensing array
            STP318SensingRead_SR[2][0][1] = new byte[1] { 0 };//ID
            STP318SensingRead_SR[2][1] = new byte[2][];//device 1
            STP318SensingRead_SR[2][1][0] = new byte[1] { 6 };//sensing array
            STP318SensingRead_SR[2][1][1] = new byte[1] { 2 };//ID
            //第四次
            STP318SensingRead_SR[3] = new byte[2][][];
            STP318SensingRead_SR[3][0] = new byte[2][];//device 0
            STP318SensingRead_SR[3][0][0] = new byte[1] { 2 };//sensing array
            STP318SensingRead_SR[3][0][1] = new byte[1] { 2 };//ID
            STP318SensingRead_SR[3][1] = new byte[2][];//device 1
            STP318SensingRead_SR[3][1][0] = new byte[1] { 4 };//sensing array
            STP318SensingRead_SR[3][1][1] = new byte[1] { 0 };//ID

            ///单发多收
            //第一次
            STP318SensingRead_DR[0] = new byte[2][][];
            STP318SensingRead_DR[0][0] = new byte[2][];//device 0
            STP318SensingRead_DR[0][0][0] = new byte[1] { 1 };//sensing array
            STP318SensingRead_DR[0][0][1] = new byte[3] { 0, 1, 2 };//ID
            STP318SensingRead_DR[0][1] = new byte[2][];//device 1
            STP318SensingRead_DR[0][1][0] = new byte[1] { 7 };
            STP318SensingRead_DR[0][1][1] = new byte[1] { 3 };
            //第二次
            STP318SensingRead_DR[1] = new byte[2][][];
            STP318SensingRead_DR[1][0] = new byte[2][];//device 0
            STP318SensingRead_DR[1][0][0] = new byte[1] { 3 };//sensing array
            STP318SensingRead_DR[1][0][1] = new byte[1] { 3 };//ID
            STP318SensingRead_DR[1][1] = new byte[2][];//device 1
            STP318SensingRead_DR[1][1][0] = new byte[1] { 5 };//sensing array
            STP318SensingRead_DR[1][1][1] = new byte[3] { 0, 1, 2 };//ID
            //第三次
            STP318SensingRead_DR[2] = new byte[2][][];
            STP318SensingRead_DR[2][0] = new byte[2][];//device 0
            STP318SensingRead_DR[2][0][0] = new byte[1] { 0 };//sensing array
            STP318SensingRead_DR[2][0][1] = new byte[1] { 0 };//ID
            STP318SensingRead_DR[2][1] = new byte[2][];//device 1
            STP318SensingRead_DR[2][1][0] = new byte[1] { 6 };//sensing array
            STP318SensingRead_DR[2][1][1] = new byte[3] { 1, 2, 3 };//ID
            //第四次
            STP318SensingRead_DR[3] = new byte[2][][];
            STP318SensingRead_DR[3][0] = new byte[2][];//device 0
            STP318SensingRead_DR[3][0][0] = new byte[1] { 2 };//sensing array
            STP318SensingRead_DR[3][0][1] = new byte[3] { 1, 2, 3 };//ID
            STP318SensingRead_DR[3][1] = new byte[2][];//device 1
            STP318SensingRead_DR[3][1][0] = new byte[1] { 4 };//sensing array
            STP318SensingRead_DR[3][1][1] = new byte[1] { 0 };//ID

            UltrasonicVehicleLocation[0].x = 342;
            UltrasonicVehicleLocation[0].y = 70;
            UltrasonicVehicleLocation[0].yaw = 15 * Math.PI / 180.0;

            UltrasonicVehicleLocation[1].x = 360;
            UltrasonicVehicleLocation[1].y = 30;
            UltrasonicVehicleLocation[1].yaw = 5 * Math.PI / 180.0;

            UltrasonicVehicleLocation[2].x = 360;
            UltrasonicVehicleLocation[2].y = -30;
            UltrasonicVehicleLocation[2].yaw = 355 * Math.PI / 180.0;

            UltrasonicVehicleLocation[3].x = 342;
            UltrasonicVehicleLocation[3].y = -70;
            UltrasonicVehicleLocation[3].yaw = 345 * Math.PI / 180.0;

            UltrasonicVehicleLocation[4].x = -87;
            UltrasonicVehicleLocation[4].y = 72;
            UltrasonicVehicleLocation[4].yaw = 150 * Math.PI / 180.0;

            UltrasonicVehicleLocation[5].x = -100;
            UltrasonicVehicleLocation[5].y = 30;
            UltrasonicVehicleLocation[5].yaw = 175 * Math.PI / 180.0;

            UltrasonicVehicleLocation[6].x = -100;
            UltrasonicVehicleLocation[6].y = -30;
            UltrasonicVehicleLocation[6].yaw = 185 * Math.PI / 180.0;

            UltrasonicVehicleLocation[7].x = -87;
            UltrasonicVehicleLocation[7].y = -72;
            UltrasonicVehicleLocation[7].yaw = 210 * Math.PI / 180.0;

            UltrasonicVehicleLocation[8].x = 295;
            UltrasonicVehicleLocation[8].y = 90;
            UltrasonicVehicleLocation[8].yaw = 90 * Math.PI / 180.0;

            UltrasonicVehicleLocation[9].x = 295;
            UltrasonicVehicleLocation[9].y = -90;
            UltrasonicVehicleLocation[9].yaw = 270 * Math.PI / 180.0;

            UltrasonicVehicleLocation[10].x = -30;
            UltrasonicVehicleLocation[10].y = 90;
            UltrasonicVehicleLocation[10].yaw = 90 * Math.PI / 180.0;

            UltrasonicVehicleLocation[11].x = -30;
            UltrasonicVehicleLocation[11].y = -90;
            UltrasonicVehicleLocation[11].yaw = 270 * Math.PI / 180.0;

            BaseAngle[0] = Math.Atan2(UltrasonicVehicleLocation[1].y - UltrasonicVehicleLocation[0].y, UltrasonicVehicleLocation[1].x - UltrasonicVehicleLocation[0].x);
            BaseAngle[1] = Math.Atan2(UltrasonicVehicleLocation[2].y - UltrasonicVehicleLocation[1].y, UltrasonicVehicleLocation[2].x - UltrasonicVehicleLocation[1].x);
            BaseAngle[2] = Math.Atan2(UltrasonicVehicleLocation[3].y - UltrasonicVehicleLocation[2].y, UltrasonicVehicleLocation[3].x - UltrasonicVehicleLocation[2].x);

            BaseAngle[3] = Math.Atan2(UltrasonicVehicleLocation[5].y - UltrasonicVehicleLocation[4].y, UltrasonicVehicleLocation[5].x - UltrasonicVehicleLocation[4].x);
            BaseAngle[4] = Math.Atan2(UltrasonicVehicleLocation[6].y - UltrasonicVehicleLocation[5].y, UltrasonicVehicleLocation[6].x - UltrasonicVehicleLocation[5].x);
            BaseAngle[5] = Math.Atan2(UltrasonicVehicleLocation[7].y - UltrasonicVehicleLocation[6].y, UltrasonicVehicleLocation[7].x - UltrasonicVehicleLocation[6].x);

            BaseLength[0] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[1].x - UltrasonicVehicleLocation[0].x, 2) 
                                    + Math.Pow(UltrasonicVehicleLocation[1].y - UltrasonicVehicleLocation[0].y, 2));
            BaseLength[1] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[2].x - UltrasonicVehicleLocation[1].x, 2)
                                    + Math.Pow(UltrasonicVehicleLocation[2].y - UltrasonicVehicleLocation[1].y, 2));
            BaseLength[2] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[3].x - UltrasonicVehicleLocation[2].x, 2)
                                    + Math.Pow(UltrasonicVehicleLocation[3].y - UltrasonicVehicleLocation[2].y, 2));

            BaseLength[3] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[5].x - UltrasonicVehicleLocation[4].x, 2)
                                    + Math.Pow(UltrasonicVehicleLocation[5].y - UltrasonicVehicleLocation[4].y, 2));
            BaseLength[4] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[6].x - UltrasonicVehicleLocation[5].x, 2)
                                    + Math.Pow(UltrasonicVehicleLocation[6].y - UltrasonicVehicleLocation[5].y, 2));
            BaseLength[5] = Math.Sqrt(Math.Pow(UltrasonicVehicleLocation[7].x - UltrasonicVehicleLocation[6].x, 2)
                                    + Math.Pow(UltrasonicVehicleLocation[7].y - UltrasonicVehicleLocation[6].y, 2));
        }

        /// <summary>
        /// 将STP318传感器的数据映射进指定控件中
        /// </summary>
        /// <param name="dat">input the parameter</param>
        /// <param name="tx"> ref textbox </param>
        /// <param name="lb"> ref label</param>
        /// 
        public void DataMapping2Control_STP318(LIN_STP318_ReadData dat, ref Label[] tx)
        {
            tx[0].Text = (dat.TOF / 58.0).ToString();
            if (dat.status == 1)
            {
                tx[1].Text = SensingStatus[0];
            }
            else if (dat.status == 2)
            {
                tx[1].Text = SensingStatus[1];
            }
            else if (dat.status == 4)
            {
                tx[1].Text = SensingStatus[2];
            }
            else if (dat.status == 8)
            {
                tx[1].Text = SensingStatus[3];
            }
            else if (dat.status == 16)
            {
                tx[1].Text = SensingStatus[4];
            }
            else if (dat.status == 0)
            {
                tx[1].Text = "正常";
            }
            else
            {
                tx[1].Text = "异常";
            }
        }
        /// <summary>
        /// 将STP313传感器的数据映射到指定的控件中
        /// </summary>
        /// <param name="dat"></param>
        /// <param name="tx"></param>
        /// <param name="lb"></param>
        public void DataMapping2Control_STP313(LIN_STP313_ReadData dat, ref Label[] tx)
        {
            tx[0].Text = ((dat.TOF1) / 58.0).ToString();
            tx[1].Text = ((dat.TOF2) / 58.0).ToString();
            tx[2].Text = (dat.Width * 16).ToString();
            tx[3].Text = (dat.Level * 3.3 / 255).ToString();

            if (dat.status == 1)
            {
                tx[4].Text = SensingStatus[0];
            }
            else if (dat.status == 2)
            {
                tx[4].Text = SensingStatus[1];
            }
            else if (dat.status == 4)
            {
                tx[4].Text = SensingStatus[2];
            }
            else if (dat.status == 8)
            {
                tx[4].Text = SensingStatus[3];
            }
            else if (dat.status == 16)
            {
                tx[4].Text = SensingStatus[4];
            }
            else
            {
                tx[4].Text = "正常";
            }
        }

 
       

        //温度补偿
        public double TemperatureCompensation(double temp)
        {
            return 0.01724137931034482758620689655172;
        }

        public double TemperatureCompensationSoundSpeed(double temp)
        {
            return 331.5 + 0.60714 * temp;
        }

        /// <summary>
        /// 长距离传感器的数据处理
        /// </summary>
        /// <param name="m_313Data">传感器原始数据的输入</param>
        /// <param name="InstallationPosition">传感器的安装位置信息</param>
        /// <param name="ultrasonic_location">输出的传感器的检测目标位置</param>
        private void STP313_Process(LIN_STP313_ReadData m_313Data, LocationPoint InstallationPosition, ref UltrasonicAxis ultrasonic_location)
        {
            if (m_313Data.status == 0)
            {
                if (m_313Data.TOF1 != 0)
                {
                    double distance = m_313Data.TOF1 * TemperatureCompensation(25);//cm
                    ultrasonic_location.x = InstallationPosition.x + distance * Math.Cos(InstallationPosition.yaw);
                    ultrasonic_location.y = InstallationPosition.y + distance * Math.Sin(InstallationPosition.yaw);
                    ultrasonic_location.state = UltrasonicStatus.Normal;
                }
                else
                {
                    ultrasonic_location.x = 0;
                    ultrasonic_location.y = 0;
                    ultrasonic_location.state = UltrasonicStatus.OverDetection;
                }
            }
            else if(m_313Data.status == 16)
            {
                ultrasonic_location.x = 0;
                ultrasonic_location.y = 0;
                ultrasonic_location.state = UltrasonicStatus.BlindZone;
            }
            ultrasonic_location.x *= 0.01;
            ultrasonic_location.y *= 0.01;
        }

        private void STP318_Process(LIN_STP318_ReadData m_318Data, LocationPoint InstallationPosition, ref UltrasonicAxis ultrasonic_location)
        {
            if (m_318Data.status == 0)
            {
                if (m_318Data.TOF != 0)
                {
                    double distance = m_318Data.TOF * TemperatureCompensation(25);//cm
                    ultrasonic_location.x = InstallationPosition.x + distance * Math.Cos(InstallationPosition.yaw);
                    ultrasonic_location.y = InstallationPosition.y + distance * Math.Sin(InstallationPosition.yaw);
                    ultrasonic_location.state = UltrasonicStatus.Normal;
                }
                else
                {
                    ultrasonic_location.x = 0;
                    ultrasonic_location.y = 0;
                    ultrasonic_location.state = UltrasonicStatus.OverDetection;
                }
            }
            else if(m_318Data.status == 16)
            {
                ultrasonic_location.x = 0;
                ultrasonic_location.y = 0;
                ultrasonic_location.state = UltrasonicStatus.BlindZone;
            }
            ultrasonic_location.x *= 0.01;
            ultrasonic_location.y *= 0.01;
        }

        /// <summary>
        /// 318传感器计算超声波的距离
        /// </summary>
        /// <param name="m_318Data">输入原始超声波的原始数据</param>
        /// <param name="Distance">计算实际距离</param>
        private void DistanceCalculateArray(LIN_STP318_ReadData[] m_318Data,ref Distance[] distance)
        {
            for(int i=0;i< m_318Data.Length; i++)
            {
                if(m_318Data[i].status == 0)
                {
                    if(m_318Data[i].TOF != 0)
                    {
                        distance[i].distance = m_318Data[i].TOF * TemperatureCompensation(25);//cm
                        distance[i].state = UltrasonicStatus.Normal;
                    }
                    else
                    {
                        distance[i].distance = 0;//cm
                        distance[i].state = UltrasonicStatus.OverDetection;
                    }
                }
                else
                {
                    distance[i].distance = 0;//cm
                    distance[i].state = UltrasonicStatus.BlindZone;
                }
            }         
        }

        /// <summary>
        /// 318 distance calculate accurate
        /// </summary>
        /// <param name="m_318Data"></param>
        /// <param name="distance"></param>
        private void DistanceAccurateCalculate(LIN_STP318_ReadData[] m_318Data, ref Distance[] distance)
        {
            double [] t = new double [3];
            if(m_318Data[1].status == 0)//正常
            {
                if(m_318Data[1].TOF != 0)
                {
                    t[1] = m_318Data[1].TOF * 0.00005;
                    t[0] = m_318Data[0].TOF * 0.0001 - t[1];
                    t[2] = m_318Data[2].TOF * 0.0001 - t[1];
                    for(int i=0;i<3;i++)
                    {
                        distance[i].distance = t[i] * TemperatureCompensationSoundSpeed(Temperature);
                        distance[i].state = UltrasonicStatus.Normal;
                    }
                }
                else
                {
                    for (int i = 0; i < 3; i++)
                    {
                        distance[i].distance = 0;
                        distance[i].state = UltrasonicStatus.OverDetection;
                    }
                }
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    distance[i].distance = 0;
                    distance[i].state = UltrasonicStatus.BlindZone;
                }
            }
        }
        /// <summary>
        /// 目标位置坐标点计算（三角定位）
        /// </summary>
        /// <param name="base_position">基准点坐标点</param>
        /// <param name="base_angle">基准角度(弧度)</param>
        /// <param name="base_L">基准长度(cm)</param>
        /// <param name="distance1">临边长度(cm)</param>
        /// <param name="distance2">对角长度(cm)</param>
        /// <param name="TargetPosition">输出最终的目标位置坐标</param>
        private void TargetPositionCalculate(LocationPoint base_position,double base_angle,double base_L, Distance distance1, Distance distance2,ref UltrasonicAxis TargetPosition,bool check_direction)
        {
            if(distance1.state == UltrasonicStatus.BlindZone ||
                distance2.state == UltrasonicStatus.BlindZone)
            {
                TargetPosition.x = 0;
                TargetPosition.y = 0;
                TargetPosition.state = UltrasonicStatus.BlindZone;
            }
            else
            {
                if (distance1.state == UltrasonicStatus.Normal && distance2.state == UltrasonicStatus.Normal)
                {
                    double beta = Math.Acos((Math.Pow(base_L, 2) + Math.Pow(distance1.distance, 2) - Math.Pow(distance2.distance, 2)) / (2 * base_L * distance1.distance));
                    double gama = check_direction ? (base_angle + beta) : (base_angle - beta);
                    TargetPosition.x = base_position.x + distance1.distance * Math.Cos(gama);
                    TargetPosition.y = base_position.y + distance1.distance * Math.Sin(gama);
                    TargetPosition.state = UltrasonicStatus.Normal;
                }
                else
                {
                    TargetPosition.x = 0;
                    TargetPosition.y = 0;
                    TargetPosition.state = UltrasonicStatus.OverDetection;
                }
            }
            TargetPosition.x *= 0.01;
            TargetPosition.y *= 0.01;
        }

        /// <summary>
        /// single send single receive modle
        /// </summary>
        /// <param name="m_318Data"></param>
        /// <param name="m_313Data"></param>
        /// <param name="ultrasonic_location"></param>
        public void UltrasonicDataVehicleLocationSingleRT(LIN_STP318_ReadData[] m_318Data, LIN_STP313_ReadData[] m_313Data, ref UltrasonicAxis[] ultrasonic_location)
        {
            for(int i=0;i< m_318Data.Length; i++)
            {
                STP318_Process(m_318Data[i], UltrasonicVehicleLocation[i], ref ultrasonic_location[i]);
            }
            for(int i=0;i< m_313Data.Length;i++)
            {
                STP313_Process(m_313Data[i], UltrasonicVehicleLocation[8+i], ref ultrasonic_location[8+i]);
            }
        }
        /// <summary>
        /// 根据超声波原始数据(一发多收模式)计算目标位置信息
        /// </summary>
        /// <param name="m_318Data">短距离超声波传感器原始数据</param>
        /// <param name="m_313Data">长距离超声波传感器原始数据</param>
        /// <param name="ultrasonic_location">目标位置定位</param>
        public void UltrasonicDataVehicleLocation(LIN_STP318_ReadData[][] m_318Data,LIN_STP313_ReadData[] m_313Data,ref UltrasonicAxis[] ultrasonic_location, ref UltrasonicAxis[] Ovoidance_location)
        {
            Distance[] distance_temp = new Distance[3];
            //2号
            //DistanceCalculateArray(m_318Data[1], ref distance_temp);
            DistanceAccurateCalculate(m_318Data[1], ref distance_temp);
            TargetPositionCalculate(UltrasonicVehicleLocation[0], BaseAngle[0], BaseLength[0], distance_temp[0], distance_temp[1], ref ultrasonic_location[0], true);
            TargetPositionCalculate(UltrasonicVehicleLocation[1], BaseAngle[1], BaseLength[1], distance_temp[1], distance_temp[2], ref ultrasonic_location[1], true);

            //3号
            //DistanceCalculateArray(m_318Data[2], ref distance_temp);
            DistanceAccurateCalculate(m_318Data[2], ref distance_temp);
            TargetPositionCalculate(UltrasonicVehicleLocation[1], BaseAngle[1], BaseLength[1], distance_temp[0], distance_temp[1], ref ultrasonic_location[2], true);
            TargetPositionCalculate(UltrasonicVehicleLocation[2], BaseAngle[2], BaseLength[2], distance_temp[1], distance_temp[2], ref ultrasonic_location[3], true);

            //6号
            //DistanceCalculateArray(m_318Data[5], ref distance_temp);
            DistanceAccurateCalculate(m_318Data[5], ref distance_temp);
            TargetPositionCalculate(UltrasonicVehicleLocation[4], BaseAngle[3], BaseLength[3], distance_temp[0], distance_temp[1], ref ultrasonic_location[4], false);
            TargetPositionCalculate(UltrasonicVehicleLocation[5], BaseAngle[4], BaseLength[4], distance_temp[1], distance_temp[2], ref ultrasonic_location[5], false);

            //7号
            //DistanceCalculateArray(m_318Data[6], ref distance_temp);
            DistanceAccurateCalculate(m_318Data[6], ref distance_temp);
            TargetPositionCalculate(UltrasonicVehicleLocation[5], BaseAngle[4], BaseLength[4], distance_temp[0], distance_temp[1], ref ultrasonic_location[6], false);
            TargetPositionCalculate(UltrasonicVehicleLocation[6], BaseAngle[5], BaseLength[5], distance_temp[1], distance_temp[2], ref ultrasonic_location[7], false);

            //9号
            STP313_Process(m_313Data[0], UltrasonicVehicleLocation[8], ref ultrasonic_location[8]);
            //10号
            STP313_Process(m_313Data[1], UltrasonicVehicleLocation[9], ref ultrasonic_location[9]);
            //11号
            STP313_Process(m_313Data[2], UltrasonicVehicleLocation[10], ref ultrasonic_location[10]);
            //12号
            STP313_Process(m_313Data[3], UltrasonicVehicleLocation[11], ref ultrasonic_location[11]);

            // calculate the obvoidance distance
            STP318_Process(m_318Data[0][0], UltrasonicVehicleLocation[0], ref Ovoidance_location[0]);
            STP318_Process(m_318Data[3][0], UltrasonicVehicleLocation[3], ref Ovoidance_location[1]);
            //Console.WriteLine("4:{0}", m_318Data[3][0].TOF);
            STP318_Process(m_318Data[4][0], UltrasonicVehicleLocation[4], ref Ovoidance_location[2]);
            STP318_Process(m_318Data[7][0], UltrasonicVehicleLocation[7], ref Ovoidance_location[3]);
        }

        /// <summary>
        /// the rotation of the vehicle coordinate system to the ground coordinate system with arrary data
        /// </summary>
        /// <param name="vehicle_center_point"> the vehicle rotation center </param>
        /// <param name="ultrasonic_vehicle_location"> the ultrasonic point in the vehicle coordinate </param>
        /// <param name="ultrasonic_ground_location"> the ultrasonic point in the ground coordinate </param>
        public void UltrasonicDataGroundLocationArray(LocationPoint vehicle_center_point, UltrasonicAxis[] ultrasonic_vehicle_location,ref UltrasonicAxis[] ultrasonic_ground_location)
        {
            for(int i=0;i< ultrasonic_vehicle_location.Length;i++)
            {
                ultrasonic_ground_location[i].x = vehicle_center_point.x 
                                                + ultrasonic_vehicle_location[i].x * Math.Cos(vehicle_center_point.yaw) 
                                                - ultrasonic_vehicle_location[i].y * Math.Sin(vehicle_center_point.yaw);
                ultrasonic_ground_location[i].y = vehicle_center_point.y 
                                                + ultrasonic_vehicle_location[i].x * Math.Sin(vehicle_center_point.yaw) 
                                                + ultrasonic_vehicle_location[i].y * Math.Cos(vehicle_center_point.yaw);
                ultrasonic_ground_location[i].state = ultrasonic_vehicle_location[i].state;
            }
        }

        /// <summary>
        /// the vehicle coordinate translate to the ground coordinate  with one data
        /// </summary>
        /// <param name="vehicle_center_point"></param>
        /// <param name="ultrasonic_vehicle_location"></param>
        /// <param name="ultrasonic_ground_location"></param>
        public void UltrasonicDataGroundLocationSingle(LocationPoint vehicle_center_point, UltrasonicAxis ultrasonic_vehicle_location, ref UltrasonicAxis ultrasonic_ground_location)
        {
                ultrasonic_ground_location.x = vehicle_center_point.x 
                                             + ultrasonic_vehicle_location.x * Math.Cos(vehicle_center_point.yaw)
                                             - ultrasonic_vehicle_location.y * Math.Sin(vehicle_center_point.yaw);
                ultrasonic_ground_location.y = vehicle_center_point.y 
                                             + ultrasonic_vehicle_location.x * Math.Sin(vehicle_center_point.yaw) 
                                             + ultrasonic_vehicle_location.y * Math.Cos(vehicle_center_point.yaw);
                ultrasonic_ground_location.state = ultrasonic_vehicle_location.state;
        }

        /// <summary>
        /// base on the front ad rear 318 sensor calculate the vehicle yaw
        /// </summary>
        /// <param name="ultrasonic_location"> the ultrasonic sensor date in vehicle coordinate </param>
        /// <param name="angle"> the output of the vehicle yaw </param>
        public void VehicleAngleCalculate(UltrasonicAxis[] ultrasonic_location,ref double [] angle)
        {
            angle[0] = Math.Atan2(ultrasonic_location[1].x - ultrasonic_location[2].x, ultrasonic_location[1].y - ultrasonic_location[2].y);
            angle[1] = Math.Atan2(ultrasonic_location[5].x - ultrasonic_location[6].x, ultrasonic_location[5].y - ultrasonic_location[6].y);
        }

        /// <summary>
        /// Obstacle Detection with single send and single receive
        /// </summary>
        /// <param name="m_318Data"> the 318 Raw data </param>
        /// <param name="m_313Data"> the 313 Raw data </param>
        /// <returns></returns>
        public bool DistanceCloseTOAlarm_SR(LIN_STP318_ReadData [] m_318Data, LIN_STP313_ReadData[] m_313Data)
        {
            for(int i=0;i<8;i++)
            {
                if (m_318Data[i].status == 16)
                {
                    return true;
                }
            }
            for (int i = 0; i < 4; i++)
            {
                if (m_313Data[i].status == 16)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Obstacle Detection with single send and many receive
        /// </summary>
        /// <param name="m_318Data"> the raw data of the 318 sensor </param>
        /// <param name="m_313Data"> the raw data of the 313 sensor </param>
        /// <returns></returns>
        public bool DistanceCloseTOAlarm_DR(LIN_STP318_ReadData[][] m_318Data, LIN_STP313_ReadData[] m_313Data)
        {
            for(int step=0; step<4; step++)
            {
                for (int i = 0; i < DevNum; i++)
                {
                    for (int k = 0; k < STP318SensingReadNum_DR[i, step]; k++)
                    {
                        if (m_318Data[STP318SensingRead_DR[step][i][0][0]][k].status == 16)
                        {
                            return true;
                        }
                    }
                }
            }

            for (int i = 0; i < 4; i++)
            {
                if (m_313Data[i].status == 16)
                {
                    return true;
                }
            }
            return false;
        }

        

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public bool CheckObstacelEdgeJudge()
        {
            return true;
        }

        /// <summary>
        /// judge the obstacle wheter need emergency stop
        /// </summary>
        /// <param name="m_vehicle"></param>
        /// <param name="ultrasonic_vehicle_location"></param>
        /// <returns></returns>
        public bool EmergencyStop(Vehicle m_vehicle, UltrasonicAxis[] ultrasonic_vehicle_location)
        {
            double min_distance,u_v_vehicle;
            min_distance = Math.Pow(m_vehicle.Speed_m, 2) * 0.5 / m_vehicle.SpeedAcc + 0.1;
            if (m_vehicle.SpeedDirection)
            {
                for(int i =0;i<4;i++)
                {
                    if(ultrasonic_vehicle_location[i].state == UltrasonicStatus.Normal)
                    {
                        u_v_vehicle = Math.Abs( ultrasonic_vehicle_location[i].x) - (m_vehicle.WheelBaseLenght + m_vehicle.FrontOverhangDistance);
                        if(min_distance >= u_v_vehicle)
                        {
                            Console.WriteLine("距离触发：{0}",u_v_vehicle);
                            return true;
                        }
                    }
                    else if(ultrasonic_vehicle_location[i].state == UltrasonicStatus.BlindZone)
                    {
                        Console.WriteLine("盲区触发");
                        return true;
                    }
                }
            }
            else
            {
                for (int i = 4; i < 8; i++)
                {
                    if(ultrasonic_vehicle_location[i].state == UltrasonicStatus.Normal)
                    {
                        u_v_vehicle = Math.Abs(ultrasonic_vehicle_location[i].x) - m_vehicle.RearOverhangDistance;
                        if (min_distance >= u_v_vehicle)
                        {
                            Console.WriteLine("距离触发：{0}", u_v_vehicle);
                            return true;
                        }
                    }
                    else if(ultrasonic_vehicle_location[i].state == UltrasonicStatus.BlindZone)
                    {
                        Console.WriteLine("盲区触发");
                        return true;
                    }
                }
            }
            return false;
        }
        
        /// <summary>
        /// base on the edge ultrasonic data to judge the whether closing
        /// </summary>
        /// <param name="m_vehicle"></param>
        /// <param name="ultrasonic_vehicle_location"></param>
        /// <returns></returns>
        public bool EmergencyStopFourEdgeSensor(Vehicle m_vehicle, UltrasonicAxis[] ultrasonic_vehicle_location)
        {
            double min_distance, u_v_vehicle;
            min_distance = Math.Pow(m_vehicle.Speed_m, 2) * 0.5 / m_vehicle.SpeedAcc + 0.1;
            if (m_vehicle.SpeedDirection)
            {
                for (int i = 0; i < 2; i++)
                {
                    if (ultrasonic_vehicle_location[i].state == UltrasonicStatus.Normal)
                    {
                        u_v_vehicle = Math.Abs(Math.Abs(ultrasonic_vehicle_location[i].x) - (m_vehicle.WheelBaseLenght + m_vehicle.FrontOverhangDistance));
                        if (min_distance >= u_v_vehicle)
                        {
                            Console.WriteLine("距离触发：{0}", u_v_vehicle);
                            return true;
                        }
                    }
                    else if (ultrasonic_vehicle_location[i].state == UltrasonicStatus.BlindZone)
                    {
                        Console.WriteLine("盲区触发");
                        return true;
                    }
                }
            }
            else
            {
                for (int i = 2; i < 4; i++)
                {
                    if (ultrasonic_vehicle_location[i].state == UltrasonicStatus.Normal)
                    {
                        u_v_vehicle = Math.Abs(Math.Abs(ultrasonic_vehicle_location[i].x) - m_vehicle.RearOverhangDistance);
                        if (min_distance >= u_v_vehicle)
                        {
                            Console.WriteLine("距离触发：{0}", u_v_vehicle);
                            return true;
                        }
                    }
                    else if (ultrasonic_vehicle_location[i].state == UltrasonicStatus.BlindZone)
                    {
                        Console.WriteLine("盲区触发");
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// Detect the Parking Space Edge
        /// </summary>
        /// <param name="m_313Data"></param>
        /// <param name="vehicle_center"></param>
        /// <param name="detection_state"></param>
        /// <param name="OutEdge"></param>
        /// <param name="InnerEdge"></param>
        public void ParkingEdgeDetection(LIN_STP313_ReadData[] m_313Data, LocationPoint vehicle_center,ref ParkingEdgeAxis [] parking_pack)
        {
            UltrasonicAxis [] current_data_point_vehicle = new UltrasonicAxis[2];
            UltrasonicAxis[] current_data_point_ground = new UltrasonicAxis[2];
            for (int i = 0;i < 2; i++)
            {
                if(m_313Data[i].TOF1 != 0)
                {
                    STP313_Process(m_313Data[i], UltrasonicVehicleLocation[8 + i], ref current_data_point_vehicle[i]);
                    UltrasonicDataGroundLocationSingle(vehicle_center, current_data_point_vehicle[i], ref current_data_point_ground[i]);
                    if(last_valid_data[i].TOF1 != 0)
                    {
                        if ((m_313Data[i].TOF1 - last_valid_data[i].TOF1) > adjacent_tof_threshold)//up
                        {
                            parking_pack[i].state = -1;
                            parking_pack[i].OutEdge = current_data_point_ground[i];
                            parking_pack[i].InnerEdge = last_valid_point[i];
                        }
                        else if ((m_313Data[i].TOF1 - last_valid_data[i].TOF1) < -adjacent_tof_threshold)//down
                        {
                            parking_pack[i].state = 1;
                            parking_pack[i].OutEdge = last_valid_point[i];
                            parking_pack[i].InnerEdge = current_data_point_ground[i];
                        }
                        else
                        {
                            parking_pack[i].state = 0;
                            parking_pack[i].OutEdge = current_data_point_ground[i];
                        }
                    }
                    last_valid_data[i] = m_313Data[i];
                    last_valid_point[i] = current_data_point_ground[i];
                }    
            }
        }

        public double ParkingCenterDetermination(UltrasonicAxis[] ultrasonic_vehicle_location)
        {
            double FrontAverageDistance = 0;
            double RearAverageDistance = 0;
            for (int i=1;i<3;i++)
            {
                FrontAverageDistance += ultrasonic_vehicle_location[i].x;
            }
            FrontAverageDistance = FrontAverageDistance / 2;

            for (int i = 5; i < 7; i++)
            {
                RearAverageDistance += ultrasonic_vehicle_location[i].x;
            }
            RearAverageDistance = RearAverageDistance / 2;

            return (FrontAverageDistance + RearAverageDistance)/2.0;
        }
    }
}
