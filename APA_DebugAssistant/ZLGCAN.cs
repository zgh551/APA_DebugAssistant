using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace APA_DebugAssistant
{ 
    class ZLGCAN
    {
        #region ZLG CAN Struct
        //1.ZLGCAN系列接口卡信息的数据类型。
        public struct VCI_BOARD_INFO
        {
            public UInt16 hw_Version;
            public UInt16 fw_Version;
            public UInt16 dr_Version;
            public UInt16 in_Version;
            public UInt16 irq_Num;
            public byte can_Num;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)] public byte[] str_Serial_Num;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 40)]
            public byte[] str_hw_Type;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] Reserved;
        }


        /////////////////////////////////////////////////////
        //2.定义CAN信息帧的数据类型。
        unsafe public struct VCI_CAN_OBJ  //使用不安全代码
        {
            public uint ID;
            public uint TimeStamp;
            public byte TimeFlag;
            public byte SendType;
            public byte RemoteFlag;//是否是远程帧
            public byte ExternFlag;//是否是扩展帧
            public byte DataLen;

            public fixed byte Data[8];

            public fixed byte Reserved[3];

        }
        //////2.定义CAN信息帧的数据类型。
        //public struct VCI_CAN_OBJ 
        //{
        //    public UInt32 ID;
        //    public UInt32 TimeStamp;
        //    public byte TimeFlag;
        //    public byte SendType;
        //    public byte RemoteFlag;//是否是远程帧
        //    public byte ExternFlag;//是否是扩展帧
        //    public byte DataLen;
        //    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        //    public byte[] Data;
        //    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        //    public byte[] Reserved;

        //    public void Init()
        //    {
        //        Data = new byte[8];
        //        Reserved = new byte[3];
        //    }
        //}

        //3.定义CAN控制器状态的数据类型。
        public struct VCI_CAN_STATUS
        {
            public byte ErrInterrupt;
            public byte regMode;
            public byte regStatus;
            public byte regALCapture;
            public byte regECCapture;
            public byte regEWLimit;
            public byte regRECounter;
            public byte regTECounter;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public byte[] Reserved;
        }

        //4.定义错误信息的数据类型。
        public struct VCI_ERR_INFO
        {
            public UInt32 ErrCode;
            public byte Passive_ErrData1;
            public byte Passive_ErrData2;
            public byte Passive_ErrData3;
            public byte ArLost_ErrData;
        }

        //5.定义初始化CAN的数据类型
        public struct VCI_INIT_CONFIG
        {
            public UInt32 AccCode;
            public UInt32 AccMask;
            public UInt32 Reserved;
            public byte Filter;
            public byte Timing0;
            public byte Timing1;
            public byte Mode;
        }

        public struct CHGDESIPANDPORT
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public byte[] szpwd;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public byte[] szdesip;
            public Int32 desport;

            public void Init()
            {
                szpwd = new byte[10];
                szdesip = new byte[20];
            }
        }

        ///////// new add struct for filter /////////
        //typedef struct _VCI_FILTER_RECORD{
        //    DWORD ExtFrame;	//是否为扩展帧
        //    DWORD Start;
        //    DWORD End;
        //}VCI_FILTER_RECORD,*PVCI_FILTER_RECORD;
        public struct VCI_FILTER_RECORD
        {
            public UInt32 ExtFrame;
            public UInt32 Start;
            public UInt32 End;
        }
        #endregion

        #region ZLG CAN Variable define and the interface
        // 设备型号
        const int VCI_PCI5121 = 1;
        const int VCI_PCI9810 = 2;
        const int VCI_USBCAN1 = 3;
        const int VCI_USBCAN2 = 4;
        const int VCI_USBCAN2A = 4;
        const int VCI_PCI9820 = 5;
        const int VCI_CAN232 = 6;
        const int VCI_PCI5110 = 7;
        const int VCI_CANLITE = 8;
        const int VCI_ISA9620 = 9;
        const int VCI_ISA5420 = 10;
        const int VCI_PC104CAN = 11;
        const int VCI_CANETUDP = 12;
        const int VCI_CANETE = 12;
        const int VCI_DNP9810 = 13;
        const int VCI_PCI9840 = 14;
        const int VCI_PC104CAN2 = 15;
        const int VCI_PCI9820I = 16;
        const int VCI_CANETTCP = 17;
        const int VCI_PEC9920 = 18;
        const int VCI_PCI5010U = 19;
        const int VCI_USBCAN_E_U = 20;
        const int VCI_USBCAN_2E_U = 21;
        const int VCI_PCI5020U = 22;
        const int VCI_EG20T_CAN = 23;
        const int VCI_USBCAN_4E_U = 31;

        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_OpenDevice(UInt32 DeviceType, UInt32 DeviceInd, UInt32 Reserved);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_CloseDevice(UInt32 DeviceType, UInt32 DeviceInd);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_InitCAN(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, ref VCI_INIT_CONFIG pInitConfig);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_ReadBoardInfo(UInt32 DeviceType, UInt32 DeviceInd, ref VCI_BOARD_INFO pInfo);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_ReadErrInfo(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, ref VCI_ERR_INFO pErrInfo);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_ReadCANStatus(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, ref VCI_CAN_STATUS pCANStatus);

        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_GetReference(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, UInt32 RefType, ref byte pData);
        [DllImport("controlcan.dll")]
        //static extern UInt32 VCI_SetReference(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, UInt32 RefType, ref byte pData);
        unsafe static extern UInt32 VCI_SetReference(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, UInt32 RefType, byte* pData);

        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_GetReceiveNum(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_ClearBuffer(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd);

        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_StartCAN(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd);
        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_ResetCAN(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd);

        [DllImport("controlcan.dll")]
        static extern UInt32 VCI_Transmit(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, ref VCI_CAN_OBJ pSend, UInt32 Len);

        //[DllImport("controlcan.dll")]
        //static extern UInt32 VCI_Receive(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, ref VCI_CAN_OBJ pReceive, UInt32 Len, Int32 WaitTime);
        [DllImport("controlcan.dll", CharSet = CharSet.Ansi)]
        static extern UInt32 VCI_Receive(UInt32 DeviceType, UInt32 DeviceInd, UInt32 CANInd, IntPtr pReceive, UInt32 Len, Int32 WaitTime);

        //static UInt32 m_devtype = 4;//USBCAN2
        private static UInt32 m_devtype = VCI_USBCAN_4E_U;
        //usb-e-u 波特率
        private static UInt32[] GCanBrTab = new UInt32[10]{
                        1000000,800000,500000,
                        250000, 125000, 100000,
                        50000, 20000, 10000,
                        5000
                };

        private const UInt32 STATUS_OK = 1;
        private UInt32 m_bOpen = 0;
        private UInt32 m_devind = 0;
        private UInt32 m_canind = 0;

        VCI_CAN_OBJ[] m_recobj = new VCI_CAN_OBJ[50];

        UInt32[] m_arrdevtype = new UInt32[20];

        //string[] DeviceType = new string[2] { "USBCAN_4E_U", "USBCAN_2E_U" };
        //string[] BaudRate = new string[10] { "1000kbps", "800kbps", "500kbps", "250kbps", "125kbps", "100kbps", "50kbps", "20kbps", "10kbps", "5kbps" };
        //string[] ECU_Status = new string[8] { "待机模式", "自动驾驶模式", "未知", "未知", "手动模式", "手动介入恢复模式", "警告模式", "错误模式" };
        //string[] EPS_Status = new string[2] { "正常", "故障" };
        //string[] ESP_Status = new string[4] { "正常", "故障1", "故障2", "故障3" };
        //string[] EMS_Status = new string[4] { "正常", "故障1", "故障2", "故障3" };
        //string[] ComunicationStatus = new string[2] { "通信正常", "通信异常" };

        public UInt32 OpenStatus
        {
            set
            {
                m_bOpen = value;
            }
            get
            {
                return m_bOpen;
            }
        }
        #endregion

        public ZLGCAN()
        {
            m_bOpen = 0;
            m_devtype = VCI_USBCAN_4E_U;
            m_devind = 0;
        }
        #region CAN 操作
        /// <summary>
        /// CAN设备连接
        /// </summary>
        /// <param name="id"> the can id number </param>
        unsafe public void CAN_Connect(UInt32 id)
        {
            m_canind = id;
            if(m_bOpen == 0)
            {
                if (VCI_OpenDevice(m_devtype, m_devind, 0) == 0)
                {
                    MessageBox.Show("打开设备失败,请检查设备类型和设备索引号是否正确", "错误",
                            MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                    return;
                }
            }
            m_bOpen = 1;
            //USB-E-U 代码
            UInt32 baud = GCanBrTab[2];
            if (VCI_SetReference(m_devtype, m_devind, m_canind, 0, (byte*)&baud) != STATUS_OK)
            {
                MessageBox.Show("设置波特率错误，打开设备0失败!", "错误", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                VCI_CloseDevice(m_devtype, m_devind);
                return;
            }
            //滤波设置
            //////////////////////////////////////////////////////////////////////////
            VCI_INIT_CONFIG config = new VCI_INIT_CONFIG();
            config.AccCode = 00000000;// System.Convert.ToUInt32("0x" + textBox_AccCode.Text, 16);
            config.AccMask = 0xFFFFFFFF;// System.Convert.ToUInt32("0x" + textBox_AccMask.Text, 16);
            config.Timing0 = 0;// System.Convert.ToByte("0x" + textBox_Time0.Text, 16);
            config.Timing1 = 14;// System.Convert.ToByte("0x" + textBox_Time1.Text, 16);
            config.Filter = 1;// 单滤波 (Byte)comboBox_Filter.SelectedIndex;
            config.Mode = 0;//正常模式 (Byte)comboBox_Mode.SelectedIndex;
            VCI_InitCAN(m_devtype, m_devind, m_canind, ref config);
            //////////////////////////////////////////////////////////////////////////
            Int32 filterMode = 2;// comboBox_e_u_Filter.SelectedIndex;
            if (2 != filterMode)//不是禁用
            {
                VCI_FILTER_RECORD filterRecord = new VCI_FILTER_RECORD();
                filterRecord.ExtFrame = (UInt32)filterMode;
                filterRecord.Start = 1;// System.Convert.ToUInt32("0x" + textBox_e_u_startid.Text, 16);
                filterRecord.End = 0xff;// System.Convert.ToUInt32("0x" + textBox_e_u_endid.Text, 16);
                                        //填充滤波表格

                VCI_SetReference(m_devtype, m_devind, m_canind, 1, (byte*)&filterRecord);
                //使滤波表格生效
                byte tm = 0;
                if (VCI_SetReference(m_devtype, m_devind, m_canind, 2, &tm) != STATUS_OK)
                {
                    MessageBox.Show("设置滤波失败", "错误", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                    VCI_CloseDevice(m_devtype, m_devind);
                    return;
                }
            }
        }

        /// <summary>
        /// 启动CAN的端口号
        /// </summary>
        /// <param name="id"></param>
        public void CAN_Open(UInt32 id)
        {
            if (m_bOpen == 0)
                return;
            VCI_StartCAN(m_devtype, m_devind, id);
        }

        /// <summary>
        /// 复位CAN的端口号
        /// </summary>
        /// <param name="id"></param>
        public void CAN_Reset(UInt32 id)
        {
            if (m_bOpen == 0)
                return;
            VCI_ResetCAN(m_devtype, m_devind, id);
        }

        /// <summary>
        /// 关闭CAN的端口号
        /// </summary>
        /// <param name="id"></param>
        public void CAN_Close()
        {
            VCI_CloseDevice(m_devtype, m_devind);
            m_bOpen = 0;
        }

        #endregion

        #region 收发操作
        /// <summary>
        /// CAN0 Receive Function
        /// </summary>
        unsafe public void CAN_Receive(UInt32 ind,ref VCI_CAN_OBJ[] obj)
        {
            UInt32 res = new UInt32();
            res = VCI_GetReceiveNum(m_devtype, m_devind, ind);
            if (res == 0)
                return;
            IntPtr pt = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(VCI_CAN_OBJ)) * (Int32)res);

            res = VCI_Receive(m_devtype, m_devind, ind, pt, res, 100);

            obj = new VCI_CAN_OBJ[res];
            for (UInt32 i = 0; i < res; i++)
            {
                obj[i] = (VCI_CAN_OBJ)Marshal.PtrToStructure((IntPtr)((UInt32)pt + i * Marshal.SizeOf(typeof(VCI_CAN_OBJ))), typeof(VCI_CAN_OBJ));
            }
            Marshal.FreeHGlobal(pt);
        }

        /// <summary>
        /// CAN Receive Function
        /// </summary>
        /// <param name="id"></param>
        /// <param name="length"></param>
        /// <param name="data"></param>
        unsafe public void CAN_Send(UInt32 ind,uint id, byte length, byte[] data)
        {
            if (m_bOpen == 0)
                return;

            VCI_CAN_OBJ sendobj = new VCI_CAN_OBJ();
            //sendobj.Init();
            sendobj.SendType = 0;//0 -> 正常发送 ;2 -> 自发自收(byte)comboBox_SendType.SelectedIndex;
            sendobj.RemoteFlag = 0;//标准帧 (byte)comboBox_FrameFormat.SelectedIndex;
            sendobj.ExternFlag = 0;// 标准帧数(byte)comboBox_FrameType.SelectedIndex;
            sendobj.ID = id;// System.Convert.ToUInt32("0x" + textBox_ID.Text, 16);
            sendobj.DataLen = length;

            for (int i = 0; i < 8; i++)
            {
                sendobj.Data[i] = data[i];
            }
            int nTimeOut = 3000;
            VCI_SetReference(m_devtype, m_devind, ind, 4, (byte*)&nTimeOut);
            if (VCI_Transmit(m_devtype, m_devind, ind, ref sendobj, 1) == 0)
            {
                MessageBox.Show("发送失败", "错误",MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }
        }
        #endregion
    }
}
