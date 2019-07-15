using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace APA_DebugAssistant
{
    public partial class AccelarateForm : Form
    {
        Series VehicleAccelerateWaveTarget = new Series();
        Series VehicleAccelerateWaveActual = new Series();
        Series VehicleAccelerateWaveLon = new Series();

        public AccelarateForm()
        {
            InitializeComponent();

            acc_chart.Series.Add(VehicleAccelerateWaveTarget);
            acc_chart.Series.Add(VehicleAccelerateWaveActual);
            acc_chart.Series.Add(VehicleAccelerateWaveLon);

            acc_chart.ChartAreas[0].AxisY.Maximum = 1;
            acc_chart.ChartAreas[0].AxisY.Minimum = -1;
            acc_chart.ChartAreas[0].AxisY.Interval = 0.1;

            VehicleAccelerateWaveTarget.ChartType = SeriesChartType.FastLine;
            VehicleAccelerateWaveTarget.BorderWidth = 5;
            VehicleAccelerateWaveTarget.BorderDashStyle = ChartDashStyle.Dash;
            VehicleAccelerateWaveTarget.Color = Color.Green;
            VehicleAccelerateWaveTarget.IsVisibleInLegend = true;
            VehicleAccelerateWaveTarget.LegendText = "目标加速度";

            VehicleAccelerateWaveActual.ChartType = SeriesChartType.FastLine;
            VehicleAccelerateWaveActual.BorderWidth = 3;
            VehicleAccelerateWaveActual.BorderDashStyle = ChartDashStyle.Solid;
            VehicleAccelerateWaveActual.Color = Color.Red;
            VehicleAccelerateWaveActual.IsVisibleInLegend = true;
            VehicleAccelerateWaveActual.LegendText = "实际控制加速度";

            VehicleAccelerateWaveLon.ChartType = SeriesChartType.FastLine;
            VehicleAccelerateWaveLon.BorderWidth = 3;
            VehicleAccelerateWaveLon.BorderDashStyle = ChartDashStyle.Solid;
            VehicleAccelerateWaveLon.Color = Color.Blue;
            VehicleAccelerateWaveLon.IsVisibleInLegend = true;
            VehicleAccelerateWaveLon.LegendText = "实际反馈加速度";

        }

        /// <summary>
        /// 清除波形数据
        /// </summary>
        private void WaveformClear()
        {
            VehicleAccelerateWaveTarget.Points.Clear();
            VehicleAccelerateWaveActual.Points.Clear();
            VehicleAccelerateWaveLon.Points.Clear();
        }

        public void VehicleAcceleratePointAdd(double targetV,double controlV, double actualV)
        {
            VehicleAccelerateWaveTarget.Points.AddY(targetV);
            VehicleAccelerateWaveActual.Points.AddY(controlV);
            VehicleAccelerateWaveLon.Points.AddY(actualV);

            while (VehicleAccelerateWaveTarget.Points.Count > 50)
            {
                VehicleAccelerateWaveTarget.Points.RemoveAt(0);
            }
            while (VehicleAccelerateWaveActual.Points.Count > 50)
            {
                VehicleAccelerateWaveActual.Points.RemoveAt(0);
            }
            while (VehicleAccelerateWaveLon.Points.Count > 50)
            {
                VehicleAccelerateWaveLon.Points.RemoveAt(0);
            }
        }

        #region 事件
        private void button1_Click(object sender, EventArgs e)
        {
            WaveformClear();
        }
        #endregion

    }
}
