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
    public partial class Waveform : Form
    {
        Series VehicleSpeedWaveTarget = new Series();
        Series VehicleSpeedWaveActual = new Series();



        public Waveform()
        {
            InitializeComponent();

            waveform_chart.Series.Add(VehicleSpeedWaveTarget);
            waveform_chart.Series.Add(VehicleSpeedWaveActual);

            waveform_chart.ChartAreas[0].AxisY.Maximum = 3;
            waveform_chart.ChartAreas[0].AxisY.Minimum = 0;
            waveform_chart.ChartAreas[0].AxisY.Interval = 0.5;

            VehicleSpeedWaveTarget.ChartType = SeriesChartType.FastLine;
            VehicleSpeedWaveTarget.BorderWidth = 5;
            VehicleSpeedWaveTarget.BorderDashStyle = ChartDashStyle.Dash;
            VehicleSpeedWaveTarget.Color = Color.Green;
            VehicleSpeedWaveTarget.IsVisibleInLegend = true;
            VehicleSpeedWaveTarget.LegendText = "目标车速";


            VehicleSpeedWaveActual.ChartType = SeriesChartType.FastLine;
            VehicleSpeedWaveActual.BorderWidth = 3;
            VehicleSpeedWaveActual.BorderDashStyle = ChartDashStyle.Solid;
            VehicleSpeedWaveActual.Color = Color.Red;
            VehicleSpeedWaveActual.IsVisibleInLegend = true;
            VehicleSpeedWaveActual.LegendText = "实际车速";


        }

        /// <summary>
        /// 清除波形数据
        /// </summary>
        private void WaveformClear()
        {
            VehicleSpeedWaveTarget.Points.Clear();
            VehicleSpeedWaveActual.Points.Clear();
        }

        public void VehicleSpeedPointAdd(double targetV,double actualV)
        {
            VehicleSpeedWaveTarget.Points.AddY(targetV);
            VehicleSpeedWaveActual.Points.AddY(actualV);
            while (VehicleSpeedWaveTarget.Points.Count > 100)
            {
                VehicleSpeedWaveTarget.Points.RemoveAt(0);
            }
            while (VehicleSpeedWaveActual.Points.Count > 100)
            {
                VehicleSpeedWaveActual.Points.RemoveAt(0);
            }
        }

        #region 事件
        /// <summary>
        /// 波形图清除
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            WaveformClear();
        }
        #endregion
    }
}
