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
    public partial class MonitorForm : Form
    {
        Series SteeringAngleTarget = new Series();
        Series SteeringAngleActual = new Series();
        public MonitorForm()
        {
            InitializeComponent();

            monitor_form.Series.Add(SteeringAngleTarget);
            monitor_form.Series.Add(SteeringAngleActual);

            monitor_form.ChartAreas[0].AxisY.Maximum = 500;
            monitor_form.ChartAreas[0].AxisY.Minimum = -500;
            monitor_form.ChartAreas[0].AxisY.Interval = 100;

            SteeringAngleTarget.ChartType = SeriesChartType.FastLine;
            SteeringAngleTarget.BorderWidth = 5;
            SteeringAngleTarget.BorderDashStyle = ChartDashStyle.Dash;
            SteeringAngleTarget.Color = Color.Green;
            SteeringAngleTarget.IsVisibleInLegend = true;
            SteeringAngleTarget.LegendText = "目标转向角";


            SteeringAngleActual.ChartType = SeriesChartType.FastLine;
            SteeringAngleActual.BorderWidth = 3;
            SteeringAngleActual.BorderDashStyle = ChartDashStyle.Solid;
            SteeringAngleActual.Color = Color.Red;
            SteeringAngleActual.IsVisibleInLegend = true;
            SteeringAngleActual.LegendText = "实际转向角";
        }

        public void SteeringAnglePointAdd(double targetA, double actualA)
        {
            SteeringAngleTarget.Points.AddY(targetA);
            SteeringAngleActual.Points.AddY(actualA);
            while (SteeringAngleTarget.Points.Count > 200)
            {
                SteeringAngleTarget.Points.RemoveAt(0);
            }
            while (SteeringAngleActual.Points.Count > 200)
            {
                SteeringAngleActual.Points.RemoveAt(0);
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            SteeringAngleTarget.Points.Clear();
            SteeringAngleActual.Points.Clear();
        }
    }
}
