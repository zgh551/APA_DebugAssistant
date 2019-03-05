using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace APA_DebugAssistant
{
    public class Vehicle
    {
        private byte save_time;

        #region Beiqi imformation
        /// <summary>
        /// Vehicle Attributes
        /// </summary>
        // speed relative
        private double speed_km;
        private double speed_m;
        private double speed_real;
        private bool speed_direction;
        private double speed_acc;
        

        // wheel angle and angle velocity and the torque
        private Int16 steering_wheel_target_angle;
        private Int16 steering_wheel_actual_angle;
        private UInt16 steering_wheel_agular_velocity;
        private double steering_wheel_torque;

        // about the brake of the vehicle
        private double actual_brake;
        private double target_brake;
        private double emergence_brake;
        // vehicle work status
        private byte status_of_ecu;
        private byte status_of_communication;
        #endregion

        /*****************长安车信息******************/
        //send messege
        private double target_acceleration_acc;
        private bool target_acceleration_enable;

        private double target_deceleration_aeb;
        private bool target_deceleration_enable;

        private double torque;
        private bool torque_enable;

        private double steering_angle_target;
        private byte steering_angle_active;

        private byte gear_shift;
        private bool gear_shift_enable;
        private bool gear_shift_valid;

        private bool velocity_enable;

        //receive messege
        private bool eps_failed;
        private bool apa_epas_failed;
        private bool apa_control_feedback;
        private bool torque_sensor_status;
        private double steering_torque;
        //Wheel Speed
        private byte wheel_speed_rear_left_direction;
        private byte wheel_speed_rear_left_valid;
        private double wheel_speed_rear_left_data;

        private byte wheel_speed_rear_right_direction;
        private byte wheel_speed_rear_right_valid;
        private double wheel_speed_rear_right_data;

        private byte wheel_speed_front_left_direction;
        private byte wheel_speed_front_left_valid;
        private double wheel_speed_front_left_data;

        private byte wheel_speed_front_right_direction;
        private byte wheel_speed_front_right_valid;
        private double wheel_speed_front_right_data;
        // vehicle speed
        private byte vehicle_speed_valid;
        private double vehicle_speed;
        private double target_vehicle_speed;
        // wheel pulse
        private byte wheel_speed_direction;
        private byte wheel_speed_rear_right_pulse;
        private byte wheel_speed_rear_left_pulse;
        private byte wheel_speed_front_right_pulse;
        private byte wheel_speed_front_left_pulse;
        //  SAS Steering angle
        private double steering_angle_actual;
        private UInt16 steering_angle_speed;
        private bool steering_angle_valid;
        private bool sas_failure;
        
        // ESP
        private bool esp_qdc_acc;
        // EMS
        private bool ems_qec_acc;

        private double lat_acc;
        private double lon_acc;
        private double yaw_rate;
        #region vehicle body
        /*** the vehicle body information ***/
        // Lenght
        private double wheelbase_lenght;
        private double front_overhang_distance;
        private double rear_overhang_distance;

        // width
        private double wheel_axis_width;
        private double wheel_axis_width_half;
        private double wheel_edge_distance;

        // the vehice four edge point 
        private double front_axis_lenght;
        private double rear_axis_lenght;
        private double beta_front;
        private double beta_rear;
        #endregion

        public double LatAcc
        {
            set
            {
                lat_acc = value;
            }
            get
            {
                return lat_acc;
            }
        }

        public double LonAcc
        {
            set
            {
                lon_acc = value;
            }
            get
            {
                return lon_acc;
            }
        }

        public double YawRate
        {
            set
            {
                yaw_rate = value;
            }
            get
            {
                return yaw_rate;
            }
        }

        public byte SaveTime
        {
            set
            {
                save_time = value;
            }
            get
            {
                return save_time;
            }
        }

        #region 北汽
        /// <summary>
        /// BeiQi Vehicle Attributes Interface
        /// </summary>
        public double Speed_km
        {
            set
            {
                speed_km = value;
            }
            get
            {
                return speed_km;
            }
        }
        public double Speed_m
        {
            set
            {
                speed_m = value;
            }
            get
            {
                return speed_m;
            }
        }
        public double SpeedReal
        {
            set
            {
                speed_real = value;
            }
            get
            {
                return speed_real;
            }
        }
        public bool SpeedDirection
        {
            set
            {
                speed_direction = value;
            }
            get
            {
                return speed_direction;
            }
        }
        public double SpeedAcc
        {
            set
            {
                speed_acc = value;
            }
            get
            {
                return speed_acc;
            }
        }
        
        public Int16 SteeringWheelTargetAngle
        {
            set
            {
                steering_wheel_target_angle = value;
            }
            get
            {
                return steering_wheel_target_angle;
            }
        }
        public Int16 SteeringWheelActualAngle
        {
            set
            {
                steering_wheel_actual_angle = value;
            }
            get
            {
                return steering_wheel_actual_angle;
            }
        }
        public UInt16 SteeringWheelAgularVelocity
        {
            set
            {
                steering_wheel_agular_velocity = value;
            }
            get
            {
                return steering_wheel_agular_velocity;
            }
        }
        public double SteeringWheelTorque
        {
            set
            {
                steering_wheel_torque = value;
            }
            get
            {
                return steering_wheel_torque;
            }
        }

        //vehicle brake
        public double ActualBrake
        {
            set
            {
                actual_brake = value;
            }
            get
            {
                return actual_brake;
            }
        }
        public double TargetBrake
        {
            set
            {
                target_brake = value;
            }
            get
            {
                return target_brake;
            }
        }
        public double EmergenceBrake
        {
            set
            {
                emergence_brake = value;
            }
            get
            {
                return emergence_brake;
            }
        }
        //vehice work status
        public byte StatusOfEcu
        {
            set
            {
                status_of_ecu = value;
            }
            get
            {
                return status_of_ecu;
            }
        }
        public byte StatusOfCommunication
        {
            set
            {
                status_of_communication = value;
            }
            get
            {
                return status_of_communication;
            }
        }
        #endregion

        #region 长安
        /// <summary>
        /// ChangAn Vehicle Attributes Interface
        /// </summary>
        /*** Control Imformation ***/
        //Control imformation
        public double TargetAccelerationACC
        {
            set
            {
                target_acceleration_acc = value;
            }
            get
            {
                return target_acceleration_acc;
            }
        }
        public bool TargetAccelerationEnable
        {
            set
            {
                target_acceleration_enable = value;
            }
            get
            {
                return target_acceleration_enable;
            }
        }

        public double TargetDecelerationAEB
        {
            set
            {
                target_deceleration_aeb = value;
            }
            get
            {
                return target_deceleration_aeb;
            }
        }
        public bool TargetDecelerationEnable
        {
            set
            {
                target_deceleration_enable = value;
            }
            get
            {
                return target_deceleration_enable;
            }
        }

        public double Torque
        {
            set
            {
                torque = value;
            }
            get
            {
                return torque;
            }
        }
        public bool TorqueEnable
        {
            set
            {
                torque_enable = value;
            }
            get
            {
                return torque_enable;
            }
        }

        public double SteeringAngleTarget
        {
            set
            {
                steering_angle_target = value;
            }
            get
            {
                return steering_angle_target;
            }
        }

        public byte SteeringAngleActive
        {
            set
            {
                steering_angle_active = value;
            }
            get
            {
                return steering_angle_active;
            }
        }

        public byte GearShift
        {
            set
            {
                gear_shift = value;
            }
            get
            {
                return gear_shift;
            }
        }
        public bool GearShiftEnable
        {
            set
            {
                gear_shift_enable = value;
            }
            get
            {
                return gear_shift_enable;
            }
        }
        public bool GearShiftValid
        {
            set
            {
                gear_shift_valid = value;
            }
            get
            {
                return gear_shift_valid;
            }
        }

        public bool VelocityEnable
        {
            set
            {
                velocity_enable = value;
            }
            get
            {
                return velocity_enable;
            }
        }
        
        /// <summary>
        /// receive imformation
        /// </summary>
        /// 
        public bool EPS_Failed
        {
            set
            {
                eps_failed = value;
            }
            get
            {
                return eps_failed;
            }
        }
        public bool APA_EpasFailed
        {
            set
            {
                apa_epas_failed = value;
            }
            get
            {
                return apa_epas_failed;
            }
        }
        public bool APA_ControlFeedback
        {
            set
            {
                apa_control_feedback = value;
            }
            get
            {
                return apa_control_feedback;
            }
        }
        public bool TorqueSensorStatus
        {
            set
            {
                torque_sensor_status = value;
            }
            get
            {
                return torque_sensor_status;
            }
        }
        public double SteeringTorque
        {
            set
            {
                steering_torque = value;
            }
            get
            {
                return steering_torque;
            }
        }

        /// <summary>
        /// Wheel Speed
        /// </summary>
        /// 
        public byte WheelSpeedRearLeftDirection
        {
            set
            {
                wheel_speed_rear_left_direction = value;
            }
            get
            {
                return wheel_speed_rear_left_direction;
            }
        }
        public byte WheelSpeedRearLeftValid
        {
            set
            {
                wheel_speed_rear_left_valid = value;
            }
            get
            {
                return wheel_speed_rear_left_valid;
            }
        }
        public double WheelSpeedRearLeftData
        {
            set
            {
                wheel_speed_rear_left_data = value;
            }
            get
            {
                return wheel_speed_rear_left_data;
            }
        }

        public byte WheelSpeedRearRightDirection
        {
            set
            {
                wheel_speed_rear_right_direction = value;
            }
            get
            {
                return wheel_speed_rear_right_direction;
            }
        }
        public byte WheelSpeedRearRightValid
        {
            set
            {
                wheel_speed_rear_right_valid = value;
            }
            get
            {
                return wheel_speed_rear_right_valid;
            }
        }
        public double WheelSpeedRearRightData
        {
            set
            {
                wheel_speed_rear_right_data = value;
            }
            get
            {
                return wheel_speed_rear_right_data;
            }
        }

        public byte WheelSpeedFrontLeftDirection
        {
            set
            {
                wheel_speed_front_left_direction = value;
            }
            get
            {
                return wheel_speed_front_left_direction;
            }
        }
        public byte WheelSpeedFrontLeftValid
        {
            set
            {
                wheel_speed_front_left_valid = value;
            }
            get
            {
                return wheel_speed_front_left_valid;
            }
        }
        public double WheelSpeedFrontLeftData
        {
            set
            {
                wheel_speed_front_left_data = value;
            }
            get
            {
                return wheel_speed_front_left_data;
            }
        }

        public byte WheelSpeedFrontRightDirection
        {
            set
            {
                wheel_speed_front_right_direction = value;
            }
            get
            {
                return wheel_speed_front_right_direction;
            }
        }
        public byte WheelSpeedFrontRightValid
        {
            set
            {
                wheel_speed_front_right_valid = value;
            }
            get
            {
                return wheel_speed_front_right_valid;
            }
        }
        public double WheelSpeedFrontRightData
        {
            set
            {
                wheel_speed_front_right_data = value;
            }
            get
            {
                return wheel_speed_front_right_data;
            }
        }

        /// <summary>
        /// Vehicle Speed
        /// </summary>
        /// 
        public byte VehicleSpeedValid
        {
            set
            {
                vehicle_speed_valid = value;
            }
            get
            {
                return vehicle_speed_valid;
            }
        }
        public double VehicleSpeed
        {
            set
            {
                vehicle_speed = value;
            }
            get
            {
                return vehicle_speed;
            }
        }

        public double TargetVehicleSpeed
        {
            set
            {
                target_vehicle_speed = value;
            }
            get
            {
                return target_vehicle_speed;
            }
        }
        
        /// <summary>
        /// wheel speed pulse
        /// </summary>
        /// 
        public byte WheelSpeedDirection
        {
            set
            {
                wheel_speed_direction = value;
            }
            get
            {
                return wheel_speed_direction;
            }
        }
        public byte WheelSpeedRearRightPulse
        {
            set
            {
                wheel_speed_rear_right_pulse = value;
            }
            get
            {
                return wheel_speed_rear_right_pulse;
            }
        }
        public byte WheelSpeedRearLeftPulse
        {
            set
            {
                wheel_speed_rear_left_pulse = value;
            }
            get
            {
                return wheel_speed_rear_left_pulse;
            }
        }
        public byte WheelSpeedFrontRightPulse
        {
            set
            {
                wheel_speed_front_right_pulse = value;
            }
            get
            {
                return wheel_speed_front_right_pulse;
            }
        }
        public byte WheelSpeedFrontLeftPulse
        {
            set
            {
                wheel_speed_front_left_pulse = value;
            }
            get
            {
                return wheel_speed_front_left_pulse;
            }
        }

        // ESP
        public bool ESPQDCACC
        {
            set
            {
                esp_qdc_acc = value;
            }
            get
            {
                return esp_qdc_acc;
            }
        }
        //EMS
        public bool EMSQECACC
        {
            set
            {
                ems_qec_acc = value;
            }
            get
            {
                return ems_qec_acc;
            }
        }
        //  SAS Steering angle
        public double SteeringAngleActual
        {
            set
            {
                steering_angle_actual = value;
            }
            get
            {
                return steering_angle_actual;
            }
        }
        public UInt16 SteeringAngleSpeed
        {
            set
            {
                steering_angle_speed = value;
            }
            get
            {
                return steering_angle_speed;
            }
        }
        public bool SteeringAngleValid
        {
            set
            {
                steering_angle_valid = value;
            }
            get
            {
                return steering_angle_valid;
            }
        }
        public bool SASFailure
        {
            set
            {
                sas_failure = value;
            }
            get
            {
                return sas_failure;
            }
        }

        #endregion
        /*** the information of the vehicle body ***/
        public double WheelBaseLenght
        {
            set
            {
                wheelbase_lenght = value;
            }
            get
            {
                return wheelbase_lenght;
            }
        }
        public double FrontOverhangDistance
        {
            set
            {
                front_overhang_distance = value;
            }
            get
            {
                return front_overhang_distance;
            }
        }
        public double RearOverhangDistance
        {
            set
            {
                rear_overhang_distance = value;
            }
            get
            {
                return rear_overhang_distance;
            }
        }

        public double WheelAxisWidth
        {
            set
            {
                wheel_axis_width = value;
            }
            get
            {
                return wheel_axis_width;
            }
        }
        public double WheelAxisWidthHalf
        {
            set
            {
                wheel_axis_width_half = value;
            }
            get
            {
                return wheel_axis_width_half;
            }
        }
        public double WheelEdgeDistance
        {
            set
            {
                wheel_edge_distance = value;
            }
            get
            {
                return wheel_edge_distance;
            }
        }

        public double BetaFront
        {
            set
            {
                beta_front = value;
            }
            get
            {
                return beta_front;
            }
        }
        public double BetaRear
        {
            set
            {
                beta_rear = value;
            }
            get
            {
                return beta_rear;
            }
        }
        public double FrontAxisLenght
        {
            set
            {
                front_axis_lenght = value;
            }
            get
            {
                return front_axis_lenght;
            }
        }
        public double RearAxisLenght
        {
            set
            {
                rear_axis_lenght = value;
            }
            get
            {
                return rear_axis_lenght;
            }
        }

        public Vehicle()
        {
            // 北汽车的信息
            wheelbase_lenght = 2.64;//unit:m
            front_overhang_distance = 0.952;//unit:m
            rear_overhang_distance = 0.905;//unit:m
            wheel_axis_width = 1.794;//unit:m
            wheel_axis_width_half = 0.797;
            wheel_edge_distance = 0.132;

            speed_acc = 0.5;
            beta_front = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / (FrontOverhangDistance + WheelBaseLenght));
            beta_rear = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / RearOverhangDistance);

            front_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance,2) + Math.Pow(WheelBaseLenght + FrontOverhangDistance, 2));
            rear_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance, 2) + Math.Pow(RearOverhangDistance, 2));

            // speed relative
            speed_km = 0.0;
            speed_m = 0.0;
            speed_real = 0.0;
            speed_direction = true;

            // wheel angle and angle velocity and the torque
            steering_wheel_target_angle = 0;
            steering_wheel_actual_angle = 0;
            steering_wheel_agular_velocity = 0;
            steering_wheel_torque = 0;

            // about the brake of the vehicle
            actual_brake = 0.0;
            target_brake = 1.0;
            emergence_brake = 1.2;
            // vehicle work status
            status_of_ecu = 0;
            status_of_communication = 0;
        }

        public Vehicle(byte type)
        {
            if(type == 0)//北汽
            {
                // 北汽车的信息
                wheelbase_lenght = 2.65;//unit:m
                front_overhang_distance = 0.952;//unit:m
                rear_overhang_distance = 1;//unit:m
                wheel_axis_width = 1.794;//unit:m
                wheel_axis_width_half = 0.765;
                wheel_edge_distance = 0.132;

                speed_acc = 0.5;
                beta_front = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / (FrontOverhangDistance + WheelBaseLenght));
                beta_rear = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / RearOverhangDistance);

                front_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance, 2) + Math.Pow(WheelBaseLenght + FrontOverhangDistance, 2));
                rear_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance, 2) + Math.Pow(RearOverhangDistance, 2));

                // speed relative
                speed_km = 0.0;
                speed_m = 0.0;
                speed_real = 0.0;
                speed_direction = true;

                // wheel angle and angle velocity and the torque
                steering_wheel_target_angle = 0;
                steering_wheel_actual_angle = 0;
                steering_wheel_agular_velocity = 0;
                steering_wheel_torque = 0;

                // about the brake of the vehicle
                actual_brake = 0.0;
                target_brake = 1.0;
                emergence_brake = 1.2;
                // vehicle work status
                status_of_ecu = 0;
                status_of_communication = 0;
            }
            else if(type == 1)//长安
            {
                // 北汽车的信息
                wheelbase_lenght = 2.65;//unit:m
                front_overhang_distance = 0.952;//unit:m
                rear_overhang_distance = 1;//unit:m
                wheel_axis_width = 1.794;//unit:m
                wheel_axis_width_half = 0.765;
                wheel_edge_distance = 0.132;

                speed_acc = 0.5;
                beta_front = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / (FrontOverhangDistance + WheelBaseLenght));
                beta_rear = Math.Atan((WheelAxisWidthHalf + WheelEdgeDistance) / RearOverhangDistance);

                front_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance, 2) + Math.Pow(WheelBaseLenght + FrontOverhangDistance, 2));
                rear_axis_lenght = Math.Sqrt(Math.Pow(WheelAxisWidthHalf + WheelEdgeDistance, 2) + Math.Pow(RearOverhangDistance, 2));

                // speed relative
                speed_km = 0.0;
                speed_m = 0.0;
                speed_real = 0.0;
                speed_direction = true;

                // wheel angle and angle velocity and the torque
                steering_wheel_target_angle = 0;
                steering_wheel_actual_angle = 0;
                steering_wheel_agular_velocity = 0;
                steering_wheel_torque = 0;

                // about the brake of the vehicle
                actual_brake = 0.0;
                target_brake = 1.0;
                emergence_brake = 1.2;
                // vehicle work status
                status_of_ecu = 0;
                status_of_communication = 0;
            }
            else
            {

            }
        }

        public void SteeringAngleControl(Vehicle m_vehicle,double dt)
        {
            double da = m_vehicle.SteeringWheelAgularVelocity * dt;
            double left_target_angle = m_vehicle.SteeringWheelTargetAngle - da;
            double right_target_angle = m_vehicle.SteeringWheelTargetAngle + da;

            if(m_vehicle.steering_angle_actual < left_target_angle)
            {
                m_vehicle.steering_angle_actual += da;
                Console.WriteLine("steering_angle:" + m_vehicle.steering_angle_actual.ToString()+"\r");
            }
            else if(m_vehicle.steering_angle_actual > right_target_angle)
            {
                m_vehicle.steering_angle_actual -= da;
                Console.WriteLine("steering_angle:" + m_vehicle.steering_angle_actual.ToString() + "\r");
            }
            else
            {
                m_vehicle.steering_angle_actual = m_vehicle.SteeringWheelTargetAngle;
            }
        }
    }
}
