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
        private double actual_acceleration_acc;
        private bool target_acceleration_enable;

        private double target_deceleration_aeb;
        private bool target_deceleration_enable;

        private double torque;
        private bool torque_enable;

        private double steering_angle_target;
        private byte steering_angle_active;

        private byte target_gear_shift;
        private byte actual_gear_shift;
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

        private double pulse_update_velocity;
        private double acc_update_velocity;
        // monitor distance
        private double target_distance;
        // wheel pulse
        private byte wheel_speed_direction;
        private UInt16 wheel_speed_rear_right_pulse;
        private UInt16 wheel_speed_rear_left_pulse;
        private UInt16 wheel_speed_front_right_pulse;
        private UInt16 wheel_speed_front_left_pulse;

        private Int32 wheel_speed_rear_left_pulse_sum;
        private Int32 wheel_speed_rear_right_pulse_sum;
        //  SAS Steering angle
        private double steering_angle_actual;
        private UInt16 steering_angle_speed;
        private bool steering_angle_valid;
        private bool sas_failure;
        
        // ESC
        private byte esc_status;
        // VCU
        private byte vcu_status;
        // EPS
        private byte eps_status;
        // EPB
        private byte epb_status;
        // SAS
        private byte sas_status;
        // TCU
        private byte tcu_status;
        // EMS
        private byte ems_status;

        private double lat_acc;
        private double lon_acc;
        private double yaw_rate;

        private double temperature;

        //EPB
        private byte epb_switch_position;

        //BCM
        private byte driver_seat_belt_switch_sts;
        private byte driver_door_sts;
        private byte passanger_door_sts;
        private byte trunk_sts;

        private byte turn_light_left_sts;
        private byte turn_light_right_sts;

        // system state
        private byte system_ready_sts;
        private byte auto_driver_mode_sts;
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

        public double Temperature
        {
            set
            {
                temperature = value;
            }
            get
            {
                return temperature;
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
        public double ActualAccelerationACC
        {
            set
            {
                actual_acceleration_acc = value;
            }
            get
            {
                return actual_acceleration_acc;
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

        public byte TargetGearShift
        {
            set
            {
                target_gear_shift = value;
            }
            get
            {
                return target_gear_shift;
            }
        }

        public byte ActualGearShift
        {
            set
            {
                actual_gear_shift = value;
            }
            get
            {
                return actual_gear_shift;
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
        public double PulseUpdateVelocity
        {
            set
            {
                pulse_update_velocity = value;
            }
            get
            {
                return pulse_update_velocity;
            }
        }

        public double AccUpdateVelocity
        {
            set
            {
                acc_update_velocity = value;
            }
            get
            {
                return acc_update_velocity;
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
        /// distace
        /// </summary>
        ///      
        public double TargetDistance
        {
            set
            {
                target_distance = value;
            }
            get
            {
                return target_distance;
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
        public UInt16 WheelSpeedRearRightPulse
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
        public UInt16 WheelSpeedRearLeftPulse
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
        public UInt16 WheelSpeedFrontRightPulse
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
        public UInt16 WheelSpeedFrontLeftPulse
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

        public Int32 WheelSpeedRearLeftPulseSum
        {
            set
            {
                wheel_speed_rear_left_pulse_sum = value;
            }
            get
            {
                return wheel_speed_rear_left_pulse_sum;
            }
        }

        public Int32 WheelSpeedRearRightPulseSum
        {
            set
            {
                wheel_speed_rear_right_pulse_sum = value;
            }
            get
            {
                return wheel_speed_rear_right_pulse_sum;
            }
        }

        // ESC
        public byte ESC_Status
        {
            set
            {
                esc_status = value;
            }
            get
            {
                return esc_status;
            }
        }
        //VCU
        public byte VCU_Status
        {
            set
            {
                vcu_status = value;
            }
            get
            {
                return vcu_status;
            }
        }

        // EPS
        public byte EPS_Status
        {
            set
            {
                eps_status = value;
            }
            get
            {
                return eps_status;
            }
        }

        // EPB
        public byte EPB_Status
        {
            set
            {
                epb_status = value;
            }
            get
            {
                return epb_status;
            }
        }
        // SAS
        public byte SAS_Status
        {
            set
            {
                sas_status = value;
            }
            get
            {
                return sas_status;
            }
        }
        // TCU
        public byte TCU_Status
        {
            set
            {
                tcu_status = value;
            }
            get
            {
                return tcu_status;
            }
        }
        // EMS
        public byte EMS_Status
        {
            set
            {
                ems_status = value;
            }
            get
            {
                return ems_status;
            }
        }

        //EPB
        public byte EPB_SwitchPosition
        {
            set
            {
                epb_switch_position = value;
            }
            get
            {
                return epb_switch_position;
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

        //BCM
        public byte DriverSeatBeltSwitchSts
        {
            set
            {
                driver_seat_belt_switch_sts = value;
            }
            get
            {
                return driver_seat_belt_switch_sts;
            }
        }
        //private byte driver_seat_belt_switch_sts;
        public byte DriverDoorSts
        {
            set
            {
                driver_door_sts = value;
            }
            get
            {
                return driver_door_sts;
            }
        }

        public byte PassangerDoorSts
        {
            set
            {
                passanger_door_sts = value;
            }
            get
            {
                return passanger_door_sts;
            }
        }

        public byte TrunkSts
        {
            set
            {
                trunk_sts = value;
            }
            get
            {
                return trunk_sts;
            }
        }

        public byte TurnLightLeftSts
        {
            set
            {
                turn_light_left_sts = value;
            }
            get
            {
                return turn_light_left_sts;
            }
        }

        public byte TurnLightRightSts
        {
            set
            {
                turn_light_right_sts = value;
            }
            get
            {
                return turn_light_right_sts;
            }
        }

        public byte SystemReadySts
        {
            set
            {
                system_ready_sts = value;
            }
            get
            {
                return system_ready_sts;
            }
        }
        public byte AutoDriverModeSts
        {
            set
            {
                auto_driver_mode_sts = value;
            }
            get
            {
                return auto_driver_mode_sts;
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
