#include <SimpleFOC.h>
void setMks2808(BLDCMotor &motor) {
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity;
    motor.motion_downsample = 0.0;
    
    // velocity loop PID
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 24.0;
    motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 1000.0;
    motor.PID_velocity.limit = 1.0;
    // Low pass filtering time constant 
    motor.LPF_velocity.Tf = 0.01;

    //angle loop PID
    motor.P_angle.P = 20.0;
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.0;
    motor.P_angle.output_ramp = 0.0;
    motor.P_angle.limit = 20.0;
    // Low pass filtering time constant 
    motor.LPF_angle.Tf = 0.0;
    // current q loop PID 
    motor.PID_current_q.P = 3.0;
    motor.PID_current_q.I = 300.0;
    motor.PID_current_q.D = 0.0;
    motor.PID_current_q.output_ramp = 0.0;
    motor.PID_current_q.limit = 12.0;
    // Low pass filtering time constant 
    motor.LPF_current_q.Tf = 0.005;
    // current d loop PID
    motor.PID_current_d.P = 3.0;
    motor.PID_current_d.I = 300.0;
    motor.PID_current_d.D = 0.0;
    motor.PID_current_d.output_ramp = 0.0;
    motor.PID_current_d.limit = 12.0;
    // Low pass filtering time constant 
    motor.LPF_current_d.Tf = 0.005;
    // Limits 
    //motor.velocity_limit = 31.0;
    motor.voltage_limit = 1.0;
    motor.current_limit = 0.7;
    // sensor zero offset - home position 
    motor.sensor_offset = 0.0;
    // general settings 
    // motor phase resistance 
    //motor.phase_resistance = 5.72;
    // pwm modulation settings 
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
     motor.modulation_centered = 1.0;
}

void setMks2808_b(BLDCMotor &motor) {
    
// control loop type and torque mode 
motor.torque_controller = TorqueControlType::voltage;
motor.controller = MotionControlType::velocity;
motor.motion_downsample = 0.0;

// velocity loop PID
motor.PID_velocity.P = 0.5;
motor.PID_velocity.I = 50.0;
motor.PID_velocity.D = 0.001;
motor.PID_velocity.output_ramp = 1000.0;
motor.PID_velocity.limit = 1.0;
// Low pass filtering time constant 
motor.LPF_velocity.Tf = 0.01;
// angle loop PID
motor.P_angle.P = 20.0;
motor.P_angle.I = 0.0;
motor.P_angle.D = 0.0;
motor.P_angle.output_ramp = 0.0;
motor.P_angle.limit = 20.0;
// Low pass filtering time constant 
motor.LPF_angle.Tf = 0.0;
// current q loop PID 
motor.PID_current_q.P = 3.0;
motor.PID_current_q.I = 300.0;
motor.PID_current_q.D = 0.0;
motor.PID_current_q.output_ramp = 0.0;
motor.PID_current_q.limit = 12.0;
// Low pass filtering time constant 
motor.LPF_current_q.Tf = 0.005;
// current d loop PID
motor.PID_current_d.P = 3.0;
motor.PID_current_d.I = 300.0;
motor.PID_current_d.D = 0.0;
motor.PID_current_d.output_ramp = 0.0;
motor.PID_current_d.limit = 12.0;
// Low pass filtering time constant 
motor.LPF_current_d.Tf = 0.005;
// Limits 
motor.velocity_limit = 90.0;
motor.voltage_limit = 1.0;
motor.current_limit = 0.5;
// sensor zero offset - home position 
motor.sensor_offset = 0.0;
// general settings 
// motor phase resistance 
//motor.phase_resistance = 0.3;
// pwm modulation settings 
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
motor.modulation_centered = 1.0;
}