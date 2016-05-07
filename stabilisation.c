#include "read_sensors.h" //initializing and reading i2c sensors
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <math.h>
#include "serial_com.h" //uart
#include "AHRS.h"//Complementary filter AHRS
#include "PWM.h"//i2c PWM motor control 

 uint8_t gain_P_X = 0;
 uint8_t gain_i_X = 0;
 uint8_t gain_D_X = 0;

 uint8_t gain_P_Y = 0;
 uint8_t gain_i_Y = 0;
 uint8_t gain_D_Y = 0;

 uint8_t gain_P_Z = 0;
 uint8_t gain_i_Z = 0;
 
 uint8_t gain_P_X_O = 0;
 uint8_t gain_P_Y_O = 0;

 float pitch_control = 0;
 float roll_control = 0;
 float z_control = 0;
 
 float prev_error_pitch = 0;
 float prev_error_roll = 0;
 
 float i_cmd_pitch = 0;
 float i_cmd_roll = 0;
 

void PID_stabilisation(double delta_t)
{
    uint16_t motor_1 = 0;//right front
    uint16_t motor_2 = 0;//left front
    uint16_t motor_3 = 0;//left back
    uint16_t motor_4 = 0;//right back
    
    float command_angle_pitch = (((float)y_com)/100)+90+18.9;//((float)y_com/100)+90;+(1891/100)
    float command_angle_roll = (((float)x_com)/100)+90;//(-(float)x_com/100)+90;
    
    //pitch_control = command_angle_pitch;
    //roll_control = command_angle_roll;// comp_angle_x - command_angle_x;
    
    float error_pitch = comp_angle_pitch - command_angle_pitch;
    float error_roll = comp_angle_roll - command_angle_roll;
    
    float p_cmd_pitch = error_pitch*((float)gain_P_X/15);
    float p_cmd_roll = error_roll*((float)gain_P_Y/15);
    
    i_cmd_pitch += error_pitch*((float)gain_i_X/20)*delta_t;
    if(i_cmd_pitch > 120)i_cmd_pitch = 120;//prevent integral windup
    if(i_cmd_pitch < -120)i_cmd_pitch = -120;
    i_cmd_roll += error_roll*((float)gain_i_Y/20)*delta_t;
    if(i_cmd_roll > 120)i_cmd_roll = 120;//prevent integral windup
    if(i_cmd_roll < -120)i_cmd_roll = -120;
    
    float d_cmd_pitch = ((error_pitch-prev_error_pitch)/delta_t)*((float)gain_D_X/20);
    float d_cmd_roll = ((error_roll-prev_error_roll)/delta_t)*((float)gain_D_Y/20);
    
    prev_error_pitch = error_pitch;
    prev_error_roll = error_roll;
    
    pitch_control = p_cmd_pitch + i_cmd_pitch + d_cmd_pitch;
    roll_control = p_cmd_roll + i_cmd_roll + d_cmd_roll;
    
    //int throttle = 205 + (t_com + 3276)/26;
    //if(throttle < 205)throttle = 205;
    
    int throttle = 819 + (t_com + 3276)/7;
    if(throttle < 819)throttle = 819;
    
    motor_1 = throttle + pitch_control + roll_control;
    motor_2 = throttle + pitch_control - roll_control;
    motor_3 = throttle - pitch_control - roll_control;
    motor_4 = throttle - pitch_control + roll_control;
    
    
    pwm_set_all(motor_1,motor_2,motor_3,motor_4);
    //pwm_set_all(throttle,205,205,205 );
    //pwm_set_all(205,throttle,205,205 );
    //pwm_set_all(205,205,throttle,205 );
    //pwm_set_all(819,819,throttle,819 );
    //pwm_set_all(205,205,205,throttle );
}

void PID_cascaded(double delta_t)
{
    uint16_t motor_1 = 0;//right front
    uint16_t motor_2 = 0;//left front
    uint16_t motor_3 = 0;//left back
    uint16_t motor_4 = 0;//right back
    
    float command_angle_pitch = (((float)y_com)/100)+90+10;//((float)y_com/100)+90;+(1891/100)
    float command_angle_roll = (((float)x_com)/100)+90;//(-(float)x_com/100)+90;
    
    float command_rate_pitch = (((float)y_com)/1);//no scaling 30 deg/sec is 30*130=3900 lsb
    float command_rate_roll = (((float)x_com)/1);//
    
    //ATTITUDE LOOP
    
    float error_pitch = comp_angle_pitch - command_angle_pitch;
    float error_roll = comp_angle_roll - command_angle_roll;
    
    float p_cmd_pitch = error_pitch*((float)gain_P_X_O*2);//was gain_P_X
    float p_cmd_roll = error_roll*((float)gain_P_Y_O*2);
    
    i_cmd_pitch += error_pitch*((float)gain_i_X/200)*delta_t;
    if(i_cmd_pitch > 40)i_cmd_pitch = 40;//prevent integral windup
    if(i_cmd_pitch < -40)i_cmd_pitch = -40;
    i_cmd_roll += error_roll*((float)gain_i_Y/200)*delta_t;
    if(i_cmd_roll > 40)i_cmd_roll = 40;//prevent integral windup
    if(i_cmd_roll < -40)i_cmd_roll = -40;
    
    float pitch_control_rate = p_cmd_pitch + i_cmd_pitch;
    float roll_control_rate =  p_cmd_roll + i_cmd_roll;
    
    //RATE LOOP
    
    //Rate control
    //float rate_error_pitch = (float)(-y_gyro_raw) - command_rate_pitch;
    //float rate_error_roll = (float)x_gyro_raw - command_rate_roll;
    
    //Outer loop control
    float rate_error_pitch = (float)(-y_gyro_raw) - (-pitch_control_rate);
    float rate_error_roll  = (float)x_gyro_raw - (-roll_control_rate);
    
    float p_cmd_pitch_r = rate_error_pitch*((float)gain_P_X/20000);
    float p_cmd_roll_r = rate_error_roll*((float)gain_P_Y/20000);//was gain_P_Y but is now temporary used by outer loop P 
    
    float d_cmd_pitch = ((rate_error_pitch-prev_error_pitch)/delta_t)*((float)gain_D_X/2000000);
    float d_cmd_roll = ((rate_error_roll-prev_error_roll)/delta_t)*((float)gain_D_Y/2000000);
    
    prev_error_pitch = rate_error_pitch;
    prev_error_roll = rate_error_roll;
    
    pitch_control = p_cmd_pitch_r + d_cmd_pitch;
    roll_control  = p_cmd_roll_r  + d_cmd_roll;
    
    int throttle = 819 + (t_com + 3276)/7;
    if(throttle < 819)throttle = 819;
    
    motor_1 = throttle + pitch_control + roll_control;
    motor_2 = throttle + pitch_control - roll_control;
    motor_3 = throttle - pitch_control - roll_control;
    motor_4 = throttle - pitch_control + roll_control;
    
    
    pwm_set_all(motor_1,motor_2,motor_3,motor_4);
    //pwm_set_all(throttle,205,205,205 );
    //pwm_set_all(205,throttle,205,205 );
    //pwm_set_all(205,205,throttle,205 );
    //pwm_set_all(819,819,throttle,819 );
    //pwm_set_all(205,205,205,throttle );
}


