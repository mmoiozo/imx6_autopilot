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
    
    float command_angle_pitch = (y_com/100)+90;
    float command_angle_roll = (-x_com/100)+90;
    
    pitch_control = command_angle_pitch;
    roll_control = command_angle_roll;// comp_angle_x - command_angle_x;
    
    float error_pitch = comp_angle_pitch - command_angle_pitch;
    float error_roll = comp_angle_roll - command_angle_roll;
    
    float p_cmd_pitch = error_pitch*(gain_P_X/200);
    float p_cmd_roll = error_roll*(gain_P_Y/200);
    
    i_cmd_pitch += error_pitch*(gain_i_X/200)*delta_t;
    i_cmd_roll += error_roll*(gain_i_Y/200)*delta_t;
    
    float d_cmd_pitch = ((error_pitch-prev_error_pitch)/delta_t)*(gain_D_X/200);
    float d_cmd_roll = ((error_roll-prev_error_roll)/delta_t)*(gain_D_Y/200);
    
    prev_error_pitch = error_pitch;
    prev_error_roll = error_roll;
    
    pitch_control = p_cmd_pitch + i_cmd_pitch + d_cmd_pitch;
    roll_control = p_cmd_roll + i_cmd_roll + d_cmd_roll;
    
    int throttle = 205 + (t_com + 3276)/26;
    if(throttle < 205)throttle = 205;
    
    motor_1 = throttle + pitch_control + roll_control;
    motor_2 = throttle + pitch_control - roll_control;
    motor_3 = throttle - pitch_control - roll_control;
    motor_3 = throttle - pitch_control + roll_control;
    
    //get_angles(&comp_angle_x, &comp_angle_y, delta_t);
    
    //pwm_set_all(motor_1,motor_2,motor_3,motor_4);
    pwm_set_all(throttle,205,205,205 );
    //pwm_set_all(205,throttle,205,205 );
    //pwm_set_all(205,205,throttle,205 );
    //pwm_set_all(205,205,205,throttle );
}


