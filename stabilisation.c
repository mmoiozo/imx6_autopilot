#include "read_sensors.h" //initializing and reading i2c sensors
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <math.h>
#include "serial_com.h" //uart
#include "AHRS.h"//Complementary filter AHRS
#include "PWM.h"//i2c PWM motor control 

//Stabilisation defines
//#define PITCH_TRIM 6
//#define ROLL_TRIM 10

//#define PITCH_I_TRIM 6
//#define ROLL_I_TRIM 10

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
 
 uint8_t pitch_trim = 90;
 uint8_t roll_trim = 90;

 
 float prev_error_pitch = 0;
 float prev_error_roll = 0;
 
 float i_cmd_pitch = 0;
 float i_cmd_roll = 0;
 
 //exern for verification
 float pitch_control_rate = 0;
 float roll_control_rate =  0;
 
 float pitch_control = 0;
 float roll_control = 0;
 float yaw_control = 0;

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
    uint16_t motor_1 = 0;//right front// anti clockwise 
    uint16_t motor_2 = 0;//left front // clockwise
    uint16_t motor_3 = 0;//left back  // anti clockwise
    uint16_t motor_4 = 0;//right back // clockwise
    
    
    float command_angle_pitch = (((float)y_com)/100)+pitch_trim;//+90+PITCH_TRIM;//6;//was +8
    //float command_angle_roll = (((float)x_com)/100) +90+10 ;//was -10
    float command_angle_roll = (((float)-x_com)/100)+roll_trim;//+90+ROLL_TRIM;//10 ;
    
    float command_rate_pitch = (((float)y_com)*2);//no scaling 30 deg/sec is 30*130=3900 lsb
    float command_rate_roll = (((float)x_com)*2);//
    float command_rate_yaw = (((float)r_com)*3);//70 -50 deg/sec //try 4times
    
    //ATTITUDE LOOP
    
    float error_pitch = comp_angle_pitch - command_angle_pitch;
    float error_roll = comp_angle_roll - command_angle_roll;
    
    float p_cmd_pitch = error_pitch*((float)gain_P_X_O*2);//was gain_P_X
    float p_cmd_roll = error_roll*((float)gain_P_Y_O*2);
    
    if(t_com > -3200)
    {
    i_cmd_pitch += error_pitch*((float)gain_i_X*10)*delta_t;
    if(i_cmd_pitch > 9000)i_cmd_pitch = 9000;//prevent integral windup was 40
    if(i_cmd_pitch < -9000)i_cmd_pitch = -9000;
    i_cmd_roll += error_roll*((float)gain_i_Y*10)*delta_t;
    if(i_cmd_roll > 9000)i_cmd_roll = 9000;//prevent integral windup was 40
    if(i_cmd_roll < -9000)i_cmd_roll = -9000;
    }
    
    pitch_control_rate = p_cmd_pitch + i_cmd_pitch;
    roll_control_rate =  p_cmd_roll + i_cmd_roll;
    
    //RATE LOOP
    
    //Rate control
    //float rate_error_pitch = (float)(-y_gyro_raw) - command_rate_pitch;
    //float rate_error_roll = (float)x_gyro_raw - command_rate_roll;
    //float rate_error_yaw = (float)z_gyro_raw - command_rate_yaw;
    
    //Outer loop control
    float rate_error_pitch = (float)(-y_gyro_raw) - (-pitch_control_rate);
    float rate_error_roll  = (float)x_gyro_raw    - (-roll_control_rate);
    float rate_error_yaw   = (float)z_gyro_raw    - command_rate_yaw;
    
    float p_cmd_pitch_r = rate_error_pitch*((float)gain_P_X/20000);
    float p_cmd_roll_r = rate_error_roll*((float)gain_P_Y/20000);//was gain_P_Y but is now temporary used by outer loop P 
    float p_cmd_yaw_r = rate_error_yaw*((float)gain_P_Z/20000);
    
    float d_cmd_pitch = ((rate_error_pitch-prev_error_pitch)/delta_t)*((float)gain_D_X/2000000);
    float d_cmd_roll = ((rate_error_roll-prev_error_roll)/delta_t)*((float)gain_D_Y/2000000);
    
    prev_error_pitch = rate_error_pitch;
    prev_error_roll = rate_error_roll;
    
    pitch_control = p_cmd_pitch_r + d_cmd_pitch;
    roll_control  = p_cmd_roll_r  + d_cmd_roll;
    yaw_control   = p_cmd_yaw_r;//later also integral term and absolute heading
    
    if(pitch_control > 45)pitch_control = 45;
    else if(pitch_control < -45)pitch_control = -45;
    if(roll_control > 45)roll_control = 45;
    else if(roll_control < -45)roll_control = -45;
    if(yaw_control > 45)yaw_control = 45;
    else if(yaw_control < -45)yaw_control = -45;
    
    
    //int throttle = 819 + (t_com + 3276)/7;
    int throttle = 819 + (t_com + 3276)/7.6;
    if(throttle < 819)throttle = 819;//throttle minimum limit
    
    
    motor_1 = (uint16_t)(throttle + pitch_control + roll_control + yaw_control);
    motor_2 = (uint16_t)(throttle + pitch_control - roll_control - yaw_control);
    motor_3 = (uint16_t)(throttle - pitch_control - roll_control + yaw_control);
    motor_4 = (uint16_t)(throttle - pitch_control + roll_control - yaw_control);
    
    
    pwm_set_all(motor_1,motor_2,motor_3,motor_4);//write new pwm duty cycles to pwm driver registers
    //pwm_set_all(throttle,819,819,819 );
    //pwm_set_all(819,throttle,819,819 );
    //pwm_set_all(819,819,throttle,819 );
    //pwm_set_all(819,819,throttle,819 );
    //pwm_set_all(819,819,819,throttle );
}


