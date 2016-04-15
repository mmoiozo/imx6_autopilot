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

 float x_control = 0;
 float y_control = 0;
 float z_control = 0;
 

void PID_stabilisation(double delta_t)
{
    uint16_t motor_1 = 0;//right front
    uint16_t motor_2 = 0;//left front
    uint16_t motor_3 = 0;//left back
    uint16_t motor_4 = 0;//right back
    
    
    
    x_control = command_angle_x;// comp_angle_x - command_angle_x;
    
    int throttle = 205 + (t_com + 3276)/26;
    if(throttle < 205)throttle = 205;
    
    //get_angles(&comp_angle_x, &comp_angle_y, delta_t);
    
    //pwm_set_all(motor_1,motor_2,motor_3,motor_4);
    pwm_set_all(throttle,205,205,205 );
    //pwm_set_all(205,throttle,205,205 );
    //pwm_set_all(205,205,throttle,205 );
    //pwm_set_all(205,205,205,throttle );
}


