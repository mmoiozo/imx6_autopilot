
#include "read_sensors.h" //initializing and reading i2c sensors
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>

#define PI 3.141592654

    int16_t x_acc_raw = 0;
    int16_t y_acc_raw = 0;
    int16_t z_acc_raw = 0;
    int16_t mpu_temp_raw = 0;
    int16_t x_gyro_raw = 0;
    int16_t y_gyro_raw = 0;
    int16_t z_gyro_raw = 0;
    
    float comp_angle_roll = 0;
    float comp_angle_pitch = 0;
    
    float comp_angle_roll_2 = 0;
    float comp_angle_pitch_2 = 0;
    
    float x_angle_vec = 0;
    float y_angle_vec = 0;
    float z_angle_vec = 100;
    float z_acc_comp = 0;
    float z_speed_comp = 0;
    int acc_count = 0;
    float z_acc_av = 0;
    float delta_t = 0;
    

int read_mpu(int16_t *x_acc, int16_t *y_acc, int16_t *z_acc, int16_t *mpu_temp, int16_t *x_rate, int16_t *y_rate,int16_t *z_rate);

void get_angles(double dt)
{
    float angle_vector;
    
    read_mpu(&x_acc_raw, &y_acc_raw, &z_acc_raw, &mpu_temp_raw, &x_gyro_raw, &y_gyro_raw, &z_gyro_raw);
    angle_vector = sqrtf(pow((double)x_acc_raw, 2) + pow((double)y_acc_raw, 2) + pow((double)z_acc_raw, 2));
    
    float x_acc_angle = acosf((float)x_acc_raw / angle_vector) * 57.3;
    float y_acc_angle = acosf((float)y_acc_raw / angle_vector) * 57.3;
    float z_acc_angle = acosf((float)z_acc_raw / angle_vector) * 57.3;
    
    float x_acc_angle_2 = atan2 (x_acc_raw,z_acc_raw) * 57.3;
    float y_acc_angle_2 = atan2 (y_acc_raw,z_acc_raw) * 57.3; 
    //float z_acc_angle_2 = atan2 (y_acc_raw,x_acc_raw) * 57.3;
    
    comp_angle_roll = (0.995 * (comp_angle_roll + (((float)x_gyro_raw/131)*dt))) + (0.005 * x_acc_angle);
    comp_angle_pitch = (0.995 * (comp_angle_pitch + (((float)y_gyro_raw/-131)*dt))) + (0.005 * y_acc_angle);
    
    
    //Compensation angles for z acceleration vector tilt Compensation
    comp_angle_roll_2 = (0.995 * (comp_angle_roll_2 + (((float)x_gyro_raw/-131)*dt))) + (0.005 * x_acc_angle_2);
    comp_angle_pitch_2 = (0.995 * (comp_angle_pitch_2 + (((float)y_gyro_raw/131)*dt))) + (0.005 * y_acc_angle_2);
    
    //tilt compensated z acceleration
    //reverse z equals unit 1
    x_angle_vec = (100.0*tan(comp_angle_roll_2*(PI/180))); 
    y_angle_vec = (100.0*tan(comp_angle_pitch_2*(PI/180)));
    
    //dot product
    float dot = x_acc_raw*x_angle_vec + y_acc_raw*y_angle_vec + z_acc_raw*z_angle_vec;
    float angle_vec = sqrtf(pow((double)x_angle_vec, 2) + pow((double)y_angle_vec, 2) + pow((double)z_angle_vec, 2));
    z_acc_comp = -((((dot/angle_vec)/15700)*9.81)-9.81);//in m/s2//16384
    z_acc_av += z_acc_comp*dt;
    delta_t  += dt;
    
}



