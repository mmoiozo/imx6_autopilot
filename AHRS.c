
#include "read_sensors.h" //initializing and reading i2c sensors
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>

    int16_t x_acc_raw = 0;
    int16_t y_acc_raw = 0;
    int16_t z_acc_raw = 0;
    int16_t mpu_temp_raw = 0;
    int16_t x_gyro_raw = 0;
    int16_t y_gyro_raw = 0;
    int16_t z_gyro_raw = 0;
    
    float comp_angle_roll = 0;
    float comp_angle_pitch = 0;

int read_mpu(int16_t *x_acc, int16_t *y_acc, int16_t *z_acc, int16_t *mpu_temp, int16_t *x_rate, int16_t *y_rate,int16_t *z_rate);

void get_angles(double dt)
{
    float angle_vector;
    
    read_mpu(&x_acc_raw, &y_acc_raw, &z_acc_raw, &mpu_temp_raw, &x_gyro_raw, &y_gyro_raw, &z_gyro_raw);
    angle_vector = sqrtf(pow((double)x_acc_raw, 2) + pow((double)y_acc_raw, 2) + pow((double)z_acc_raw, 2));
    
    float x_acc_angle = acosf((float)x_acc_raw / angle_vector) * 57.3;
    float y_acc_angle = acosf((float)y_acc_raw / angle_vector) * 57.3;
    float z_acc_angle = acosf((float)z_acc_raw / angle_vector) * 57.3;
    
    comp_angle_roll = (0.995 * (comp_angle_roll + (((float)x_gyro_raw/131)*dt))) + (0.005 * x_acc_angle);
    comp_angle_pitch = (0.995 * (comp_angle_pitch + (((float)y_gyro_raw/-131)*dt))) + (0.005 * y_acc_angle);
    
}