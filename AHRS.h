#include <sys/types.h>

extern void get_angles(double dt);

extern int16_t x_acc_raw;
extern int16_t y_acc_raw;
extern int16_t z_acc_raw;
extern int16_t mpu_temp_raw;
extern int16_t x_gyro_raw;
extern int16_t y_gyro_raw;
extern int16_t z_gyro_raw;

extern float comp_angle_roll;
extern float comp_angle_pitch;

extern double alt;
