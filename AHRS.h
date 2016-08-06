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

extern float comp_angle_roll_2;
extern float comp_angle_pitch_2;

extern float x_angle_vec;
extern float y_angle_vec;
extern float z_angle_vec;
extern float z_acc_comp;
extern float z_speed_comp;
extern float z_acc_av;
extern float delta_t;


