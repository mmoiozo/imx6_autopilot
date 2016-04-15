
#include <sys/types.h>

//extern void initialize_sensors(int *all_connected);
void initialize_sensors(int *all_connected);//function prototype
int read_mpu(int16_t *x_acc, int16_t *y_acc, int16_t *z_acc, int16_t *mpu_temp, int16_t *x_rate, int16_t *y_rate,int16_t *z_rate);
extern int fd_i2c;
void init_bmp();
void init_sc16();
void sc16_read();
