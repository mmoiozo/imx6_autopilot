
#include <sys/types.h>

//extern void initialize_sensors(int *all_connected);
void initialize_sensors(int *all_connected);//function prototype
int read_mpu(int16_t *x_acc, int16_t *y_acc, int16_t *z_acc, int16_t *mpu_temp, int16_t *x_rate, int16_t *y_rate,int16_t *z_rate);
extern int fd_i2c;
void init_bmp();
void init_sc16();
void mpu_init();
int bmp_get(long *temp, long *pressure);
int hmc_read(int16_t *x_mag, int16_t *y_mag, int16_t *z_mag);
int sc16_read();

void log_data(double delta_t, double current_time,float com_rate);
void write_log();