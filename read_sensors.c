#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h> //i2c
#include <fcntl.h>		//Used for UART
#include <termios.h>
#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include "main_ap_loop.h" 
#include "AHRS.h"
#include "serial_com.h"
#include "stabilisation.h"

#define MPU_ADDR 0x69//b1101001 (pin AD0 is logic high)
#define BMP_ADDR 0x77// 1110111
#define HMC_ADDR 0x1E
#define SC16_ADDR 0x4C// 0x90 (1001 000X)  (7bit)->0x48 // a0:vdd a1: vss 0x98 (7 bit)-> 0x4C 

char buffer[2];
char gyro_buffer[14];
int fd_i2c;

//BMP180 

short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
long b3;
unsigned long b4;
long b5;
long b6;
unsigned long b7;
short mb;
short mc;
short md;
long ut;   //uncompensated temperature
long up;   //uncompensated pressure
long x1;
long x2;
long x3;
long t;   //temperature
long p;   //pressure
short oss = 3;//oversampling setting 3,max 40hz 0.25m rms noise
double po = 10132.5;//icao standard atmosphere
char bmp_count = 0;

//SC16IS750
// See section 8.4 of the datasheet for definitions
// of bits in the Line Control Register (LCR)
#define LCR_ENABLE_DIVISOR_LATCH 1 << 7
#define LCR_DATA_FORMAT 0x03

uint8_t  read_byte = 0x80;
uint8_t  SPR = 0x07<<3;
uint8_t  LCR = 0x03<<3;
uint8_t  LSR = 0x05<<3;
uint8_t  TXLVL = 0x08<<3;
uint8_t  RXLVL = 0x09<<3;
uint8_t  DLL = 0x00<<3;
uint8_t  DLH = 0x01<<3;
uint8_t  FCR = 0x02<<3;
uint8_t  RHR = 0x00<<3;


//Data logging
FILE *fp;
struct timeval start_time;
double log_start_time = 0;
int     log_count = 0;
int     log_count_array[150];
float   log_time       [150];
float   log_elapsed    [150];
float   log_com_rate   [150];
int16_t log_gyro_x     [150];
int16_t log_gyro_y     [150];
int16_t log_gyro_z     [150];
int16_t log_acc_x      [150];
int16_t log_acc_y      [150];
int16_t log_acc_z      [150];
float   log_pitch      [150];
float   log_roll       [150];
float   log_alt        [150];
int16_t log_com_x      [150];
int16_t log_com_y      [150];
int16_t log_com_t      [150];
float   log_rc_pitch   [150];
float   log_rc_roll    [150];
float   log_pitch_control   [150];
float   log_roll_control   [150];

//logging cooldown
int cooldown_count = 0;
int start_cooldown = 0;

int initialize_sensors(int *all_connected)//void
{
	//i2c intialisation
	*all_connected = 0;
	fd_i2c = open("/dev/i2c-2", O_RDWR);

	if (fd_i2c < 0) {
		printf("Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd_i2c, I2C_SLAVE, MPU_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	buffer[0]=117;
	write(fd_i2c, buffer, 1);
	
	read(fd_i2c, buffer, 1);
	if(buffer[0] == 0x68){
	*all_connected += 1;
	printf("MPU 6050 IMU connected 0x%02X\n", buffer[0]);
	}
	if (ioctl(fd_i2c, I2C_SLAVE, MPU_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	gyro_buffer[0]=0x6B;
	gyro_buffer[1]=0x00;
	write(fd_i2c, gyro_buffer, 2);


	if (ioctl(fd_i2c, I2C_SLAVE, BMP_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	buffer[0]=0xD0;
	write(fd_i2c, buffer, 1);
	
	read(fd_i2c, buffer, 1);
	if(buffer[0] == 0x55){
	*all_connected += 1;
	printf("BMP 180 pressure sensor connected 0x%02X\n", buffer[0]);
	}

	if (ioctl(fd_i2c, I2C_SLAVE, HMC_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	buffer[0]=10;//idenification regiser A 
	write(fd_i2c, buffer, 1);
	
	read(fd_i2c, buffer, 1);
	if(buffer[0] == 0x48){//ansci value h = 01001000 0x48
	*all_connected += 1;
	printf("HMC 5883l sensor connected 0x%02X\n", buffer[0]);
	}
	
	
	if (ioctl(fd_i2c, I2C_SLAVE, SC16_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
	

	buffer[0]=(read_byte | SPR);
        buffer[1]= 0x12;
	write(fd_i2c, buffer, 2);
	
	read(fd_i2c, buffer, 1);
	if(buffer[0] == 0x12){//ansci value h = 01001000 0x48
	*all_connected += 1;
	printf("SC16IS750 connected 0x%02X\n", buffer[0]);
	}
	


}
int read_mpu(int16_t *x_acc, int16_t *y_acc, int16_t *z_acc, int16_t *mpu_temp, int16_t *x_rate, int16_t *y_rate,int16_t *z_rate)
//int read_mpu(int *x_acc, int *y_acc, int *z_acc, int *mpu_temp, int *x_rate, int *y_rate, int *z_rate)
{
    if (ioctl(fd_i2c, I2C_SLAVE, MPU_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		
		fp = fopen("log.txt", "a");
		fprintf(fp,"MPU ioctl error: %s\n", strerror(errno));
		fclose(fp);
		
		return 1;
	}

	gyro_buffer[0]=0x3B;
	write(fd_i2c, gyro_buffer, 1);
	
	read(fd_i2c, gyro_buffer, 14);
	//int x_rate = gyro_buffer[1] + (gyro_buffer[0] >> 8);
	*x_acc = (((int16_t)gyro_buffer[0]) << 8) | gyro_buffer[1];
	*y_acc = (((int16_t)gyro_buffer[2]) << 8) | gyro_buffer[3];
	*z_acc = (((int16_t)gyro_buffer[4]) << 8) | gyro_buffer[5];
	*mpu_temp = (((int16_t)gyro_buffer[6]) << 8) | gyro_buffer[7];
	*y_rate = (((int16_t)gyro_buffer[8]) << 8) | gyro_buffer[9];
	*x_rate = (((int16_t)gyro_buffer[10]) << 8) | gyro_buffer[11];
	*z_rate = (((int16_t)gyro_buffer[12]) << 8) | gyro_buffer[13];
}

int init_bmp()
{
    char calib_buffer[22];
    
    if (ioctl(fd_i2c, I2C_SLAVE, BMP_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
	calib_buffer[0]=0xAA;
	write(fd_i2c, calib_buffer, 1);
	
	read(fd_i2c, calib_buffer, 22);
        
        ac1 = (calib_buffer[0] << 8) | calib_buffer[1];
        ac2 = (calib_buffer[2] << 8) | calib_buffer[3];
        ac3 = (calib_buffer[4] << 8) | calib_buffer[5];
        ac4 = (calib_buffer[6] << 8) | calib_buffer[7];
        ac5 = (calib_buffer[8] << 8) | calib_buffer[9];
        ac6 = (calib_buffer[10] << 8) | calib_buffer[11];
        b1  = (calib_buffer[12] << 8) | calib_buffer[13];
        b2  = (calib_buffer[14] << 8) | calib_buffer[15];
        mb  = (calib_buffer[16] << 8) | calib_buffer[17];
        mc  = (calib_buffer[18] << 8) | calib_buffer[19];
        md  = (calib_buffer[20] << 8) | calib_buffer[21];
        
        printf("BMP180 eeprom calibration: ac1:  %d ac2: %d ac3: %d ac4: %d ac5: %d ac6: %d b1: %d b2: %d mb: %d mc: %d md: %d\n",ac1,ac2,ac3,ac4,ac5,ac6,b1,b2,mb,mc,md);
        
    
}

int bmp_get(long *temp, long *pressure)
{
    
    char bmp_buffer[10];
    if (ioctl(fd_i2c, I2C_SLAVE, BMP_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		
		fp = fopen("log.txt", "a");
		fprintf(fp,"BMP ioctl error: %s\n", strerror(errno));
		fclose(fp);
		
		return 1;
	}
    
    if(bmp_count == 0)
    {
     //request uncompensated temp reading
     bmp_buffer[0] = 0xF4;
     bmp_buffer[1] = 0x2E;
     write(fd_i2c, bmp_buffer, 2);
     usleep(4500);//wait for temperature reading
     
     //read uncompensated temperature
     bmp_buffer[0] = 0xF6;
     write(fd_i2c, bmp_buffer, 1);
     read(fd_i2c, bmp_buffer, 2);
     
     ut = (bmp_buffer[0] << 8) | bmp_buffer[1];
     
     //request uncompensated pressure
     bmp_buffer[0] = 0xF4;
     bmp_buffer[1] = 0x34 + (oss << 6);;
     write(fd_i2c, bmp_buffer, 2);
     
        bmp_count = 1;
    }
    else
    {
     //request uncompensated temp reading
     bmp_buffer[0] = 0xF6;
     write(fd_i2c, bmp_buffer, 1);
     read(fd_i2c, bmp_buffer, 3);
     up = ((bmp_buffer[0] << 16) | (bmp_buffer[1] << 8) | bmp_buffer[2]) >> (8 - oss);
     
      //calculate the temperature
       x1 = (ut - ac6) * ac5 >> 15;

       x2 = ((long) mc << 11) / (x1 + md);

       b5 = x1 + x2;

       *temp = (b5 + 8) >> 4;
       
       //calculate the pressure

    b6 = b5 - 4000;

    x1 = (b2 * (b6 * b6 >> 12)) >> 11;

    x2 = ac2 * b6 >> 11;

    x3 = x1 + x2;

    b3 = (((long) ac1 * 4 + x3) << oss) >> 2;

    x1 = ac3 * b6 >> 13;

    x2 = (b1 * (b6 * b6 >> 12)) >> 16;

    x3 = ((x1 + x2) + 2) >> 2;

    b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;

    b7 = ((unsigned long) up - b3) * (50000 >> oss);

    *pressure = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    
    x1 = (*pressure >> 8) * (*pressure >> 8);

    x1 = (x1 * 3038) >> 16;

    x2 = (-7357 * *pressure) >> 16;

    *pressure = *pressure + ((x1 + x2 + 3791) >> 4);
        
        bmp_count = 0;
    }
}

int hmc_init()
{
    if (ioctl(fd_i2c, I2C_SLAVE, HMC_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	buffer[0]=2;//mode register
	buffer[1]=0;//clear mode register for continous measurement noise
	write(fd_i2c, buffer, 2);
}

int hmc_read(int16_t *x_mag, int16_t *y_mag, int16_t *z_mag)
{
    char hmc_buffer[10];
    
    if (ioctl(fd_i2c, I2C_SLAVE, HMC_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		
		fp = fopen("log.txt", "a");
		fprintf(fp,"HMC ioctl error: %s\n", strerror(errno));
		fclose(fp);
		
		return 1;
	}
	
	hmc_buffer[0]=3;//X MSB register
	write(fd_i2c, hmc_buffer, 1);
        
        read(fd_i2c, hmc_buffer, 6);
        
	*x_mag = (((int16_t)hmc_buffer[0]) << 8) | hmc_buffer[1];
	*y_mag = (((int16_t)hmc_buffer[2]) << 8) | hmc_buffer[3];
	*z_mag = (((int16_t)hmc_buffer[4]) << 8) | hmc_buffer[5];
        
        
    
}

int init_sc16()
{
    char sc16_buffer[65];
    
    if (ioctl(fd_i2c, I2C_SLAVE, SC16_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
	
	unsigned long divisor = ((14745600/1)/(9600*16));
        
        sc16_buffer[0]= (read_byte | LCR);
	write(fd_i2c, sc16_buffer, 1);
	
	read(fd_i2c, sc16_buffer, 1);
	printf("SC16IS750 LCR: 0x%02X\n", sc16_buffer[0]);
        char temp = sc16_buffer[0];
	sc16_buffer[0]= LCR;
        sc16_buffer[1]= (temp | LCR_ENABLE_DIVISOR_LATCH);
	write(fd_i2c, sc16_buffer, 2);
        
        
        sc16_buffer[0]= DLL;
        sc16_buffer[1]= divisor & 0xFF;
	write(fd_i2c, sc16_buffer, 2);
        
        //sc16_buffer[2]= (divisor>>8) & 0xFF;
        //printf("SC16IS750 divisor: 0x%02X\n", sc16_buffer[1]);
        
        sc16_buffer[0]= DLH;
        sc16_buffer[1]= (divisor>>8) & 0xFF;
	write(fd_i2c, sc16_buffer, 2);
        
        sc16_buffer[0]= (read_byte | DLH);
	write(fd_i2c, sc16_buffer, 1);
	
	read(fd_i2c, sc16_buffer, 1);
	printf("SC16IS750 DLL: 0x%02X\n", sc16_buffer[0]);
        
        /*
        sc16_buffer[0]= LCR;
        sc16_buffer[1]= LCR_ENABLE_DIVISOR_LATCH;
	write(fd_i2c, sc16_buffer, 2);
        */ 
        
	// 8 data bit, 1 stop bit, no parity
	sc16_buffer[0]= LCR;
        sc16_buffer[1]= 0x00;
	write(fd_i2c, sc16_buffer, 2);
        sc16_buffer[0]= LCR;
        sc16_buffer[1]= LCR_DATA_FORMAT;
	write(fd_i2c, sc16_buffer, 2);
        
        // reset TXFIFO, reset RXFIFO, non FIFO mode
        sc16_buffer[0]= FCR;
        sc16_buffer[1]= 0x06;
        write(fd_i2c, sc16_buffer, 2);
        
        sc16_buffer[0]= FCR;
        sc16_buffer[1]= 0x01;
        write(fd_i2c, sc16_buffer, 2);
        
	sc16_buffer[0]= (read_byte | FCR);
	write(fd_i2c, sc16_buffer, 1);
	
	read(fd_i2c, sc16_buffer, 1);
	printf("SC16IS750 init: 0x%02X\n", sc16_buffer[0]);
	
}

int sc16_read()
{
    char sc16_buffer[65];
    
    if (ioctl(fd_i2c, I2C_SLAVE, SC16_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		
		fp = fopen("log.txt", "a");
		fprintf(fp,"SC16 ioctl error: %s\n", strerror(errno));
		fclose(fp);
		
		return 1;
	}
	
	sc16_buffer[0]= (read_byte | RXLVL);
	write(fd_i2c, sc16_buffer, 1);
	
	read(fd_i2c, sc16_buffer, 1);
        char read_count = sc16_buffer[0];
	
        if(read_count > 0)
        {
        //printf("SC16IS750 read count: %d\n", read_count);
	sc16_buffer[0]= (read_byte | RHR);
	write(fd_i2c, sc16_buffer, 1);
	
	read(fd_i2c, sc16_buffer, read_count);
        sc16_buffer[read_count] = '\0';
        
        //printf("%s\n", sc16_buffer);
        }
	//printf("SC16IS750 read: 0x%02X\n", sc16_buffer[0]);
        //printf("SC16IS750 read: %c %c\n", sc16_buffer[0],sc16_buffer[1]);
        
	
}

int mpu_init()
{
    if (ioctl(fd_i2c, I2C_SLAVE, MPU_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}

	buffer[0]=0x1A;//configuration register
	buffer[1]=0x06;//2=98hz 3=42hz 4=20hz 5=10hz 6=5hz
	write(fd_i2c, buffer, 2);
}

int log_data(double delta_t, double current_time,float com_rate)
{
    if(flight_status == 0 && t_com > -3200)
    {
        log_count = 0;
        
        log_start_time = current_time;
        
        flight_status = 1;
	
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
        
        fp = fopen("log.txt", "a");
        fprintf(fp, "START LOGGING %s\n",asctime(tm));
        fclose(fp);
    }
    else if(flight_status == 1 && t_com < -3200)
    {
	
	start_cooldown = 1;//start 5 sec cooldown counter
	
    }
    
    if(cooldown_count > 10)//after 5 sec cooldown stop logging
    {
       flight_status = 0;
       cooldown_count = 0;
       start_cooldown = 0;
       fp = fopen("log.txt", "a");
       fprintf(fp, "STOP LOGGING___________________________________________________________________________________\n");
       fclose(fp); 
    }
    
    if(start_cooldown == 1 && t_com > -3200) //if throttle is increased during cooldown stop counting and reset cooldown counter
    {
      cooldown_count = 0;
      start_cooldown = 0;
    }
      
    
    if(flight_status == 1 && log_count < 149)
    {
    
        log_count_array[log_count] = log_count;
        log_time       [log_count] = (float)(current_time - log_start_time);
        log_elapsed    [log_count] = (float)delta_t;
        log_com_rate   [log_count] = (float)com_rate;
        log_gyro_x     [log_count] = x_gyro_raw;
        log_gyro_y     [log_count] = y_gyro_raw;
        log_gyro_z     [log_count] = z_gyro_raw;
        log_acc_x      [log_count] = x_acc_raw;
        log_acc_y      [log_count] = y_acc_raw;
        log_acc_z      [log_count] = z_acc_raw;
        log_pitch      [log_count] = comp_angle_pitch;
        log_roll       [log_count] = comp_angle_roll;
        log_alt        [log_count] = alt;
        log_com_x      [log_count] = x_com;
        log_com_y      [log_count] = y_com;
        log_com_t      [log_count] = t_com;
        log_rc_pitch   [log_count] = pitch_control_rate;
        log_rc_roll    [log_count] = roll_control_rate;
        log_pitch_control   [log_count] = pitch_control;
        log_roll_control    [log_count] = roll_control;
    
        log_count +=1;

    }
    
}


void write_log()
{
    if(flight_status == 1)
    {
        for(int i = 0;i < log_count;i++)
        {
            
        fp = fopen("log.txt", "a");
        fprintf(fp,"%d %f %f %f %d %d %d %d %d %d %f %f %f %d %d %d %f %f %f %f \n",log_count_array[i],log_time[i],log_elapsed[i],log_com_rate[i],log_gyro_x[i],log_gyro_y[i]
                ,log_gyro_z[i],log_acc_x[i],log_acc_y[i],log_acc_z[i],log_pitch[i],log_roll[i],log_alt[i],log_com_x[i],log_com_y[i]
                ,log_com_t[i], log_rc_pitch[i],log_rc_roll[i],log_pitch_control[i],log_roll_control[i]);
        fclose(fp);
            
        }
        log_count = 0;
    }
    
    if(start_cooldown == 1) cooldown_count+=1;
}