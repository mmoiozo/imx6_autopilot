#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <stdint.h>
#include <linux/i2c-dev.h> //i2
#include "read_sensors.h" //initializing and reading i2c sensors

#define PCA_ADDR 0x40
#define PCA_MODE1 0x00
#define PCA_PRE_SCALE 0xFE
#define PCA_LED0 6
#define PCA_LED4 22

char pca_buffer[20];

void pca_init(int *pca_success)
{
    if (ioctl(fd_i2c, I2C_SLAVE, PCA_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
	
	//Write write_byte(char address, char value) function 
	
        pca_buffer[0] = PCA_MODE1;
	pca_buffer[1] = 0x01;//reset module
	write(fd_i2c, pca_buffer, 2);
	usleep(500);
	read(fd_i2c, pca_buffer, 1);
        printf("PCA 0x%02X\n", pca_buffer[0]);
        
        usleep(500);
        pca_buffer[0] = PCA_MODE1;
	pca_buffer[1] = 0x10;//go to sleep before setting prescaler
	write(fd_i2c, pca_buffer, 2);
	usleep(500);
	read(fd_i2c, pca_buffer, 1);
        printf("PCA 0x%02X\n", pca_buffer[0]);
        usleep(500);
        
        //Prescaler 
        pca_buffer[0] = PCA_PRE_SCALE;
	//pca_buffer[1] = 0x79;//121;//presacale value 50hz:121 
	pca_buffer[1] = 0x0E;//presacale value 400hz:14 
	//pca_buffer[1] = 0x1E;//presacale value 200hz:30
	write(fd_i2c, pca_buffer, 2);
	usleep(500);
	read(fd_i2c, pca_buffer, 1);
        printf("PCA 0x%02X\n", pca_buffer[0]);
        
        usleep(500);
        pca_buffer[0] = PCA_MODE1;
	pca_buffer[1] = 0x00;//turn off sleep
	write(fd_i2c, pca_buffer, 2);
	usleep(500);
	read(fd_i2c, pca_buffer, 1);
        printf("PCA 0x%02X\n", pca_buffer[0]);
        
        //Normal operation with auto-increment
        usleep(500);
        pca_buffer[0] = PCA_MODE1;
	pca_buffer[1] = 0xA1;//0x04;//Enable auto-increment 
	write(fd_i2c, pca_buffer, 2);
	usleep(500);
	read(fd_i2c, pca_buffer, 1);
        printf("PCA 0x%02X\n", pca_buffer[0]);
        usleep(500);
        
}

int pwm_set(uint8_t port, uint16_t duration)
{
     if (ioctl(fd_i2c, I2C_SLAVE, PCA_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
        if(duration>4095) duration=4095;		// Ensure within bounds
        pca_buffer[0] = PCA_LED0+(4*port);
	pca_buffer[1] = 0;
        pca_buffer[2] = 0;
        pca_buffer[3] = duration; // Send Low Byte
        pca_buffer[4] = (duration>>8); // Send High Byte
        write(fd_i2c, pca_buffer, 5);
        
}

int pwm_set_all(uint16_t duration_1, uint16_t duration_2, uint16_t duration_3, uint16_t duration_4)
{
     if (ioctl(fd_i2c, I2C_SLAVE, PCA_ADDR) < 0) {
		printf("ioctl error: %s\n", strerror(errno));
		return 1;
	}
	
	duration_1 *=2;		// Multiply with two to compensate for increase of pwm freq from 200 to 400hz
        duration_2 *=2;
        duration_3 *=2;
        duration_4 *=2;
	
        if(duration_1>4095) duration_1=4095;		// Ensure within bounds
        if(duration_2>4095) duration_2=4095;
        if(duration_3>4095) duration_3=4095;
        if(duration_4>4095) duration_4=4095;
        pca_buffer[0] = PCA_LED0;
	pca_buffer[1] = 0;
        pca_buffer[2] = 0;
        pca_buffer[3] = duration_1; // Send Low Byte
        pca_buffer[4] = (duration_1>>8); // Send High Byte
	pca_buffer[5] = 0;
        pca_buffer[6] = 0;
        pca_buffer[7] = duration_2; // Send Low Byte
        pca_buffer[8] = (duration_2>>8); // Send High Byte
	pca_buffer[9] = 0;
        pca_buffer[10] = 0;
        pca_buffer[11] = duration_3; // Send Low Byte
        pca_buffer[12] = (duration_3>>8); // Send High Byte
	pca_buffer[13] = 0;
        pca_buffer[14] = 0;
        pca_buffer[15] = duration_4; // Send Low Byte
        pca_buffer[16] = (duration_4>>8); // Send High Byte
        write(fd_i2c, pca_buffer, 17);
        
}



