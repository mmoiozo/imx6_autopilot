#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <gst/gst.h>
#include <glib.h>
#include <fcntl.h>		//Used for UART
#include <termios.h>		//Used for UART
#include <errno.h>
#include <sys/types.h>
#include <stdint.h>
#include <linux/i2c-dev.h> //i2c
#include <math.h>//math
#include "read_sensors.h" //initializing and reading i2c sensors
#include "serial_com.h" //initializing uart
#include "AHRS.h"//Complementary filter AHRS
#include "PWM.h"//i2c PWM motor control and init
#include "spi_uart.h"//spi to uart driver
#include "gst_video.h"//
#include "stabilisation.h"//

//#define MPU_ADDR 0x69//b1101001 (pin AD0 is logic high)
//#define BMP_ADDR 0x77// 1110111
//#define HMC_ADDR 0x1E

int loop_status = 1;
void uart_init();
void uart_read();
void uart_write();

int main (int   argc, char **argv[])

{

 int a = 0;
 int last = 0;
 int b = 0;
 int last_20 = 0;
 struct timeval start, stop;
 double last_time = 0;
 double elapsed_long = 0;
 double last_time_20 = 0;
 double elapsed_20 = 0;
 int16_t x_acceleration;
 int16_t y_acceleration;
 int16_t z_acceleration;
 int16_t mpu_temperature;
 int16_t x_gyro_rate;
 int16_t y_gyro_rate;
 int16_t z_gyro_rate;
 int16_t mag_x;
 int16_t mag_y;
 int16_t mag_z;
 
 uint16_t pwm_counter = 205;//250;
 int pwm_count = 0;
 int pwm_direction = 1;
 int gps_count = 0;
 
// usleep(5000000);//five second wait
 
  int state = 0;
  initialize_pipeline(argc,argv);

	//UART initialisation
	//uart_init();
        //UART init non-canonical input
        uart_init_nc();
        usleep(2000000);//one second wait
        close(uart0_filestream);
        usleep(1000000);//one second wait
        uart_init_nc();//reopen
        usleep(1000000);//one second wait
        
        
	//i2c intialisation
	int x=0;
 	initialize_sensors(&x); /* function call*/
 	printf("Sensors connected: %d \n",x);
        x=0;
        //init pca servo driver and set 
        pca_init(&x);
        //pwm_set(0,pwm_counter);//port0 205 for 1 ms (0/4095) with 4095  308 for center pos 410 max
        pwm_set_all(pwm_counter,pwm_counter,pwm_counter,pwm_counter);
        printf("PWM set 205\n");
        
        //init spi_uart
        init_spi(&x);
        
        //file print test
       
       FILE *fp;
   
        /* open the file */
        fp = fopen("log.txt", "a");
        if (fp == NULL) {
            printf("I couldn't open results.dat for appending.\n");
        }
    
        /* write to the file */
        fprintf(fp, "--------------------------------------------------------------\n");
        /* close the file */
        fclose(fp);
        
        //WIFI check esp8266
        //esp8266_init();
        usleep(1000000);//three second wait
        send_string("AT+RST\r\n");
        usleep(1000000);
        uart_read_nc();
        send_string("AT+CWMODE?\r\n");
        usleep(10000);
        uart_read_nc();
        send_string("AT+CIFSR\r\n");
        usleep(10000);
        uart_read_nc();
        send_string("AT+CIPMUX=1\r\n");//multiple connection mode in order to enable server mode
        usleep(10000);
        uart_read_nc();
        send_string("AT+CIPSERVER=1,80\r\n");
        usleep(100000);

        //init sensors
       init_bmp();
       hmc_init();
       init_sc16();
       
       
 gettimeofday(&start, 0);
 double prev_time = start.tv_sec + (double)(start.tv_usec / 1000000.0);

 double elapsed_time_20 = 0;
 float loop_rate = 0;
 char recv = 0;
 double alt = 0;
 
 while(loop_status == 1)
   {
      
      a++;
	gettimeofday(&stop, 0);
	double curr_time =  (double)(stop.tv_sec + stop.tv_usec/1000000.0);
        double elapsed = curr_time - (double)(start.tv_sec + start.tv_usec/1000000.0);
	gettimeofday(&start, 0);
	elapsed_long = curr_time - last_time;
        elapsed_20 = curr_time - last_time_20;

        //20 HZ loop
        if(elapsed_20 > 0.03)
	{
            last_time_20 = (double)(start.tv_sec + start.tv_usec/1000000.0);
            elapsed_time_20 = elapsed_20;
            uart_read_nc(&recv);// read to empty buffer
            send_string("+IPD\r\n");
            usleep(2000);
            uart_read_nc(&recv);//try to remove while 
            if(recv == 1)
            {
                //esp8266_send(4);
                
                int16_t x_angle_d = (comp_angle_pitch * 10)-900;
                int16_t y_angle_d = (comp_angle_roll * 10)-900;
                int16_t altitude = (int16_t)(alt);
                int16_t refresh = loop_rate;
                int16_t connected = recv;
                
                debug_send(x_angle_d,y_angle_d,altitude,refresh,connected);
                recv = 0;
                b++;
            }
            else if(gain_recv == 1)
            {
                printf("send x_p: %d x_i: %d x_d: %d y_p: %d\n",gain_P_X,gain_i_X,gain_D_X,gain_P_Y);
                gain_send();
                gain_recv = 0;
            }
            
        
        }

	if(elapsed_long > 0.5)
	{
	loop_rate = (a - last)/elapsed_long;
	last = a;
        float loop_rate_20 = (b - last_20)/elapsed_long;
	last_20 = b;
        
	last_time = (double)(start.tv_sec + start.tv_usec/1000000.0);

        //check closing i2c dev
        //close(fd_i2c);
	//UART Send
        
        //send_string("+IPD\r\n");
        //usleep(1000);
        ////esp8266_send(4);
	////uart_send();	
	//UART Read
	//uart_read_nc();
        ////fd_i2c = open("/dev/i2c-2", O_RDWR);
        long temp = 0;
        long press = 0;
        double po = 102200;//de bilt //101325;//icao
        bmp_get(&temp,&press);
        alt = 44330*( 1-pow(((double)press/po),0.19029));   
        hmc_read(&mag_x,&mag_y,&mag_z);
        
        int16_t x_angle_16 =(comp_angle_roll * 10)-900;
        unsigned char first = x_angle_16 & 0xFF;
        unsigned char second = x_angle_16 >> 8;
        uint16_t reconstructed = (second << 8) | first;//(second << 8) | first;
        int16_t original =(int16_t)(reconstructed);// (int16_t)((second << 8) | first);
        //printf("x_angle_16: %d first: %d second: %d original: %d reconstructed: %d\n",x_angle_16,first,second,original,reconstructed);
        
        /*
        //print sensor values:
        printf("Gyro rate X: %d Y: %d Z: %d\n",x_gyro_rate,y_gyro_rate,z_gyro_rate);
	printf("Accelerometer acceleration X: %d Y: %d Z: %d\n",x_acceleration,y_acceleration,z_acceleration);
        printf("Complementary filter angle X: %f Y: %f\n",comp_angle_x,comp_angle_y);
        printf("Elapsed long: %f Loop rate HZ: %f current time %f \n", elapsed_long,loop_rate , curr_time);
        printf("x mag: %d y mag: %d z mag: %d \n",mag_x,mag_y,mag_z);
        */
        //printf("u_temp: %d u_press: %d Altitude: %f \n",temp,press,alt);
        //printf("pwm_counter: %d pwm_direction: %d pwm_count: %d\n",pwm_counter,pwm_direction,pwm_count);
        printf("Elapsed long: %f Loop rate HZ: %f current time %f \n", elapsed_time_20,loop_rate_20 , curr_time);
        //printf("Angle Pitch: %f Roll: %f Pitch control: %f Roll control: %f\n",comp_angle_pitch,comp_angle_roll,pitch_control,roll_control);
        //printf("Angle Pitch: %f Roll: %f Pitch control: %d Roll control: %d\n",comp_angle_pitch,comp_angle_roll,y_com,x_com);
        
        
        fp = fopen("log.txt", "a");
          /* write to the file */
          fprintf(fp,"Elapsed long: %f Loop rate com HZ: %f Loop rate Main: %f current time %f \n", elapsed_time_20,loop_rate_20,loop_rate,curr_time);
          /* close the file */
          fclose(fp);
        
	}
	
	//START MAIN LOOP CODE
        
        //read_mpu(&x_acceleration, &y_acceleration, &z_acceleration, &mpu_temperature, &x_gyro_rate, &y_gyro_rate, &z_gyro_rate);
        get_angles(elapsed);//get_angles(&comp_angle_x, &comp_angle_y, elapsed);
        PID_stabilisation(elapsed);
        //pwm_set_all(pwm_counter,pwm_counter,pwm_counter,pwm_counter);
        
        if(gps_count > 12)
        {
            sc16_read();
            gps_count = 0;
        }
        else
        {
            gps_count +=1;
        }
        
        
	//END MAIN LOOP CODE

          // See if we have pending messages on the bus and handle them
        while ((msg = gst_bus_pop (g_bus))) 
	{
          // Call your bus message handler
          bus_call (g_bus, msg);
          gst_message_unref (msg); 
	}
      //usleep(1000);
     
   }

 /*Out of the main loop, clean up nicely*/
 g_print ("Returned, stopping playback\n");
 gst_element_set_state (pipeline, GST_STATE_NULL);
 g_print ("Deleting pipeline\n");
 gst_object_unref (GST_OBJECT (pipeline));
 gst_object_unref (g_bus);


 return 0;
}
