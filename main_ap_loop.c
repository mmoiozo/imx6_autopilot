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
#include "safety.h"//wait for throttle and link lost functions

//#define MPU_ADDR 0x69//b1101001 (pin AD0 is logic high)
//#define BMP_ADDR 0x77// 1110111
//#define HMC_ADDR 0x1E
#define LOG_WRITE 0//Write data to log with chance on +700ms lag. (if zero max lag 23ms)

int loop_status = 1;
void uart_init();
void uart_read();
void uart_write();

int main (int argc, char **argv)//[]

{

 int a = 0;
 int last = 0;
 int b = 0;
 int last_20 = 0;
 float loop_rate_20 = 0;
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
 
 int flight_status = 0;//0 for motors off 1 for motors on (in future 2 for real flight? > 1m)
 
 //uint16_t pwm_counter = 205;//250;
 uint16_t pwm_counter = 819;//250; 200 hz
 int pwm_count = 0;
 int pwm_direction = 1;
 int gps_count = 0;
 
// usleep(5000000);//five second wait
 
  int state = 0;
  

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
        //init_spi(&x);
        
        //file print test
       
       FILE *fp;
   
        /* open the file */
        fp = fopen("log.txt", "a");
        if (fp == NULL) {
            //printf("I couldn't open results.dat for appending.\n");
        }
    
    
	//get system time
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	//printf("%s\n", asctime(tm));
        /* write to the file */
        fprintf(fp, "STARTING NEW FLIGHT At: %s\n", asctime(tm));
        /* close the file */
        fclose(fp);
        
        //WIFI check esp8266
        //esp8266_init();
        usleep(1000000);//three second wait
        send_string("AT+RST\r\n");
        usleep(1000000);
        uart_read_simple();
        send_string("AT+CWMODE?\r\n");
        usleep(10000);
        //uart_read_simple();
        send_string("AT+CIFSR\r\n");
        usleep(10000);
        //uart_read_simple();
        send_string("AT+CIPMUX=1\r\n");//multiple connection mode in order to enable server mode
        usleep(10000);
        //uart_read_simple();
        send_string("AT+CIPSERVER=1,80\r\n");
        usleep(100000);
	send_string("AT+RFPOWER=82\r\n");
	usleep(1000000);
	uart_read_simple();

        //init sensors
       init_bmp();
       hmc_init();
       init_sc16();
       mpu_init();
       
       
 gettimeofday(&start, 0);
 double prev_time = start.tv_sec + (double)(start.tv_usec / 1000000.0);

 double elapsed_time_20 = 0;
 float loop_rate = 0;
 char recv = 0;
 //double alt = 0;
 
 //Start named pipe to gstreamer process
 init_gst_pipe();
 
 //SAFETY WAIT FOR THROTTLE SIGNAL AT LOWEST POSITION
 wait_signal();
 
   //START GSTREAMER PIPELINE//
   //initialize_720p(&argc,&argv);
   ///initialize_pipeline(&argc,&argv);
   //start_1080p_record(argc,argv);
   //start_720p_record(&argc,&argv);
//start_720p_mpeg4(&argc,&argv);
   //start_720x960_record(argc,argv);
 
 int lag = 0;
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
        if(elapsed_20 > 0.05)//was 0.03
	{
            last_time_20 = (double)(start.tv_sec + start.tv_usec/1000000.0);
            elapsed_time_20 = elapsed_20;
            uart_read_nc(&recv);// read to empty buffer
            send_string("+IPD\r\n");
            usleep(2000);
            uart_read_nc(&recv);//try to remove while 
           if(recv == 1)
           {
               recv = 0;
               b++;
           }
               
           if(link_status == 1||link_status == 0 && gain_recv == 0)    
            {
                //esp8266_send(4);
                
                //int16_t x_angle_d = (comp_angle_pitch * 10)-900;
                //int16_t y_angle_d = (comp_angle_roll * 10)-900;
	      //if(elapsed > 0.5)lag+=1;
	      /*
	      if((elapsed*1000)>lag)lag=(elapsed*1000);
                int16_t pitch_control_d = lag;//wait_for_state_change;//rec_com;//(int16_t)pitch_control;
                int16_t roll_control_d = pipeline_status;//y_com;//(int16_t)roll_control;
                //int16_t altitude = (int16_t)(alt);i_cmd_pitch
                int16_t altitude = (press_pa-101000);//alt;// (int16_t)(i_cmd_pitch);//check integral wind-up  
                //int16_t refresh = loop_rate;
                int16_t refresh = loop_rate_20;// (int16_t)(i_cmd_roll);
                //int16_t connected = recv;
                int16_t connected = loop_rate;
	      */
	        int16_t pitch_control_d = comp_angle_pitch;//wait_for_state_change;//rec_com;//(int16_t)pitch_control;
                int16_t roll_control_d = comp_angle_roll;//y_com;//(int16_t)roll_control;
                //int16_t altitude = (int16_t)(alt);i_cmd_pitch
                int16_t altitude = i_cmd_pitch;//alt;// (int16_t)(i_cmd_pitch);//check integral wind-up  
                //int16_t refresh = loop_rate;
                int16_t refresh = i_cmd_roll;// (int16_t)(i_cmd_roll);
                //int16_t connected = recv;
                int16_t connected = loop_rate;
                
                debug_send(pitch_control_d,roll_control_d,altitude,refresh,connected);
            }
            
            if(gain_recv == 1)
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
        loop_rate_20 = (b - last_20)/elapsed_long;
	last_20 = b;
        
	last_time = (double)(start.tv_sec + start.tv_usec/1000000.0);

	//get_alt();
        hmc_read(&mag_x,&mag_y,&mag_z);
        
        /*
        //print sensor values:
        printf("Gyro rate X: %d Y: %d Z: %d\n",x_gyro_rate,y_gyro_rate,z_gyro_rate);
	printf("Accelerometer acceleration X: %d Y: %d Z: %d\n",x_acceleration,y_acceleration,z_acceleration);
        printf("Complementary filter angle X: %f Y: %f\n",comp_angle_x,comp_angle_y);
        printf("Elapsed long: %f Loop rate HZ: %f current time %f \n", elapsed_long,loop_rate , curr_time);
        printf("x mag: %d y mag: %d z mag: %d \n",mag_x,mag_y,mag_z);
        */
      //printf("temp_deg: %d press_pa: %d Altitude: %f \n",temp_deg,press_pa,alt);
        //printf("pwm_counter: %d pwm_direction: %d pwm_count: %d\n",pwm_counter,pwm_direction,pwm_count);
      //printf("Elapsed long: %f Loop rate HZ: %f current time %f link_status: %d \n", elapsed_long,loop_rate, curr_time,link_status);
        //printf("Angle Pitch: %f Roll: %f Pitch control: %f Roll control: %f\n",comp_angle_pitch,comp_angle_roll,pitch_control,roll_control);
        //printf("x joy: %d y joy: %d t joy: %d r joy: %d rec_com %d\n",x_com,y_com,t_com,r_com,rec_com);
       //printf("Pitch control: %f Roll control: %f Throttle command: %d Pitch command: %d\n",pitch_control,roll_control,t_com,y_com);
        //printf("x joy: %d y joy: %d t joy: %d r joy: %d\n",x_com,y_com,t_com,r_com);
        
        if(link_status == 1||link_status == 2)    
        {
            /*
        fp = fopen("log.txt", "a");
          // write to the file /
        fprintf(fp,"Elapsed long: %f Loop rate com HZ: %f Loop rate Main: %f current time %f \n", elapsed_time_20,loop_rate_20,loop_rate,curr_time);
          // close the file /
        fclose(fp);
        */
        }
#if LOG_WRITE
        write_log();//write log data at 2hz
#endif
       //check_pipeline_status();//check if pipeline has to be stopped or started
        check_gst_pipe();//check if pipeline has to be stopped or started
	}
	
	//START MAIN LOOP CODE
        
        get_alt();
        get_angles(elapsed);//get_angles(&comp_angle_x, &comp_angle_y, elapsed);
        link_check(loop_rate_20);//check if we have a good signal
        PID_cascaded(elapsed);//run the cascaded PID loop
#if LOG_WRITE
        log_data(elapsed,curr_time,loop_rate_20);//high speed logging
#endif
#if !LOG-WRITE
	log_data_single(elapsed,curr_time,loop_rate_20);
#endif
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

     
   }

 /*Out of the main loop, clean up nicely*/
 //g_print ("Returned, stopping playback\n");
 //gst_element_set_state (pipeline, GST_STATE_NULL);
 //g_print ("Deleting pipeline\n");
 //gst_object_unref (GST_OBJECT (pipeline));
 //gst_object_unref (g_bus);


 return 0;
}
