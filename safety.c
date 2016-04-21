
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <stdint.h>
#include "read_sensors.h" //initializing and reading i2c sensors
#include "serial_com.h" //initializing uart
#include "AHRS.h"//Complementary filter AHRS



void wait_signal()
{
    struct timeval start, stop;
    int waiting = 0;
    
    gettimeofday(&start, 0);
    double prev_time = start.tv_sec + (double)(start.tv_usec / 1000000.0);
    int wait_count = 0;
    
    while(waiting==0)
    {
            gettimeofday(&stop, 0);
            double curr_time =  (double)(stop.tv_sec + stop.tv_usec/1000000.0);
            double elapsed = curr_time - (double)(start.tv_sec + start.tv_usec/1000000.0);
            gettimeofday(&start, 0);
            
            if(wait_count > 10)
            {
            char recv = 0;
            uart_read_nc(&recv);// read to empty buffer
            send_string("+IPD\r\n");
            usleep(2000);
            uart_read_nc(&recv);//try to remove while 
            if(recv == 1 && t_com < -3260)
            {
                //check throttle zero (t_com = -3276;) --> -3260
                waiting = 1;
            }
            printf("Angle Pitch: %f Angle Roll: %f Throttle control: %d\n",comp_angle_pitch,comp_angle_roll,t_com);
            wait_count = 0;
            }
            get_angles(elapsed);
            wait_count +=1;
            
            usleep(10000);//100hz 
    }
}