#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>		//Used for UART
#include <termios.h>		//Used for UART
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "PWM.h"//i2c PWM motor control and init
#include "stabilisation.h"//for pid gains
#include "safety.h"//check link_status

void uart_init();
void uart_send();
void uart_read();
int uart0_filestream = -1;
char gain_recv = 0;

int16_t x_com = 0;
int16_t y_com = 0;
int16_t t_com = -3276;//0
int16_t r_com = 0;
int16_t rec_com = 0;//record start stop command 0:idle 1: start 2: stop

   void uart_init()
   {
   //UART initialization

//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttymxc0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	//options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	//pcduino style: https://learn.sparkfun.com/tutorials/programming-the-pcduino/serial-communications
	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= B9600;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
   }
   
void uart_init_nc()
   {
   //UART initialization

//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttymxc0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	//options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	//pcduino style: https://learn.sparkfun.com/tutorials/programming-the-pcduino/serial-communications
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        options.c_lflag = 0;//ICANON; //ICANON FOR CANONICAL MODE
         
        options.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        options.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
   }

void uart_send()
{
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'H';
	*p_tx_buffer++ = 'e';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'o';
	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,"h",1); //write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, 			number of bytes to write
		if (count < 0)
		{
                        
                        if(errno==5)
			{
			  close(uart0_filestream);
			  uart_init();
			  printf("UART closer and initialized again\n");
			}
			printf("write error %d %s", errno, strerror(errno));
			//printf("UART TX error: %d \n",count);
		}
	}
}

void uart_read_simple()
{
//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 charac1ers from the port if they are there
		unsigned char rx_buffer[500];//256
		int rx_length = 1;
                while(rx_length > 0)
                {
                    rx_length = read(uart0_filestream, (void*)rx_buffer, 499);//255		//Filestream, buffer to store in, number of bytes to 			read (max)
                    if (rx_length < 0)
                    {
                            //An error occured (will occur if there are no bytes)
                            printf("read error %d %s\n", errno, strerror(errno));
                    }
                    else if (rx_length == 0)
                    {
                            //No data waiting
                        //int w = write(uart0_filestream,"h",1);
                        printf("No data\n");
                        
                    }
                    else
                    {
                            //Bytes received
                            rx_buffer[rx_length] = '\0';
                            printf("%i bytes read : %s\n", rx_length, rx_buffer);
                    }
                }
	}
}

void uart_read_nc(char *received)
{
//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 charac1ers from the port if they are there
		unsigned char buffer[500];//256
		unsigned char rx_buffer[1500];
		char chk_string[7];
                //unsigned char str_match[7] = "\r\n+IPD,0"; 
                //unsigned char str_match[16] = {13,10,43,73,80,68,44,48,44,49,50,58,132,122,115,152};//"\r\n+IPD,0" and preamble
                //unsigned char str_match[16] = {43,73,80,68,44,48,44,49,50,58,132,122,115,152};
                unsigned char str_match[4] = {132,122,115,152};
                unsigned char str_match_pid[16] = {43,73,80,68,44,48,44,49,50,58,133,123,116,153};
		int rx_length = 0;
                int length = 1;
                FILE *fp;//here???
                
                
                while(length > 0)
                {
                    length = read(uart0_filestream, buffer, 499);//255		//Filestream, buffer to store in, number of bytes to 			read (max)
                    if (length < 0)
                    {
                            //An error occured (will occur if there are no bytes)
                            printf("read error %d %s\n", errno, strerror(errno));
		      fp = fopen("log.txt", "a");
		      fprintf(fp,"Serial read error %d %s\n", errno, strerror(errno));
		      fclose(fp);
                    }
                    else if (length == 0)
                    {
                            //No data waiting
                        //int w = write(uart0_filestream,"h",1);
                        //printf("GPS Poked: %d\n",w);
                        
                    }
                    else if(rx_length+length < 1500)
                    {
                        memcpy(rx_buffer+rx_length, buffer, length);
                        rx_length += length;
                    }
                }
                            //Bytes received
                            
                            //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                         //    int bytes_received = memcmp(str_match,rx_buffer+10,4);//check 
                        
                        if(link_status == 1||link_status == 2)    
                        {
                            /* open the file /
                            fp = fopen("log.txt", "a");
                            // write to the file /
                            fprintf(fp,"%i bytes read :\n", rx_length);
                            for(int i = 0;i<rx_length;i++)
                            {
                            fprintf(fp,"%d/",rx_buffer[i]);
                            }
                             fprintf(fp,">end\n");
                           
                            fclose(fp);
                            */
                        }
                        
                        
                            for(int i = 0;i<rx_length;i++)
                            {
                                if(rx_buffer[i]==132 && rx_buffer[i+1]==122 && rx_buffer[i+2]==115 && rx_buffer[i+3]==152)
                                {
                                    
                                    int16_t chk_sum = (rx_buffer[i+14] << 8) | rx_buffer[i+13];
                                    int16_t sum = 0;
                                    for(int j = 4;j<13;j++)
                                    {
                                       sum += rx_buffer[i+j];
                                    }
                                  //printf("sum: %d chk_sum: %d\n",sum,chk_sum);
                                  if(sum == chk_sum)
                                  {
                                    *received = 1;
                                    x_com = ((rx_buffer[i+5] << 8) | rx_buffer[i+4]);//different sign on roll was changed here
                                    y_com = (rx_buffer[i+7] << 8) | rx_buffer[i+6];
                                    t_com = (rx_buffer[i+9] << 8) | rx_buffer[i+8];
                                    r_com = (rx_buffer[i+11] << 8) | rx_buffer[i+10];
				    rec_com = rx_buffer[i+12];
				    
                                    //printf("x joy: %d y joy: %d t joy: %d r joy: %d\n",x_com,y_com,t_com,r_com);
                                   // fprintf(fp,"ack\n");
                                    break;
                                  }
                                }//pid gains
                                else if(rx_buffer[i]==133 && rx_buffer[i+1]==123 && rx_buffer[i+2]==116 && rx_buffer[i+3]==153)
                                {
                                    //checksum 
                                    int16_t chk_sum = (rx_buffer[i+17] << 8) | rx_buffer[i+16];
                                    int16_t sum = 0;
                                    for(int j = 4;j<16;j++)
                                    {
                                       sum += rx_buffer[i+j];
                                    }
                                  //printf("sum: %d chk_sum: %d\n",sum,chk_sum);
                                  if(sum == chk_sum)
                                  {
                                    *received = 0;
                                    gain_P_X = rx_buffer[i+4];
                                    gain_i_X = rx_buffer[i+5];
                                    gain_D_X = rx_buffer[i+6];
                                    gain_P_Y = rx_buffer[i+7];
                                    gain_i_Y = rx_buffer[i+8];
                                    gain_D_Y = rx_buffer[i+9];
                                    gain_P_Z = rx_buffer[i+10];
                                    gain_i_Z = rx_buffer[i+11];
                                    gain_P_X_O = rx_buffer[i+12];
                                    gain_P_Y_O = rx_buffer[i+13];
				    pitch_trim = rx_buffer[i+14];
                                    roll_trim = rx_buffer[i+15];
                                    
                                    gain_recv = 1;
                                    
                                    break;
                                  }
                                }
                               
                            }
                            
                    }

}

void uart_read_old(char *received)
{
//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 charac1ers from the port if they are there
		unsigned char rx_buffer[500];//256
		char chk_string[7];
                //unsigned char str_match[7] = "\r\n+IPD,0"; 
                //unsigned char str_match[16] = {13,10,43,73,80,68,44,48,44,49,50,58,132,122,115,152};//"\r\n+IPD,0" and preamble
                //unsigned char str_match[16] = {43,73,80,68,44,48,44,49,50,58,132,122,115,152};
                unsigned char str_match[4] = {132,122,115,152};
                unsigned char str_match_pid[16] = {43,73,80,68,44,48,44,49,50,58,133,123,116,153};
		int rx_length = 1;
                FILE *fp;//here???
                while(rx_length > 0)
                {
                    rx_length = read(uart0_filestream, rx_buffer, 499);//255		//Filestream, buffer to store in, number of bytes to 			read (max)
                    if (rx_length < 0)
                    {
                            //An error occured (will occur if there are no bytes)
                            //printf("read error %d %s\n", errno, strerror(errno));
                    }
                    else if (rx_length == 0)
                    {
                            //No data waiting
                        //int w = write(uart0_filestream,"h",1);
                        //printf("GPS Poked: %d\n",w);
                        
                    }
                    else
                    {
                        
                        
                            //Bytes received
                            
                            //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                         //    int bytes_received = memcmp(str_match,rx_buffer+10,4);//check 
                        
                        if(link_status == 1||link_status == 2)    
                        {
                            // open the file 
                            fp = fopen("log.txt", "a");
                            // write to the file 
                            fprintf(fp,"%i bytes read :\n", rx_length);
                            for(int i = 0;i<rx_length;i++)
                            {
                            fprintf(fp,"%d/",rx_buffer[i]);
                            }
                             fprintf(fp,">end\n");
                           
                            // close the file /
                            fclose(fp);
                        }
                        
                        
                            for(int i = 0;i<rx_length;i++)
                            {
                                if(rx_buffer[i]==132 && rx_buffer[i+1]==122 && rx_buffer[i+2]==115 && rx_buffer[i+3]==152)
                                {
                                    
                                    int16_t chk_sum = (rx_buffer[i+13] << 8) | rx_buffer[i+12];
                                    int16_t sum = 0;
                                    for(int j = 4;j<12;j++)
                                    {
                                       sum += rx_buffer[i+j];
                                    }
                                  //printf("sum: %d chk_sum: %d\n",sum,chk_sum);
                                  if(sum == chk_sum)
                                  {
                                    *received = 1;
                                    x_com = -((rx_buffer[i+5] << 8) | rx_buffer[i+4]);//different sign on roll
                                    y_com = (rx_buffer[i+7] << 8) | rx_buffer[i+6];
                                    t_com = (rx_buffer[i+9] << 8) | rx_buffer[i+8];
                                    r_com = (rx_buffer[i+11] << 8) | rx_buffer[i+10];
                                    //printf("x joy: %d y joy: %d t joy: %d r joy: %d\n",x_com,y_com,t_com,r_com);
                                   // fprintf(fp,"ack\n");
                                    break;
                                  }
                                }//pid gains
                                else if(rx_buffer[i]==133 && rx_buffer[i+1]==123 && rx_buffer[i+2]==116 && rx_buffer[i+3]==153)
                                {
                                    //checksum 
                                    int16_t chk_sum = (rx_buffer[i+15] << 8) | rx_buffer[i+14];
                                    int16_t sum = 0;
                                    for(int j = 4;j<14;j++)
                                    {
                                       sum += rx_buffer[i+j];
                                    }
                                  //printf("sum: %d chk_sum: %d\n",sum,chk_sum);
                                  if(sum == chk_sum)
                                  {
                                    *received = 0;
                                    gain_P_X = rx_buffer[i+4];
                                    gain_i_X = rx_buffer[i+5];
                                    gain_D_X = rx_buffer[i+6];
                                    gain_P_Y = rx_buffer[i+7];
                                    gain_i_Y = rx_buffer[i+8];
                                    gain_D_Y = rx_buffer[i+9];
                                    gain_P_Z = rx_buffer[i+10];
                                    gain_i_Z = rx_buffer[i+11];
                                    gain_P_X_O = rx_buffer[i+12];
                                    gain_P_Y_O = rx_buffer[i+13];
                                    
                                    gain_recv = 1;
                                    
                                    //fprintf(fp,"ack pid\n");
                                    break;
                                  }
                                }
                                //fprintf(fp,"%d/",rx_buffer[i]);
                            }
                           // fprintf(fp,">end\n");
                           
                            // close the file 
                            //fclose(fp);
                            
                            
                            int bytes_received = memcmp(str_match_pid,rx_buffer,14);
                            
                            if(bytes_received == 0)
                            {
                                //esp8266_send(4);
                                *received = 0;
                                //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                                  //printf("bytes received: %d\n",bytes_received);
                                
                                  //printf("DATA received\n");
                                
                                gain_P_X = rx_buffer[14];
                                gain_i_X = rx_buffer[15];
                                gain_D_X = rx_buffer[16];
                                gain_P_Y = rx_buffer[17];
                                gain_i_Y = rx_buffer[18];
                                gain_D_Y = rx_buffer[19];
                                gain_P_Z = rx_buffer[20];
                                gain_i_Z = rx_buffer[21];
                                
                               gain_recv = 1;
                               
                                
                                
                    
                            }
                            
                            
                            
                    }
                }//
	}
}

void uart_read(char *received)
{
//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 charac1ers from the port if they are there
		unsigned char rx_buffer[500];//256
		char chk_string[7];
                //unsigned char str_match[7] = "\r\n+IPD,0"; 
                //unsigned char str_match[16] = {13,10,43,73,80,68,44,48,44,49,50,58,132,122,115,152};//"\r\n+IPD,0" and preamble
                //unsigned char str_match[16] = {43,73,80,68,44,48,44,49,50,58,132,122,115,152};
                unsigned char str_match[4] = {132,122,115,152};
                unsigned char str_match_pid[16] = {43,73,80,68,44,48,44,49,50,58,133,123,116,153};
		int rx_length = 1;
                FILE *fp;//here???
                while(rx_length > 0)
                {
                    rx_length = read(uart0_filestream, rx_buffer, 499);//255		//Filestream, buffer to store in, number of bytes to 			read (max)
                    if (rx_length < 0)
                    {
                            //An error occured (will occur if there are no bytes)
                            //printf("read error %d %s\n", errno, strerror(errno));
                    }
                    else if (rx_length == 0)
                    {
                            //No data waiting
                        //int w = write(uart0_filestream,"h",1);
                        //printf("GPS Poked: %d\n",w);
                        
                    }
                    else
                    {
                            //Bytes received
                            //rx_buffer[rx_length] = '\0';
                            //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                             int bytes_received = memcmp(str_match,rx_buffer+10,4);//check 
                            /* open the file */
                            fp = fopen("log.txt", "a");
                            /* write to the file */
                            //fprintf(fp,"%i bytes read : %s\n", rx_length, rx_buffer);
                            
                            fprintf(fp,"%i bytes read :\n", rx_length);
                            for(int i = 0;i<rx_length;i++)
                            {
                                fprintf(fp,"%d/",rx_buffer[i]);
                            }
                            fprintf(fp,">end\n");
                            fprintf(fp,"ack: %d\n",bytes_received);
                            /* close the file */
                            fclose(fp);
                            //strncpy(chk_string,rx_buffer,6);
                            //chk_string[6] = '\0';
                            //int st = strcmp(chk_string,"+IPD,0");
                            //if(st > 0)printf("bytes received\n");
                            //esp8266_send(chk_string,4)
                            //printf("check string : %s string compare %d\n",chk_string,st);
                            //printf("%i bytes: %s\n", rx_length, rx_buffer);
                            if(bytes_received == 0)
                            {
                                //esp8266_send(4);
                                *received = 1;
                                //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                                  //printf("bytes received: %d\n",bytes_received);
                                
                                  //printf("DATA received\n");
                                
                                 x_com = (rx_buffer[15] << 8) | rx_buffer[14];
                                 y_com = (rx_buffer[17] << 8) | rx_buffer[16];
                                 t_com = (rx_buffer[19] << 8) | rx_buffer[18];
                                 r_com = (rx_buffer[21] << 8) | rx_buffer[20];
                                //int throttle = 205 + (t_com + 3276)/26;
                                
                                printf("x joy: %d y joy: %d t joy: %d r joy: %d\n",x_com,y_com,t_com,r_com);
                                
                                 int x_command = (x_com/20) + 300;
                                
                                 //pwm_set_all(throttle,throttle ,throttle ,throttle );
                                 //pwm_set_all(205,205,205,throttle );
                                 
                                //pwm_set_all(x_command,x_command ,x_command ,x_command );
                               /* 
                            printf("string match : %d / %d / %d / %d / %d / %d\n",str_match[0],str_match[1],str_match[2],str_match[3],str_match[4],str_match[5]);
                            printf("received : %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d  / %d\n",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rx_buffer[4],rx_buffer[5],rx_buffer[6],rx_buffer[7],rx_buffer[8],rx_buffer[9],rx_buffer[10],rx_buffer[11],rx_buffer[12],rx_buffer[13]);
                                */
                            }
                            
                            bytes_received = memcmp(str_match_pid,rx_buffer,14);
                            
                            if(bytes_received == 0)
                            {
                                //esp8266_send(4);
                                *received = 0;
                                //printf("%i bytes read : %s\n", rx_length, rx_buffer);
                                  //printf("bytes received: %d\n",bytes_received);
                                
                                  //printf("DATA received\n");
                                
                                gain_P_X = rx_buffer[14];
                                gain_i_X = rx_buffer[15];
                                gain_D_X = rx_buffer[16];
                                gain_P_Y = rx_buffer[17];
                                gain_i_Y = rx_buffer[18];
                                gain_D_Y = rx_buffer[19];
                                gain_P_Z = rx_buffer[20];
                                gain_i_Z = rx_buffer[21];
                                
                               gain_recv = 1;
                               
                                
                                
                    
                            }
                            
                            
                            
                            //printf("bytes received: %d\n",bytes_received);
                            /*
                            printf("string match : %d / %d / %d / %d / %d / %d\n",str_match[0],str_match[1],str_match[2],str_match[3],str_match[4],str_match[5]);
                            printf("received : %d / %d / %d / %d / %d / %d\n",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rx_buffer[4],rx_buffer[5]);
                            */
                            
                    }
                }//
	}
}

void esp8266_init()
{
    //----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'A';
	*p_tx_buffer++ = 'T';
	*p_tx_buffer++ = '+';
	*p_tx_buffer++ = 'G';
	*p_tx_buffer++ = 'M';
	*p_tx_buffer++ = 'R';
	*p_tx_buffer++ = '\r';
	*p_tx_buffer++ = '\n';
	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,"AT+CWLAP\r\n",12); //write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, 			number of bytes to write
		if (count < 0)
		{
                    printf("UART TX error: %d \n",count);
		}
	}
    
}

void send_string(char out_buffer[])
{
	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,out_buffer,strlen(out_buffer)); //write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, 			number of bytes to write
		if (count < 0)
		{
                    printf("UART TX error: %d \n",count);
		}
	}
    
}

void esp8266_send(int length)
{
        char data_buffer[4];
        data_buffer[0] = 68;
        data_buffer[1] = 65;
        data_buffer[2] = 84;
        data_buffer[3] = 68;
        
        send_string("AT+CIPSEND=0,4\r\n");
        usleep(5000);
        
        if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,data_buffer,4);  
		if (count < 0)
		{
                    printf("UART TX error: %d \n",count);
		}
	}
    
}
    
int debug_send(int16_t x_angle, int16_t y_angle, int16_t alt, int16_t loop_rate, int16_t connected)
{
    
        unsigned char data_buffer[10];
        data_buffer[0] = x_angle & 0xFF;
        data_buffer[1] = x_angle >> 8;
        data_buffer[2] = y_angle & 0xFF;
        data_buffer[3] = y_angle >> 8;
        data_buffer[4] = alt & 0xFF;
        data_buffer[5] = alt >> 8;
        data_buffer[6] = loop_rate & 0xFF;
        data_buffer[7] = loop_rate >> 8;
        data_buffer[8] = connected & 0xFF;
        data_buffer[9] = connected >> 8;
        
        // merge two char into short
        //MyShort = (Char2 << 8) | Char1;
        
        send_string("AT+CIPSEND=0,10\r\n");
        usleep(5000);
        
        if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,data_buffer,10);  
		if (count < 0)
		{
                    printf("UART TX error: %d \n",count);
		}
	}
}

void gain_send()
{
    
        unsigned char data_buffer[12];
        data_buffer[0] = gain_P_X;
        data_buffer[1] = gain_i_X;
        data_buffer[2] = gain_D_X;
        data_buffer[3] = gain_P_Y;
        data_buffer[4] = gain_i_Y;
        data_buffer[5] = gain_D_Y;
        data_buffer[6] = gain_P_Z;
        data_buffer[7] = gain_i_Z;
        data_buffer[8] = gain_P_X_O;
        data_buffer[9] = gain_P_Y_O;
	data_buffer[10] = pitch_trim;
        data_buffer[11] = roll_trim;
      
        
        // merge two char into short
        //MyShort = (Char2 << 8) | Char1;
        
        send_string("AT+CIPSEND=0,12\r\n");
        usleep(5000);
        
        if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream,data_buffer,12);  
		if (count < 0)
		{
                    printf("UART TX error: %d \n",count);
		}
	}
}
