
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <linux/spi/spidev.h> //spi
#include <fcntl.h>		//Used for UART
#include <termios.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>

static const char *device = "/dev/spidev1.1";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 400000;//1000000; 4mhz 4mbit/sec? max speed of device
static uint16_t delay;

int ret = 0;
int fd_spi;
int status = 0;

void init_spi(int *connected)
{
        fd_spi = open(device, O_RDWR);
	if (fd_spi < 0)printf("can't open device\n");
    
        /*
	 * spi mode
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printf("can't set spi mode\n");

	ret = ioctl(fd_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printf("can't get spi mode\n");
        
        /*
	 * bits per word
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word\n");

	ret = ioctl(fd_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't get bits per word\n");
        
        /*
	 * max speed hz
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't set max speed hz\n");

	ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't get max speed hz\n");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
        

  // Okay, now that we're all set up, we can start thinking about transferring
  //   data. This, too, is done through ioctl(); in this case, there's a special
  //   struct (spi_ioc_transfer) defined in spidev.h which holds the needful
  //   info for completing a transfer. Its members are:
  //    * tx_buf - a pointer to the data to be transferred
  //    * rx_buf - a pointer to storage for received data
  //    * len - length in bytes of tx and rx buffers
  //    * speed_hz - the clock speed, in Hz
  //    * delay_usecs - delay between last bit and deassertion of CS
  //    * bits_per_word - override global word length for this transfer
  //    * cs_change - strobe chip select between transfers?
  //    * pad - ??? leave it alone.

  // For this example, we'll be reading the address location of an ADXL362
  //   accelerometer, then writing a value to a register and reading it back.
  //   We'll do two transfers, for ease of data handling: the first will
  //   transfer the "read register" command (0x0B) and the address (0x02), the
  //   second will dump the response back into the same buffer.

  struct spi_ioc_transfer xfer[2];
  //memset(&xfer, 0, sizeof(xfer));
  char dataBuffer[2];
  char rxBuffer[2];
  char RegOpMode = 0x01;
  const char spr = 0x07<<3;//0x03;
  const char read_byte = 0x80;
   memset(&dataBuffer, 0, sizeof(dataBuffer));
   memset(&rxBuffer, 0, sizeof(rxBuffer));
   memset(&xfer, 0, sizeof(xfer));
  
   

    xfer[0].delay_usecs = 0; //delay in us
    //xfer[1].cs_change = 0; // Keep CS activated 
    xfer[0].speed_hz = 400000; //speed
    xfer[1].speed_hz = 400000; //speed
  
    dataBuffer[0] = 0x03 << 3 | 0x80;//0x01;//(read_byte | spr);//WRITE BYTE?
    dataBuffer[1] = 0xFF; //Dummy byte
    
    xfer[0].tx_buf = (unsigned long)dataBuffer;
    xfer[0].rx_buf = (unsigned long)NULL;
    xfer[0].len = 1; // Length of  command to write
    xfer[0].cs_change = 0; // Keep CS activated 
    
    xfer[1].tx_buf = (unsigned long)NULL;
    xfer[1].rx_buf = (unsigned long) rxBuffer;
    xfer[1].len = 1; // Length of Data to read 
    status = ioctl(fd_spi, SPI_IOC_MESSAGE(2), xfer);
    
    
    //write(fd_spi,dataBuffer,1);
    //status = read(fd_spi,rxBuffer,1);
    
  printf("SPI result: %d\n", status);
  printf("RX buffer: %d // %d\n", rxBuffer[0],rxBuffer[1]);
        
    
}

