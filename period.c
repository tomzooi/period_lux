#include <stdio.h>
#include <signal.h> //used for periodic update
#include <sys/time.h>
#include <time.h> //Used for time
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <inttypes.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <errno.h>

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
     
 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

 // ALL COMMAND TSL2561
    // Default I2C RPI address in (0x39) = FLOAT ADDR (Slave) Other [(0x49) = VCC ADDR / (0x29) = GROUND ADDR]
    #define TSL2561_ADDR_LOW                   (0x29)
    #define TSL2561_ADDR_FLOAT                 (0x39)   
    #define TSL2561_ADDR_HIGH                   (0x49)
    #define TSL2561_CONTROL_POWERON             (0x03)
    #define TSL2561_CONTROL_POWEROFF          (0x00)
    #define TSL2561_GAIN_0X                        (0x00)   //No gain
    #define TSL2561_GAIN_AUTO                (0x01)
    #define TSL2561_GAIN_1X                 (0x02)
    #define TSL2561_GAIN_16X                  (0x12) // (0x10)
    #define TSL2561_INTEGRATIONTIME_13MS          (0x00)   // 13.7ms
    #define TSL2561_INTEGRATIONTIME_101MS          (0x01) // 101ms
    #define TSL2561_INTEGRATIONTIME_402MS         (0x02) // 402ms
    #define TSL2561_READBIT                   (0x01)
    #define TSL2561_COMMAND_BIT                (0x80)   //Must be 1
    #define TSL2561_CLEAR_BIT                (0x40)   //Clears any pending interrupt (write 1 to clear)
    #define TSL2561_WORD_BIT                   (0x20)   // 1 = read/write word (rather than byte)
    #define TSL2561_BLOCK_BIT                  (0x10)   // 1 = using block read/write
    #define TSL2561_REGISTER_CONTROL           (0x00)
    #define TSL2561_REGISTER_TIMING            (0x81)
    #define TSL2561_REGISTER_THRESHHOLDL_LOW      (0x02)
    #define TSL2561_REGISTER_THRESHHOLDL_HIGH     (0x03)
    #define TSL2561_REGISTER_THRESHHOLDH_LOW      (0x04)
    #define TSL2561_REGISTER_THRESHHOLDH_HIGH     (0x05)
    #define TSL2561_REGISTER_INTERRUPT            (0x06)
    #define TSL2561_REGISTER_CRC                  (0x08)
    #define TSL2561_REGISTER_ID                   (0x0A)
    #define TSL2561_REGISTER_CHAN0_LOW            (0x8C)
    #define TSL2561_REGISTER_CHAN0_HIGH           (0x8D)
    #define TSL2561_REGISTER_CHAN1_LOW            (0x8E)
    #define TSL2561_REGISTER_CHAN1_HIGH           (0x8F)
    //Delay getLux function
    #define LUXDELAY 500
    
int uart0_filestream = -1;
int fd = 0;
unsigned int lastlux;
unsigned int lastdimfactor;
/* signal process */
void everysecond() {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [180];
	unsigned int curlux;
	unsigned int newdimfactor;
	curlux = getLux(fd);

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	//strftime (buffer,80,"Now it's %x %H:%M:%S.",timeinfo);
	//printf(buffer);
	//sprintf(buffer," Lux: %d", curlux);
	//printf(buffer);
	//printf("\n");
	//fflush(stdout); // This will flush any pending printf output
	
	
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	usleep(1000);
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'T';
	*p_tx_buffer++ = timeinfo->tm_sec;
	*p_tx_buffer++ = timeinfo->tm_min;
	*p_tx_buffer++ = timeinfo->tm_hour;
	*p_tx_buffer++ = timeinfo->tm_mday;
	*p_tx_buffer++ = timeinfo->tm_mon;
	*p_tx_buffer++ = timeinfo->tm_year;
	if (abs(lastlux-curlux) > 1) { //significant change in light intensity
		printf("change!\n");
		lastlux = curlux;
		if (curlux <= 16) { // send a dimfactor
			newdimfactor = min((16-curlux)*3, 30);
			//sprintf(buffer, "newdim: %d", newdimfactor);
			//printf(buffer);
		}
		else { //set dimfactor to zero
			newdimfactor = 0;
		}
		if (newdimfactor != lastdimfactor) { // send new dimfactor
			*p_tx_buffer++ = 'D'; //send new dimfactor
			*p_tx_buffer++ = newdimfactor;
			lastdimfactor = newdimfactor;
		}
	}
	*p_tx_buffer++ = '\n';
	if (uart0_filestream != -1){
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0){
			printf("UART TX error\n");
		}
		//else {
			//printf("Sent: ");
			//printf(p_tx_buffer);
		//}
			
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)	{
			//An error occured (will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
			//No data waiting
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			printf("%i bytes read : %s\n", rx_length, rx_buffer);
		}
	
	}
}

 int getLux(int fd){
       wiringPiI2CWriteReg8(fd, TSL2561_COMMAND_BIT, TSL2561_CONTROL_POWERON); //enable the device
       wiringPiI2CWriteReg8(fd, TSL2561_REGISTER_TIMING, TSL2561_GAIN_AUTO); //auto gain and timing = 101 mSec
       //Wait for the conversion to complete
       delay(LUXDELAY);
       //Reads visible + IR diode from the I2C device auto
       uint16_t visible_and_ir = wiringPiI2CReadReg16(fd, TSL2561_REGISTER_CHAN0_LOW);
       wiringPiI2CWriteReg8(fd, TSL2561_COMMAND_BIT, TSL2561_CONTROL_POWEROFF); //disable the device
       return visible_and_ir*2;
    }


int main(void) {
	fd = wiringPiI2CSetup(TSL2561_ADDR_LOW); //open lux sensor system
	lastlux = getLux(fd);
    char *str;
    char c;
    struct timespec tim, tim2;
	tim.tv_sec = 5;
	//tim.tv_nsec = 0;
    
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	else {
		printf("uart opened, running...\n");
	}
	fflush(stdout); // This will flush any pending printf output
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
    
    while(1) {
		if(nanosleep(&tim , &tim2) < 0 )   {
		  printf("Nano sleep system call failed \n");
		  return -1;
		}
		else {
			everysecond();
		}
	}
    return 0;
}
