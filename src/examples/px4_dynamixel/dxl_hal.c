// Dynamixel SDK platform dependent source

#include "dxl_hal.h"
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include "../../../src/drivers/hott/comms.cpp"

long	glStartTime	= 0;
double  	gfRcvWaitTime	= 0;
double 	gfByteTransTime	= 0;

static int 		uart = -1;
const int	       id = 0;

#define DEFAULT_UART "/dev/ttyS6";  //Indirizzo giusto per la serial 4

// Opening device
// devIndex: Device index
// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
// Return: 0(Failed), 1(Succeed)
int dxl_hal_open( int devIndex, float baudrate )
{
	const char *device = DEFAULT_UART;

    /* baud rate */
	speed_t speed = baudrate;
	gfByteTransTime = (double)(10000000.0f / baudrate); //Modificato da default
    //printf("baudrate: %.2g  gfByteTransTime: %.2g \n",(double)baudrate,gfByteTransTime);

	dxl_hal_close();

	/* open uart */
	uart = open(device, O_RDWR|O_NOCTTY|O_NONBLOCK); //O_NONBLOCK aggiunto come da driver linux
	if (uart < 0) {
		printf("ERR: opening %s", device);
	}

	/* Back up the original uart configuration to restore it after exit */
	int termios_state;
	struct termios uart_config_original;

	if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
		close(uart);
		printf("ERR: %s: %d", device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(uart);
		printf("ERR: %s: %d (cfsetispeed, cfsetospeed)",
		    device, termios_state);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		close(uart);
		printf("ERR: %s (tcsetattr)", device);
	}

	/* Activate single wire mode */
	ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);

	int res = 1;
	if (termios_state < 0) {
	res = 0;
	}
	return res;
}
// Closing device
void dxl_hal_close()
{
	close(uart);
}

// Clear communication buffer
void dxl_hal_clear(void)
{
	tcflush(uart, TCIFLUSH);
}

// Transmiting date
// *pPacket: data array pointer
// numPacket: number of data array
// Return: number of data transmitted. -1 is error.
int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{

    tcflush(uart, TCIFLUSH);
    int numWritePacket = write(uart, pPacket, numPacket);

////    printf("Post Write %d \n",myclock());
////	printf("Write: ", numWritePacket, numPacket);
////  for (int var = 0; var < numPacket; ++var) {
////      printf("%d ", pPacket[var]);
////  }
////	printf("\n");
////	printf("Post Print %d \n",myclock());
//
//	uint8_t dummy[numPacket];
//	//usleep((int)(gfByteTransTime*(double)(numPacket)+500)); //Tempo di attesa necessario perchè il buffer legga il msg inviato
//
//	usleep(20);
//	int i=0;
//	int err=0;
////	long time = myclock() + (int)(gfByteTransTime*(double)(numPacket*2));
////	while((i<numPacket) && (time>myclock())){
////	printf("Read %d \n",myclock());
////	printf("dummy: ");
//	while(i<numPacket){
//        read(uart,&dummy,1);
//        //printf("%d ",dummy[0]);
//        if (dummy[0] != pPacket[i]) {
//            printf("pPacket: %d dummy: %d\n",pPacket[i],dummy[0]);
//            err = 1;
//            } else
//                i++;
//        }
////	printf("\n");
////	printf("EndRead %d \n",myclock());
//	usleep(20);

    uint8_t dummy[numPacket];
    usleep(1500);
    read(uart, &dummy, numPacket);
	int var;
	int err = 0;
	for (var = 0; var < numPacket; ++var) {
        if (dummy[var] != pPacket[var]) {
            err = 1;
            break;
        }
    }
    if (err) {
            printf("[DYNAMIXEL] Errore clean buffer lettura \n");
    }
	return numWritePacket-err;
}

// Recieving date
// *pPacket: data array pointer
// numPacket: number of data array
// Return: number of data recieved. -1 is error.
int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	memset(pPacket, 0, numPacket);
	int readPacket = read(uart, pPacket, numPacket);
	return readPacket;
}

long myclock( void )
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_usec);
	//return ((tv.tv_sec) * 1000 + tv.tv_usec / 1000);
}

// Start stop watch
// NumRcvByte: number of recieving data(to calculate maximum waiting time)
void dxl_hal_set_timeout( int NumRcvByte )
{
	glStartTime = myclock();
	gfRcvWaitTime = (double)(gfByteTransTime*(double)NumRcvByte+200);
	//printf("StartTime: %d Wait time: %f \n", glStartTime, (double)gfRcvWaitTime);
}

// Check timeout
// Return: 0 is false, 1 is true(timeout occurred)
int dxl_hal_timeout(void)
{
	long time;
	time = myclock() - glStartTime;
	if(time > gfRcvWaitTime){
	    printf("[DYNAMIXEL] Timeout ricezione pacchetto! \n");
		return 1;
	}
	else if(time < 0)
		glStartTime = myclock();
	return 0;
}
