// Dynamixel SDK platform dependent source
#include "dxl_hal.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>

long	glStartTime	= 0;
float	gfRcvWaitTime	= 0.0f;
float	gfByteTransTime	= 0.0f;

static int 		uart = -1;
const int	       id = 0;

#define DEFAULT_UART "/dev/ttyS6";  //Indirizzo giusto per la serial 4??? Trovato sul forum

int dxl_hal_open( int devIndex, float baudrate )
{
	// Opening device
	// devIndex: Device index
	// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
	// Return: 0(Failed), 1(Succeed)
	
	const char *device = DEFAULT_UART;

        /* baud rate */
	//static const speed_t speed = B19200;
	speed_t speed = baudrate;
	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);

	/* open uart */
	uart = open(device, O_RDWR|O_NOCTTY|O_NONBLOCK); //O_NONBLOCK aggiunto come da driver linux

	if (uart < 0) {
		err(1, "ERR: opening %s", device);
	}

	/* Back up the original uart configuration to restore it after exit */
	int termios_state;
	struct termios uart_config_original;

	if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
		close(uart);
		err(1, "ERR: %s: %d", device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(uart);
		err(1, "ERR: %s: %d (cfsetispeed, cfsetospeed)",
		    device, termios_state);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		close(uart);
		err(1, "ERR: %s (tcsetattr)", device);
	}

	/* Activate single wire mode */
	ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);

	int res = 1;
	if (termios_state < 0) {
	res = 0;
	}
	return res;
}

void dxl_hal_close()
{
	// Closing device
	close(uart);
}

void dxl_hal_clear(void)
{
	// Clear communication buffer
	tcflush(uart, TCIFLUSH);
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	// Transmiting date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data transmitted. -1 is error.
	int numWritePacket = write(uart, pPacket, numPacket);
	uint8_t dummy[numPacket];
	read(uart, &dummy, numPacket);

	return numWritePacket;
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	// Recieving date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data recieved. -1 is error.
	memset(pPacket, 0, numPacket);
	return read(uart, pPacket, numPacket);
}

long myclock( void )
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void dxl_hal_set_timeout( int NumRcvByte )
{
	// Start stop watch
	// NumRcvByte: number of recieving data(to calculate maximum waiting time)
	glStartTime = myclock();
	gfRcvWaitTime = (float)(gfByteTransTime*(float)NumRcvByte + 5.0f);
}

int dxl_hal_timeout(void)
{
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)
	long time;
	time = myclock() - glStartTime;
	if(time > gfRcvWaitTime)
		return 1;
	else if(time < 0)
		glStartTime = myclock();
		
	return 0;
}
