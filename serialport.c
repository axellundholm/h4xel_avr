#include "serialport.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>


/* baudrate settings are defined in <asm/termbits.h>, which is
 * included by <termios.h> */
#ifndef BAUDRATE
#define BAUDRATE B2400
#endif

// portable operating system interface
#define _POSIX_SOURCE 1		/* POSIX compliant source */

static int fd, c, res;
static struct termios oldtio, newtio;
static char *device;

/* The serial_init function configures the port for 2400 bps, 8 bits, no parity,
1 stop bit (commonly abbreviated 2400-8N1) and opens it for reading and writing.
*modemdevice: COM1 - port
*/
int serial_init(char *modemdevice)
{
	/*
	 * Open modem device for reading and writing and not as controlling tty
	 * because we don't want to get killed if linenoise sends CTRL-C.
	 **/
	device = modemdevice;
	fd = open (device, O_RDWR | O_NOCTTY );
	if (fd < 0)
	  {
	  perror (device);
	  exit(-1);
	  }

	tcgetattr (fd, &oldtio);	/* save current serial port settings */
	memcpy(&newtio, &oldtio, sizeof(newtio));

	cfsetispeed(&newtio, BAUDRATE);
	cfsetospeed(&newtio, BAUDRATE);

	/*
	 *CRTSCTS : output hardware flow control (only used if the cable has
	 *all necessary lines. )
	 *CS8     : 8n1 (8bit,no parity,1 stopbit)
	 *CLOCAL  : local connection, no modem contol
	 *CREAD   : enable receiving characters
	 **/
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag &= ~PARENB;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cflag |= BAUDRATE | CS8 | CLOCAL | CREAD;

	/*
	 *IGNPAR  : ignore bytes with parity errors
	 *ICRNL   : map CR to NL (otherwise a CR input on the other computer
	 *          will not terminate input)
	 *          otherwise make device raw (no other input processing)
	 **/
	newtio.c_iflag |= IGNPAR | ICRNL;

#ifndef  SEND_RAW_NEWLINES
	/*
	 * Map NL to CR NL in output.
	 *                  */
	newtio.c_oflag |= ONLCR;
#else
	newtio.c_oflag &= ~ONLCR;
#endif


	/*
	 * ICANON  : enable canonical input
	 *           disable all echo functionality, and don't send signals to calling program
	 **/
#ifndef NON_CANONICAL
	newtio.c_lflag |= ICANON;
#else
	newtio.c_lflag &= ~ICANON;
#endif

	/*
	 * now clean the modem line and activate the settings for the port
	 **/
	tcflush (fd, TCIFLUSH);
	tcsetattr (fd, TCSANOW, &newtio);

	/*
	 * terminal settings done, return file descriptor
	 **/

	return fd;
}

/* Closes the serial device if ifd == fd after restoring the serial port setting
** to the old configurations.
*/
void serial_cleanup(int ifd){
	if(ifd != fd) {
		fprintf(stderr, "WARNING! file descriptor != the one returned by serial_init()\n");
	}
	/* restore the old port settings */
	tcsetattr (ifd, TCSANOW, &oldtio);
	close(ifd);
}

// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---  //
// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---  //


/* the Main code that alternates between reading and writing on the serial port.
*/
int main (){


	int fd = serial_init("/dev/ttyS0");

	unsigned  char buf [20];
	size_t  nbytes = 1;
	ssize_t  bytes_written;
	ssize_t  bytes_read;
	unsigned char choice;
	
	while(1){
		
		printf("r (READ) / w (WRITE) / q (QUIT)\n");
		scanf("%c",&choice);
		getchar();
		if (choice == 'w') {
			printf("Set reference speed: ");
			scanf("%d\n",&buf);
			getchar();
			bytes_written = write(fd, buf , nbytes);
		} else if (choice == 'r') { 
			buf[0] = 250;
			bytes_written = write(fd, buf , nbytes);

			bytes_read = read(fd, buf, nbytes);

			printf("Speed is ");
			printf("%d\n\n", *buf);
		} else if (choice == 'q') {
			serial_cleanup(fd);
			exit(1);
		}
	}
}

