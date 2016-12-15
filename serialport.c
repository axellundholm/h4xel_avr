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

/* Reads a character to the serial port.
*/
void serial_read() {
  char array[20];
  size_t nbytes = 1;  //We only use 8-bit conversation
  ssize_t bytes_read;
  //nbytes = sizeof(array); always 8-bits since the serial comunication & avr is 8-bit data.
  bytes_read = read(fd,array,nbytes); //nbytes
  if (bytes_read < 0) {
    printf("Oooh dear, something went wrong with the read function! %s\n", strerror(errno));
    exit(-1);
  } else {
    printf("Reading the following string: %s\n",array);
  }
}

/* Writes characters to the serial port.
*/
void serial_write() {
  char array[20];
  size_t nbytes = 1;
  ssize_t bytes_written;

  printf("Set a rpm for the motor (rpm ≈ [0,120]): ");
  scanf("%d",&array);
  getchar(); //flush the new line mark that comes with enter key after Integers
  /* strlen() : determining the length (the lenght of the string itself,
  ** not the size of the array containing it) of a null-terminated string.
  */
  //nbytes = sizeof(array); //intlen()? don't need more than 8-bit...
  bytes_written = write(fd,&array,nbytes);
  if(bytes_written < 0) {
    printf("Oooh dear, something went wrong with the write function! %s\n", strerror(errno));
    exit(-1);
  }
}

/* the Main code that alternates between reading and writing on the serial port.
*/
int main() {
  // "/dev/ttypS0 == COM1"
  fd = serial_init("/dev/ttyS0");
  char alt;
  uint8_t readMode = 250;					//to tell the AVR to transmit

  while(1) {
	   printf("r (READ) / w (WRITE) / q (QUIT): ");
     scanf("%c",&alt);
     if (alt == 'r') {
       write(fd,&readMode,1);
       serial_read();
     } else if (alt == 'w') {
       serial_write();
     } else if (alt == 'q') {
       serial_cleanup(fd);
       exit(1);
     } else {
       printf("Wrong character, try again!\n");
     }
     printf("******************** NEW LOOP *********************\n\n");
   }
   serial_cleanup(fd);
   exit(0);
   return 0;
}
