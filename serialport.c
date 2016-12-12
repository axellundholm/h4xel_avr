#include "serialport.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

/* baudrate settings are defined in <asm/termbits.h>, which is
 * included by <termios.h> */
#ifndef BAUDRATE
#define BAUDRATE B2400
#endif

#define _POSIX_SOURCE 1		/* POSIX compliant source */

static int fd, c, res;
static struct termios oldtio, newtio;
static char *device;

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

int serial_read() {

    char buf[100];
    size_t nbytes = sizeof(buf);
    ssize_t bytes_read;

    bytes_read = read(fd, buf, nbytes);

    if(bytes_read > 0) {
        printf("%zd bytes was read from device.\n", bytes_read);
        printf("The following string was read: \n%s\n", buf);
    } else {
        printf("%s\n", strerror(errno));
        exit(-1);
    }

    return bytes_read;
}

int serial_write() {
    
    char buf[100];
    ssize_t bytes_written;

    printf("Enter message: \n");
    scanf("%s", buf);

    size_t nbytes = strlen(buf);

    bytes_written = write(fd, buf, nbytes);

    if(bytes_written > 0) {
        printf("%zd bytes was written to device.\n", bytes_written);
    } else {
        printf("%s\n", strerror(errno));
        exit(-1);
    }

    return bytes_written;
}


int main(int argc, char *argv[]) {

    fd = serial_init("/dev/ttyS0");

    while(1){
        serial_write();
        serial_read();
    }

    serial_cleanup(fd);

    return 0;
}

