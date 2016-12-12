#ifndef SERIALPORT_H
#define SERIALPORT_H

/*
 * Open serial port.
 * Returns file descriptor
 */
int serial_init(char *modemdevice);

/* 
 * restore the old port settings 
 */
void serial_cleanup(int fd);

int serial_read();

int serial_write();

#endif
