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

void serial_read();

void serial_write();

#endif
