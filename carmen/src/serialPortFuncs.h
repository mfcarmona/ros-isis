/* 
 * File:   serialPortFuncs.h
 * Author: mfcarmona
 *
 * Created on 6 de septiembre de 2011, 19:10
 */

#ifndef _SERIALPORTFUNCS_H
#define	_SERIALPORTFUNCS_H



//.........
//INCLUDES
//.........
#include <unistd.h>             //close, write    
#include <errno.h>		// error codes
#include <fcntl.h>		// open, O_NOCTTY
#include <termio.h>		// CLOCAL, CREAD, IGNPAR,VMIN,VTIME,TCIOFLUSH,TCSANOW, TCIOFLUSH
#include <string.h>
#include <stdio.h>
#include <unistd.h>                  //usleep

int OpenSerialPort(char *device, int rw, int block);

struct termios ConfigSerialPort(int fd, int baud, int data, int stop, int parity, int par_type,
	int mode, int size, int timer);

int SendSerialPort(int fd, unsigned  char *buffer, int buffer_size, int num_chars, int retry);

int ReceiveSerialPort(int fd, unsigned char *buffer, int buffer_size, int num_chars, int retry);

void CloseSerialPort(int fd, struct termios config);



#endif	/* _SERIALPORTFUNCS_H */

