/* 
 * File:   serialPortFuncs.h
 * Author: mfcarmona
 *
 * Created on 6 de septiembre de 2011, 19:10
 */

#ifndef _SERIALPORT_H
#define	_SERIALPORT_H

//.........
//INCLUDES
//.........
#include <ros/ros.h>

#include <unistd.h>             //close, write    
#include <errno.h>		// error codes
#include <fcntl.h>		// open, O_NOCTTY
#include <termio.h>		// CLOCAL, CREAD, IGNPAR,VMIN,VTIME,TCIOFLUSH,TCSANOW, TCIOFLUSH
#include <termios.h>            //tcsetattr
#include <string.h>
#include <stdio.h>
#include <unistd.h>                  //usleep
#include <pthread.h>                //mutex

class SerialPort {
public:
  
    
    SerialPort();
    
    void setPortName( std::string name);
    
    bool isConnected();
    
    bool init();
    
    virtual ~SerialPort();

int Open();

struct termios Config();

int Send( unsigned  char *buffer, int buffer_size, int num_chars, int retry);

int Receive( unsigned char *buffer, int buffer_size, int num_chars, int retry);

void Close();

private:
    int serial_port_handler;
    std::string serial_port_device_name;
    struct termios config;
    struct termios defaultConfig;
    int rw;
    int block;
    pthread_mutex_t ptm_serial;
}; //fin clase SerialPort
#endif	/* _SERIALPORT_H */