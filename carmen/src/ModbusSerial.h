/* 
 * File:   ModbusSerial.h
 * Author: mfcarmona
 * 
 * Funciones para el acceso a modbus sobre puerto serie.
 * El dispositivo es un puerto serie que ha de ser correctamente abierto
 * y configurado de la forma usual.
 *
 * El paquete modbus, obviamente, ha de ser construido a medida para la aplicacion
 *
 * Created on 6 de septiembre de 2011, 19:10
 */

#ifndef _MODBUSSERIAL_H
#define	_MODBUSSERIAL_H

// DEFINES
#define CARMEN_DEV	"/dev/ttyS0"
#define CARMEN_BUFFER	2048
#define NUM_RETRY	4  //intentos de lectura del puerto serie
#define MAX_TRY         4 //intentos de transferencia modbus (lectura y escritura)


//.........
//INCLUDES
//.........
#include <stdio.h>   //  NULL
#include <stdlib.h>  //malloc, realloc
#include "SerialPort.h"   

#include <ros/ros.h>



class ModbusSerial {
public:
  
    
    ModbusSerial();
    virtual ~ModbusSerial();

    //fija el identificador del puerto serie
    void setPort(std::string portName);
    
    //configura el puerto serie para usarlo y nos conecta
    bool init();
    
    void printModbusCommand(char * buff, int isShort, char *ans);
    void printModbusCommandResponse(char * buff, int isShort, char *ans) ;

    int modBusParseWord(char *buff, int offset);
    int modBusParseSignedWord(char *buff, int offset);
    int modBusParseDoubleWord(char *buff, int offset);
   
   //sends by carmen serial port modbus query stored in modbus command buffer and waits for response
    //returns true on success
    bool doModbusExchange(int sendSize,int recSize, char * request, char * response);

    //returns true if serial port is opened and configured
    bool isConnected();
    
    bool close();
    

private: 
    // serial port 
    SerialPort carmenPort;
    
    int SendModBus(char dev_addr, char *buffer, int buffer_size, int num_bytes, int retry);
    int ReceiveModBus( char dev_addr, char *buffer, int buffer_size, int num_bytes, int retry);


    
};


#endif	/* _MODBUSSERIAL_H */
