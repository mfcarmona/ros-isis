/* 
 * File:   ModbusSerial.cpp
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

#include "ModbusSerial.h"    

  
ModbusSerial::ModbusSerial(){

}
    
void ModbusSerial::setPort(std::string name){
      carmenPort.setPortName( name);
}    
  
// **************************************************************************
//Function:
//   char//printModbusCommand(char *buff)
//--------------------------------------------------------------------------
//Description:
//   Send data by a serial port
//--------------------------------------------------------------------------
//Parameters:
//       char//buff: modbus command
//--------------------------------------------------------------------------
//Returned value:
//          String describing the contents of the command
//**************************************************************************
 void ModbusSerial::printModbusCommand(char * buff, int isShort, char *ans)
 {
    char tipoComando;
    int initialAddress;
    int numRegs;
    int numBytes;
    int parameter;
    int i;

    
    tipoComando = buff[0];

    if (tipoComando == (char) 0x03) {
          sprintf(ans, "Commando: Read Holding Registers");
          initialAddress = modBusParseWord( buff,-1);
          numRegs  =  modBusParseWord( buff, 1 );

	  if (isShort==0){
              
         sprintf(ans, "%s\n\t Direccion inicial : %d",ans, initialAddress);
         sprintf(ans, "%s\n\t Numero de registros : %d",ans, numRegs);
	  }
	/*
         buff[0] = (char) 0x03; // READ HOLDING REGISTERS: 0x03
         buff[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x00 = 0
         buff[2] = (char) 0x00; // INITIAL ADDR: 0x00.0x00 = 0
         buff[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x11= 17
         buff[4] = (char) 0x11; //
        */
    } else if(tipoComando == (char) 0x10){
        sprintf(ans, "Commando: Write multiple  Registers");
        initialAddress = modBusParseWord( buff,-1);
        numRegs  =  modBusParseWord( buff, 1 );
	numBytes =  (int) (buff[5]) & 0xFF ;

	if (isShort==0){
        sprintf(ans, "%s\n\t Direccion inicial : %d",ans, initialAddress);
        sprintf(ans, "%s\n\t Numero de registros : %d",ans, numRegs);
        sprintf(ans, "%s\n\t Numero de bytes : %d",ans, numBytes);

	for (i=0;i<numRegs;i++){
	  parameter= modBusParseSignedWord( buff, 4+2*i);
	  sprintf(ans, "%s\n\t valor del parametro %d: %d = 0x%02x 0x%02x",ans, i, parameter,((parameter >> 8) & (0xFF)),((parameter >> 0) & (0xFF)));                  
	}
	}
	      /*
  carmen_buffer[0]=  (char) 0x10; // write multiple registers
  carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0C = 12 (ROT_VEL)
  carmen_buffer[2] = (char) 0x0C; // INITIAL ADDR(lOW)
  carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x01 = 1
  carmen_buffer[4] = (char) 0x01; // 
  carmen_buffer[5] = (char) 0x02; // NUMBER BYTES: 0x02 = 2
  carmen_buffer[6] = (char) ((parameter >> 8) & (0xFF));
  carmen_buffer[7] = (char) ((parameter >> 0) & (0xFF));
	      */
    }else {
        sprintf(ans, "Commando desconocido 0x%X", (int) tipoComando);
    }

    
}

    
    void ModbusSerial::printModbusCommandResponse(char * buff, int isShort, char *ans) {
    
    char tipoComando;
    int initialAddress;
    int numRegs;
    int parameter;
    int i;

    
    tipoComando = buff[0];

    if (tipoComando == (char) 0x03) {
        /*
         buff[0] = (char) 0x03; // READ HOLDING REGISTERS: 0x03
         buff[1] = (char) 0x00; // NUMBER OF BYTES == 2 x Number of registers
         buff[2] = (char) ((parameter >> 8) & (0xFF)); //  high byte reg 1
         buff[3] = (char) ((parameter >> 0) & (0xFF)); //  low byte reg 1
         ...
        */
          sprintf(ans, "Respuesta: Read Holding Registers");
          numRegs  =  ((int) (buff[1]) & 0xFF)/2 ; 

	  if (isShort==0){              
                 sprintf(ans, "%s\n\t Numero de registros: %d",ans, numRegs);                 
                 
                 for (i=0;i<numRegs;i++){
                 parameter= modBusParseWord( buff, 2*i);
	         sprintf(ans, "%s\n\t valor (unsigned) del parametro %d: %d = 0x%02x 0x%02x",ans, i, parameter,((parameter >> 8) & (0xFF)),((parameter >> 0) & (0xFF)));                  
                 
                }
         }	
    } else if(tipoComando == (char) 0x10){
        sprintf(ans, "Respuesta: Write multiple  Registers");
	      /*
                carmen_buffer[0]=  (char) 0x10; // write multiple registers RESPONSE
                carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0C = 12 (ROT_VEL)
                carmen_buffer[2] = (char) 0x0C; // INITIAL ADDR(lOW)
                carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x01 = 1
                carmen_buffer[4] = (char) 0x01; // 
	      */        
        
        initialAddress = modBusParseWord( buff,-1);
        numRegs  =  modBusParseWord( buff, 1 );

	if (isShort==0){
        sprintf(ans, "%s\n\t Direccion inicial : %d",ans, initialAddress);
        sprintf(ans, "%s\n\t Numero de registros : %d",ans, numRegs);
        
	}

    }else {
        sprintf(ans, "Commando desconocido 0x%X", (int) tipoComando);
    }

    
}



   



    //returns true if serial port is opened and configured
    bool ModbusSerial::isConnected(){
      return carmenPort.isConnected();
    }


// **************************************************************************
//Function:
//   int SendModBus(int fd, char dev_addr, char *buffer, int size_buffer,
//		int num_bytes, int retry)
//--------------------------------------------------------------------------
//Description:
//   Send data by a serial port
//--------------------------------------------------------------------------
//Parameters:
//   fd         : file descriptor
//   dev_addr   : device address
//   buffer     : data to be sent
//	buffer_size: size of buffer
//   num_bytes  : number of bytes to be sent
//	retry      : number of retry if failed
//--------------------------------------------------------------------------
//Returned value:
//	>0: number of bytes sent
//	 0: bad size specified
//	-1: fatal error
//***************************************************************************
    int ModbusSerial::SendModBus( char dev_addr, char *buffer, int buffer_size, int num_bytes, int retry) {

    int num_bytes_send;
    static unsigned char *modbus_buffer = NULL;
    static int modbus_buffer_size = -1;
    unsigned char *CRC_buffer;
    int CRC_buffer_size;
    unsigned char CRC_high, CRC_low;
    unsigned short CRC_index;
    int cont_for;

    static unsigned char CRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40
    }; // Table of CRC values for high-order byte

    static char CRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
        0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
        0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
        0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
        0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
        0x40
    }; // Table of CRC values for low-order byte

    if ((num_bytes <= 0) || (num_bytes > buffer_size)) {
        return (0);
    }

    if (modbus_buffer_size == -1) {

        if ((modbus_buffer = (unsigned char *) malloc((1 + buffer_size + 2) * sizeof (char))) == NULL) {
            return (-1);
        }

        modbus_buffer_size = (1 + buffer_size + 2); // DEV_ADDR (1) + PDU (N) + CRC (2)
    }
    else if (modbus_buffer_size != (1 + buffer_size + 2)) {

        if ((modbus_buffer = (unsigned char *) realloc(modbus_buffer, (1 + buffer_size + 2) * sizeof (char))) == NULL) {
            return (-1);
        }

        modbus_buffer_size = (1 + buffer_size + 2); // DEV_ADDR (1) + PDU (N) + CRC (2)
    }

    // DEV_ADDR

    modbus_buffer[0] = dev_addr;

    // PDU

    for (cont_for = 0; cont_for < num_bytes; cont_for++) {
        modbus_buffer[1 + cont_for] = buffer[cont_for];
    }

    // CRC

    CRC_high = 0xFF; // high byte of CRC initialized
    CRC_low = 0xFF; // low byte of CRC initialized
    CRC_buffer = modbus_buffer;
    CRC_buffer_size = 1 + num_bytes; // DEV_ADDR (1) + PDU (N)

    while (CRC_buffer_size--) { // pass through message buffer
        CRC_index = CRC_low ^ *CRC_buffer++; // calculate the CRC
        CRC_low = CRC_high ^ CRCHi[CRC_index];
        CRC_high = CRCLo[CRC_index];
    }

    modbus_buffer[1 + num_bytes + 0] = CRC_low;
    modbus_buffer[1 + num_bytes + 1] = CRC_high;

    num_bytes_send = this->carmenPort.Send(modbus_buffer, modbus_buffer_size, 1 + num_bytes + 2, retry);

    if (num_bytes_send == -1) {
        return (-1);
    }

    return (num_bytes_send);
}
    
//*************************************************************************
//Function:
//   int ReceiveModBus(int fd, char dev_addr, char *buffer, int size_buffer,
//		int num_bytes, int retry)
//--------------------------------------------------------------------------
//Description:
//   Receive data by a serial port
//--------------------------------------------------------------------------
//Parameters:
//   fd         : file descriptor
//   dev_addr   : device address
//   buffer     : data to be received
//	buffer_size: size of buffer
//   num_bytes  : number of bytes to be received
//	retry      : number of retry if failed
//--------------------------------------------------------------------------
//Returned value:
//	>0: number of bytes received
//	 0: bad size specified
//	-1: fatal error
//************************************************************************** 
int ModbusSerial::ReceiveModBus( char dev_addr, char *buffer, int buffer_size, int num_bytes, int retry) {

    int num_bytes_recv;
    static unsigned char *modbus_buffer = NULL;
    static int modbus_buffer_size = -1;
    unsigned char *CRC_buffer;
    int CRC_buffer_size;
    unsigned char CRC_high, CRC_low;
    unsigned short CRC_index;
    int cont_for;

    static unsigned char CRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40
    }; // Table of CRC values for high-order byte

    static char CRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
        0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
        0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
        0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
        0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
        0x40
    }; // Table of CRC values for low-order byte

    if ((num_bytes <= 0) || (num_bytes > buffer_size)) {
        return (0);
    }
    
    if (modbus_buffer_size == -1) {

        if ((modbus_buffer = (unsigned char *) malloc((1 + buffer_size + 2) * sizeof (char))) == NULL) {
            return (-1);
        }

        modbus_buffer_size = (1 + buffer_size + 2); // DEV_ADDR (1) + PDU (N) + CRC (2)
    }
    else if (modbus_buffer_size != (1 + buffer_size + 2)) {

        if ((modbus_buffer = (unsigned char *) realloc(modbus_buffer, (1 + buffer_size + 2) * sizeof (char))) == NULL) {
            return (-1);
        }

        modbus_buffer_size = (1 + buffer_size + 2); // DEV_ADDR (1) + PDU (N) + CRC (2)
    }

    num_bytes_recv = this->carmenPort.Receive(modbus_buffer, modbus_buffer_size, 1 + num_bytes + 2, retry);

    if (num_bytes_recv == -1) {
        return (-1);
    }

    // DEV_ADDR

    if (modbus_buffer[0] != dev_addr) {
        return (-1);
    }

    // CRC

    CRC_high = 0xFF; // high byte of CRC initialized
    CRC_low = 0xFF; // low byte of CRC initialized
    CRC_buffer = modbus_buffer;
    CRC_buffer_size = 1 + num_bytes; // DEV_ADDR (1) + PDU (N)

    while (CRC_buffer_size--) { // pass through message buffer
        CRC_index = CRC_low ^ *CRC_buffer++; // calculate the CRC
        CRC_low = CRC_high ^ CRCHi[CRC_index];
        CRC_high = CRCLo[CRC_index];
    }

    if ((modbus_buffer[1 + num_bytes + 0] != CRC_low) || (modbus_buffer[1 + num_bytes + 1] != CRC_high)) {

#if DEBUG
        printf("BAD CRC: recv %x.%x -> comp %x.%x\n\n", (unsigned char) modbus_buffer[1 + num_bytes + 1], (unsigned char) modbus_buffer[1 + num_bytes + 0], CRC_high, CRC_low);
#endif

        return (-1);
    }

    // PDU

    for (cont_for = 0; cont_for < num_bytes; cont_for++) {
        buffer[cont_for] = modbus_buffer[1 + cont_for];
    }
    
    return (num_bytes_recv);
}

//*************************************************************************
//Function:
//   modBusParseWord(char *buff, int offset) 
//--------------------------------------------------------------------------
//Description:
//   reads a 16 bits long unsigned value from a modbus buffer beginning at offset.
//--------------------------------------------------------------------------
//Parameters:   
//   buffer     : modbus buffer containing response data
//   offset     : memory position of the data (i.e. offset from memory address 0 )
// --------------------------------------------------------------------------
//Returned value:
//	value as integer. does not take into account sign
//************************************************************************** * 
int ModbusSerial::modBusParseWord(char *buff, int offset) {
    int val = ((((int) (buff[2 + offset + 0])) & 0xFF) << (8)) +
              ((((int) (buff[2 + offset + 1])) & 0xFF) << (0));
    return val;
}

//**************************************************************************
//Function:
//   modBusParseSignedWord(char *buff, int offset) 
//--------------------------------------------------------------------------
//Description:
//   reads a 16 bits long SIGNED value from a modbus buffer beginning at offset.
//--------------------------------------------------------------------------
//Parameters:   
//   buffer     : modbus buffer containing response data
//   offset     : memory position of the data (i.e. offset from memory address 0 )
// --------------------------------------------------------------------------
//Returned value:
//	value as SIGNED integer (extends sign from 16 bits read)
//************************************************************************** 
int ModbusSerial::modBusParseSignedWord(char *buff, int offset) {
    int val = modBusParseWord(buff, offset);

    if ((val & 0x8000) != 0) {
        // IF NEGATIVE, EXTENDS SIGN
        val |= 0xFFFF0000;
    }
    return val;
}

//*************************************************************************
// Function:
//    modBusParseDoubleWord(char *buff, int offset) 
// --------------------------------------------------------------------------
// Description:
//    reads a 32 bits long value from a modbus buffer beginning at offset.
// --------------------------------------------------------------------------
// Parameters:   
//    buffer     : modbus buffer containing response data
//    offset     : memory position of the data (i.e. offset from memory address 0 )
//  --------------------------------------------------------------------------
// Returned value:
// 	value as integer
// ************************************************************************** *
int ModbusSerial:: modBusParseDoubleWord(char *buff, int offset) {
    int ans = ((((int) (buff[2 + offset + 0])) & 0xFF) << (24)) +
              ((((int) (buff[2 + offset + 1])) & 0xFF) << (16)) +
              ((((int) (buff[2 + offset + 2])) & 0xFF) << (8)) +
              ((((int) (buff[2 + offset + 3])) & 0xFF) << (0));

    return ans;
}




////////////////////////////
//////////////////////////
//////////////////////////

   //sends by carmen serial port modbus query stored in modbus command buffer and waits for response
    //returns true on success
bool ModbusSerial::doModbusExchange(int sendSize, int recSize, char * request, char * response)
{
  int isDone;
  int numTry_send = 0;
  int numTry_rec = 0;
  bool return_val;
  //char ans[1024];
  
  //printModbusCommand (request,0,ans);  
  //ROS_INFO("[CARMENnode] Modbus pet :  %s", ans );
  do
  {
    do
    {
      //ROS_INFO("[CARMENnode] Send Modbus request size %d",sendSize);
      isDone = SendModBus( 0x01, request, CARMEN_BUFFER, sendSize, NUM_RETRY);
     
      
      numTry_send++;
    }
    while ((isDone == -1) && (numTry_send < MAX_TRY));
    //

    if ((isDone - 3) != sendSize)
    {
      ROS_ERROR("[CARMENnode:%d] Error: serial port sent %d of %d bytes", __LINE__, isDone - 3, sendSize);
      isDone = -1;
    }
    else
    {
      //ROS_INFO("[CARMENnode] Expected Modbus response size %d",recSize);
    isDone = ReceiveModBus( 0x01, response, CARMEN_BUFFER, recSize, NUM_RETRY);
     
    }
    numTry_rec++;
  }
  while ((isDone == -1) && (numTry_rec < MAX_TRY));
  //
  
  return_val = ((isDone - 3) == recSize);

  
  if ((isDone - 3) != recSize)
  {
    ROS_ERROR("[CARMENnode:%d] Error: serial port received %d of %d bytes", __LINE__, isDone - 3, recSize);
    isDone = -1;
  }
  else
  {
    //printModbusCommandResponse (response,0,ans);
    //ROS_INFO("[CARMENnode] modbus resp: %s", ans );
  }

  return return_val;
}
//restores serial port configuration and closes it. return true if successful

bool ModbusSerial::close()
{

  if (isConnected())
  {
    
    this->carmenPort.Close();
 
   
  }
  return (!isConnected());
}

ModbusSerial::~ModbusSerial()
{
  this->close();
}

// opens and configures carmen serial port. returns true if successful

bool ModbusSerial::init(){
  bool return_val;
  
  return_val=carmenPort.init();
  
  return return_val;

}