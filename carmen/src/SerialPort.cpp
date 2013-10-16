#include "SerialPort.h"
#include "ModbusSerial.h"

/*
      pthread_mutex_lock(&ptm_serial);

  pthread_mutex_unlock(&ptm_serial);
 */


SerialPort::SerialPort(){
  serial_port_handler = -1; // isConnected==false      
  pthread_mutex_init(&ptm_serial, NULL);
}


  void SerialPort::setPortName(std::string name){
    this->serial_port_device_name=name;
  }
  
  
  bool SerialPort::isConnected(){
    bool ans;
    ans= this->serial_port_handler!=-1;
    
    return ans;
  }
  
  
  bool SerialPort::init(){
    bool return_val;
   
    return_val = isConnected();

  if (!return_val)
  {
    //ROS_INFO("[SerialPort] Closed port %s->%s has handler %d",serialPortName.c_str(),charPort,serialPort_handler);
    Open();
    ROS_INFO("[SerialPort] OPEN port %s has handler %d",serial_port_device_name.c_str(),serial_port_handler);
    return_val = isConnected();
    if (return_val)
    {
      Config();        
    }
    else
    {
      ROS_FATAL("[SerialPort:%d] unable to open serial port ", __LINE__);
    }


  }
  else
  {
    ROS_ERROR("[SerialPort] port already opened");
  }

  return return_val;
    
  }
  
  
   void SerialPort::Close(){
     	tcsetattr(this->serial_port_handler, TCSANOW, &config);
      close(this->serial_port_handler);
   }
   
   
   SerialPort::~SerialPort(){
     this->Close();
   }

/* **************************************************************************
* Function:
*    int OpenSerialPort(char *device, int rw, int block)
* --------------------------------------------------------------------------
* Description:
*    Opens a serial port
* --------------------------------------------------------------------------
* Parameters:
*    device: device to open
*    rw    : read, write or readwrite mode [CONSTANT]
* 	        O_RDONLY (read only), O_WRONLY (write only),
* 	        O_RDWR	(readwrite)
*    block : blocking mode [CONSTANT]
* 	        0 (blocking), O_NONBLOCK (non blocking)
* --------------------------------------------------------------------------
* Returned value:
* 	File descriptor, or (-1) if an error ocurred
* ***************************************************************************/
int SerialPort::Open() {
  char *charPort;    
  
  charPort = new char [serial_port_device_name.size() + 1];
  strcpy(charPort, serial_port_device_name.c_str());
	serial_port_handler = open(charPort, O_RDWR | 0 | O_NOCTTY);

	int ret=tcflush(serial_port_handler, TCIOFLUSH);

	if (ret==-1){
	  printf( "Error code on tcflush: %s\n", strerror( errno ) );
	}

	return(serial_port_handler);
}

/***************************************************************************
* Function:
*    struct termios ConfigSerialPort(int fd, int baud, int data, int stop,
*		   int parity, int par_type, int mode, int size, int timer)
* --------------------------------------------------------------------------
* Description:
*    Configure a serial port and returns old serial port configuration
* --------------------------------------------------------------------------
* Parameters:
*    fd      : file descriptor
*    baud    : baudrate [CONSTANT]
*		 B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800,
*		 B2400, B4800, B9600, B19200, B38400, B57600, B115200,
*		          B230400
*    data    : number of bits in data [CONSTANT]
*		          CS5, CS6, CS7, CS8
*    stop    : number of stop bits [CONSTANT]
*		          0 (1 stop bit), CSTOPB (2 stop bits)
*    parity  : parity [CONSTANT]
*					 0 (none), PARENB (parity)
*		par_type: type of parity [CONSTANT]
*		          0 (even), PARODD (odd)
*    mode    : mode of operation [CONSTANT]
* 	          0 (non canonical), ICANON (canonical)
*    size    : number of chars to be read
*    timer   : timeout [miliseconds*100]
* --------------------------------------------------------------------------
* Returned value:
* 	Old serial port configuration
* ************************************************************************** */
struct termios SerialPort::Config() {
  
	tcgetattr(serial_port_handler, &defaultConfig);
	bzero(&config, sizeof(config));
  //config.c_cflag = baud | data | stop | parity | par_type | CLOCAL | CREAD;
	config.c_cflag = B115200 | CS8 | 0 | PARENB | 0 | CLOCAL | CREAD;
	config.c_iflag = IGNPAR;
	config.c_oflag = 0;
	config.c_lflag = 0;
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 1;

	tcflush(serial_port_handler, TCIOFLUSH);
	tcsetattr(serial_port_handler, TCSANOW, &config);

	return(defaultConfig);
}



/****************************************************************************
* Function:
*    int SendSerialPort(int fd, char *buffer, int size_buffer,
* 		int num_chars, int retry)
* --------------------------------------------------------------------------
* Description:
*    Send data by a serial port
* --------------------------------------------------------------------------
* Parameters:
*    fd         : file descriptor
*    buffer     : data to be sent
* 	buffer_size: size of buffer
*    num_chars  : number of chars to be sent
* 	retry      : number of retry if failed
* --------------------------------------------------------------------------
* Returned value:
* 	>0: number of chars sent
* 	 0: bad size specified
* 	-1: fatal error
* ***************************************************************************/
int SerialPort::Send(unsigned char *buffer, int buffer_size, int num_chars, int retry) {

	int offset; 			// Relative offset of sent chars
	int num_chars_send;	// Number of chars sent

	// Variables initialization

	offset = 0;
	num_chars_send = 0;

	// BEGIN

	if ((num_chars <= 0) || (num_chars > buffer_size)) {
                printf( "[serialPortFuncs] improper size\n" );
		return(0);
	}

	do {
	  //printf("[SerialPort] about to send %d chars\n", num_chars-offset);
            
                //printf("W xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
                num_chars_send = write(this->serial_port_handler, buffer+offset, num_chars-offset);
                //printf("W ----------------------------------------------------\n");
                        
		if (num_chars_send  <= 0) {
			printf( "Error writting on serial port (%d)",this->serial_port_handler );
            		printf( "Error code: %s\n", strerror( errno ) );
			return(-1);
		}

		offset += num_chars_send;


		//printf("[SerialPort] SERIAL SENt: %d chars, begining at %d, from a total of %d\n", num_chars_send, offset, num_chars);
#if DEBUG
        printf("SR Enviados %d de %d, llevo leidos %d\n",num_chars_send,num_chars,offset);
#endif
	} while (offset < num_chars);

	if (offset > num_chars) {
                printf( "Error writting on serial port. Offset bigger than numchars\n");
		return(-1);
	}


#if DEBUG
	{
		int cont;
		printf("SERIAL SEND DATA (%d): ", num_chars);
		for (cont=0;cont<num_chars;cont++){
			printf("%02X.",(unsigned char) buffer[cont]);
		}
		printf("\b\n");
	}
#endif

	return(offset);
}

/***************************************************************************
* Function:
*    int ReceiveSerialPort(int fd, char *buffer, int size_buffer,
* 		int num_chars, int retry)
* --------------------------------------------------------------------------
* Description:
*    Receive data by a serial port
* --------------------------------------------------------------------------
* Parameters:
*    fd         : file descriptor
*    buffer     : data to be received
* 	buffer_size: size of buffer
*    num_chars  : number of chars to be received
* 	retry      : number of retry if failed
* --------------------------------------------------------------------------
* Returned value:
* 	>0: number of chars received
* 	 0: bad size specified
* 	-1: fatal error
* ***************************************************************************/
int SerialPort::Receive( unsigned char *buffer, int buffer_size, int num_chars, int retry) {

	int offset; 			// Relative offset of received chars
	int num_chars_recv;	// Number of chars received
	int num_retry;

	// Variables initialization

	offset = 0;
	num_chars_recv = 0;
	num_retry = 0;

	// BEGIN

	if ((num_chars <= 0) || (num_chars > buffer_size)) {
	  printf( "Error reading serial port (%d). Improper size\n",this->serial_port_handler);
		return(0);
	}

	do {                
                num_chars_recv = read(this->serial_port_handler, buffer+offset, num_chars-offset);                                
		if ( num_chars_recv<= 0) { 
                    printf( "Error code on handler (%d): %s\n", this->serial_port_handler, strerror( errno ) );
			if (num_retry == retry) {
			  printf( "Not reading anymore. Max attempts ( %d of %d)\n",num_retry,retry );
				return(-1);
			}

			num_retry++;
		}
		else {
			num_retry = 0;
			offset += num_chars_recv;
		}
              
#if DEBUG
                printf("SR Recibidos %d de %d, llevo leidos %d\n",num_chars_recv,num_chars,offset);		
#endif

	} while (offset < num_chars);

	if (offset > num_chars) {
                printf( "Error reading serial port. Offset bigger than num_chars\n");
		return(-1);
	}

#if DEBUG
	{
		int cont;

		printf("SERIAL RECV DATA (%d): ", num_chars);

		for (cont=0;cont<num_chars;cont++){
			printf("%02X.",(unsigned char) buffer[cont]);
		}

		printf("\b\n\n");
	}
#endif

	return(offset);
}

