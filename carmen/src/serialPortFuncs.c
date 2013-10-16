#include "serialPortFuncs.h"




serialPort_handler = -1; // isConnected==false      
  pthread_mutex_init(&ptm_serial, NULL);



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
int OpenSerialPort(char *device, int rw, int block) {

	int fd;

	fd = open(device, rw | block | O_NOCTTY);

	int ret=tcflush(fd, TCIOFLUSH);

	if (ret==-1){
	  printf( "Error code on tcflush: %s\n", strerror( errno ) );
	}

	return(fd);
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
struct termios ConfigSerialPort(int fd, int baud, int data, int stop, int parity,
	int par_type, int mode, int size, int timer) {

	struct termios newconfig, oldconfig;

	tcgetattr(fd, &oldconfig);
	bzero(&newconfig, sizeof(newconfig));
	newconfig.c_cflag = baud | data | stop | parity | par_type | CLOCAL | CREAD;
	newconfig.c_iflag = IGNPAR;
	newconfig.c_oflag = 0;
	newconfig.c_lflag = mode;
	newconfig.c_cc[VMIN] = size;
	newconfig.c_cc[VTIME] = timer;

	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &newconfig);

	return(oldconfig);
}

/***************************************************************************
* Function:
*    void CloseSerialPort(int fd, struct termios config)
* --------------------------------------------------------------------------
* Description:
*    Closes a serial port and restores old serial port configuration
* --------------------------------------------------------------------------
* Parameters:
*		fd    : file descriptor
*    config: old serial port configuration
* --------------------------------------------------------------------------
* Returned value:
*    None
****************************************************************************/
void CloseSerialPort(int fd, struct termios config) {
	tcsetattr(fd, TCSANOW, &config);
	close(fd);
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
int SendSerialPort(int fd, unsigned char *buffer, int buffer_size, int num_chars, int retry) {

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
	  //printf("[CARMENnode] about to send %d chars\n", num_chars-offset);
            
                //printf("W xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
                num_chars_send = write(fd, buffer+offset, num_chars-offset);
                //printf("W ----------------------------------------------------\n");
                        
		if (num_chars_send  <= 0) {
			printf( "Error writting on serial port (%d)",fd );
            		printf( "Error code: %s\n", strerror( errno ) );
			return(-1);
		}

		offset += num_chars_send;


		//printf("[CARMENnode] SERIAL SENt: %d chars, begining at %d, from a total of %d\n", num_chars_send, offset, num_chars);
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
int ReceiveSerialPort(int fd, unsigned char *buffer, int buffer_size, int num_chars, int retry) {

	int offset; 			// Relative offset of received chars
	int num_chars_recv;	// Number of chars received
	int num_retry;

	// Variables initialization

	offset = 0;
	num_chars_recv = 0;
	num_retry = 0;

	// BEGIN

	if ((num_chars <= 0) || (num_chars > buffer_size)) {
	  printf( "Error reading serial port (%d). Improper size\n",fd);
		return(0);
	}

	do {                
                num_chars_recv = read(fd, buffer+offset, num_chars-offset);                                
		if ( num_chars_recv<= 0) { 
                    printf( "Error code: %s\n", strerror( errno ) );
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

