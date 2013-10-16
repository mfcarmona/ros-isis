
/* ************************************************************************** */
/* Includes definitions                                                       */
/* ************************************************************************** */
#include "libCbrClient.h"

/* ************************************************************************** */
/* Constants definitions                                                      */
/* ************************************************************************** */
#define MAXDATASIZE 2048 				// max number of bytes we can get at once 
#define DEBUGLIBFPARAMS    0			// 1 = SHOW DEBUG INFO ON CONSOLE;   0 = DON'T SHOW IT
#define VERBOSELIBFPARAMS  0			// 1 = SHOW VERBOSE INFO ON CONSOLE; 0 = DON'T SHOW IT

/* ************************************************************************** */
/* Global variables definitions                                               */
/* ************************************************************************** */

int sockfd, numbytes;  
char buf[MAXDATASIZE];
struct hostent *he;
struct sockaddr_in their_addr; // connector's address information 
struct sockaddr_in server_addr;
static int cbr_socket = -1;
char host[100];	// host where is located the CBR Server
int port = 5000;								// port where is located the CBR Server

/* int searchParameteri(char *parametername,int *parametervalue)
* Loads file named filename
* Parameters:
* - filename = filename of the file to load
* Return:
* -1 on error. 0 on success
*/
int closeCBR()
{
	if (cbr_socket != -1)
	{	// Close CBR
		if (close(cbr_socket) == -1)
		{
			perror("\nERROR [CBR] (close:cbr_socket)");
			return (0);
		}
		cbr_socket = -1;
		//printf("Socket closed.");
		return (1);
	}
	return (0);
}

int openCBR(char *host, int port)
{
	if (cbr_socket != -1)
	{
		if ( !closeCBR() )
			perror("\nERROR [CBR] (closing previously open:cbr_socket)");
	}
	bzero((char *) &server_addr, sizeof(server_addr));
	server_addr.sin_family		 = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(host);
	server_addr.sin_port 		 = htons(port);
	if ((cbr_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		perror("\nERROR [CBR] (socket:cbr_socket)");
		return(0);
	}
	if (connect(cbr_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) == -1)
	{
		perror("\nERROR [CBR] (connect:cbr_socket)");
		if ( !closeCBR() )
			perror("\nERROR [CBR] (close:cbr_socket)");
		cbr_socket = -1;
		return(0);
	}
	return (1);
}

int txCBR(char *auxstr)
{	// Transmitiendo datos
	if (cbr_socket != -1)
	{
		if (write(cbr_socket, auxstr, strlen(auxstr)) < 0)
		{
			perror("\nERROR [CBR] (write:aux_string)");
			return(0);
		}
		else
			;//printf("Tx: %s",auxstr);
		return (1);
	}
	return (0);
}

/* ************************************************************************** */
/* 	Look for the parameter Name on TableParameter given (or in the file		*/
/* previously loaded with the loadParameterFile function* if pointer NULL) and*/
/* returns its	value on value. This functions reads a float value.				*/
/* ************************************************************************** */
int rxCBR(char *auxstr,int data)
{	// RECIBIENDO DATOS
	int ret = 0;
	memset(auxstr,0,data);
	if (cbr_socket != -1)
	{
		if ( (ret = read(cbr_socket, auxstr, data)) < 0)
		{
			perror("\nERROR [CBR] (read:aux_string)");
			return(0);
		}
		else
		{
			;//printf("Rx: %s\n",auxstr);
			
			//char aux[data];
			//memset(aux,0,data);
			//char aux2[data];
			//memset(aux2,0,data);
			//sscanf(auxstr,"%s%*s%s",aux,aux2);
			//printf("Rx: %s\n",auxstr);
			//printf("Aux: %s\n",aux);
			//printf("Aux2: %s (Descartada segunda palabra)\n",aux2);
		}
		return (ret);
	}
	return(0);
}
