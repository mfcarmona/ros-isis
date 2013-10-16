#ifndef LIBCBRCLIENT
#define LIBCBRCLIENT

/* ************************************************************************** */
/* Includes definitions                                                       */
/* ************************************************************************** */

#include <stdio.h>				// fgets, fopen, perror, printf, sprintf
#include <stdlib.h>				// atoi, exit, malloc, realloc
#include <unistd.h>
#include <errno.h>
#include <string.h>				// strcat, strchr, strcmp, strcpy, strlen, strrchr
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>


/* ************************************************************************** */
/* Constants definitions                                                      */
/* ************************************************************************** */

// ************************************************************************** //
// Prototypes definitions                                                     //
// ************************************************************************** //


/* int searchParameteri(char *parametername,int *parametervalue)
* Loads file named filename
* Parameters:
* - filename = filename of the file to load
* Return:
* -1 on error. 0 on success
*/
int closeCBR();

int openCBR(char *host, int port);

int txCBR(char *auxstr);

/* ************************************************************************** */
/* 	Look for the parameter Name on TableParameter given (or in the file		*/
/* previously loaded with the loadParameterFile function* if pointer NULL) and*/
/* returns its	value on value. This functions reads a float value.				*/
/* ************************************************************************** */
int rxCBR(char *auxstr,int data);
	
#endif
