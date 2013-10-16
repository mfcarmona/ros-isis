/* ************************************************************************** */
/* Includes definitions 																		*/
/* ************************************************************************** */

#include "libParams.h"  //parse config data given by DLA connection
#include "libCbrClient.h" //openCBR, closeCBR, txCBR, rxCBR

#include "DLALibrary.h"		// DLA client library
#include <fcntl.h>

#include <ctype.h>						// tolower
#include <math.h>							// atan, cos, sin, sqrt
#include <signal.h>						// signal
#include <stdio.h>						// fclose, fgets, fgetpos, fopen, fputs, fsetpos, sprintf, perror, printf
#include <stdlib.h>						// atof, atoi, exit, malloc
#include <string.h>						// strcat, strcmp, strcpy, strlen, strrchr
#include <time.h>							// gmtime, time
#include <unistd.h>						// gettimeofday, sleep

#include <netinet/in.h> 				// DEFINES;
#include <arpa/inet.h>					// DEFINES;
#include <sys/time.h>					// gettimeofday
#include <sys/types.h>					// socket
#include <sys/socket.h>		 			// socket


/* ************************************************************************** */
/* Constants definitions																		*/
/* ************************************************************************** */
#define MAX_STRING_SIZE		1024
#define MAXDATASIZE 1024 				// max number of bytes we can get at once 

const long WAIT_TIME = 50000;			// Sends each 500 miliseconds

// CBR CONSTANTS
int inputlength = 0;
int outputlength = 3;

#define LIMITPOS(a,b)	( ( a < 0.0f) ? 0.0f : ( ( a > b) ?	b :  a) )
#define LIMITSAT(a,b)	( ( a < -b) ? -b : ( ( a > b) ?	b :  a) )

/* ************************************************************************** */
/* Global variables definitions																*/
/* ************************************************************************** */

// ----- DLA PARAMETERS -----
static int Num_Sonars;							// number of sonars of the robot
static char Host_Cbr[MAX_STRING_SIZE+1];	// host where is located the CBR Server
static int Port_Cbr;								// port where is located the CBR Server

int *state;	// FLAG (int) -> indica si los valores de state son correctos
				// ANG_SONARS[NUM_SONARS] (int deg*10) +
				// SONARS[NUM_SONARS] (int mm) +
				// POS_X (int mm) + POS_Y (int mm) +
				// HEADING (int deg*10) +
				// COMPASS (int deg*10) +
				// VEL_TRANS (int mm/s) +
				// VEL_ROT (int deg*10/s) +
				// JOY_TRANS (int %) +
				// JOY_ROT (int %) +
				// STATUS (int bits)
				// TIME_SEC (int) -> para medir la latencia del sistema
				// TIME_USEC (int) -> para medir la latencia del sistema

Connection *state_conn;
Connection *track_conn;
Connection *alt_command_conn;

Mailbox *init_mail;
int track[3 + (2)];
int alt_comm[3];
int incr_x,incr_y,distance;

struct timeval time_begin,time_end;
struct timezone tz;
float time_diff;

char aux_string[MAX_STRING_SIZE+1];
char caseInput[MAX_STRING_SIZE];
char caseInputTmp[MAX_STRING_SIZE];
		
		
/* ************************************************************************** */
/* 	Show usage program and read arguments passed to it								*/
/* ************************************************************************** */
void usage_And_Read_Arguments(int argc, char *argv[],char *Host_Dla, int *Port_Dla);

/* ************************************************************************** */
/* 	Initializes most of the variables of the program and show messages on	*/
/* 	console if there are problems with the initialization.						*/
/* ************************************************************************** */
int initParameters(int argc,char *argv[], char *Host_Dla, int *Port_Dla);

/* ************************************************************************** */
/* 	Actives kill signal to be caught in order to finish the program 			*/
/* correctly closing all files and conections.											*/
/* ************************************************************************** */
int activeKillSignals();

/* ************************************************************************** */
/* Function called when signal kill is catch. This function close all files	*/
/* and connections opened.																		*/
/* ************************************************************************** */
void quitProgram(int sig);

/* ************************************************************************** */
/* Function called for sending and receiving cbr case									*/
/* ************************************************************************** */
void txrxCBR(char *envio,float* caseOutput);

void startTimer() { gettimeofday(&time_begin, &tz); };
void stoptTimer() { gettimeofday(&time_end, &tz); };
float getDiffTimer()
{	/// Returns time elapsed between call to startTimer and stopTimer in miliseconds
	time_diff = (float)(time_end.tv_sec - time_begin.tv_sec) * 1000.0f + (float)(time_end.tv_usec - time_begin.tv_usec)/1000.0f;
	return (time_diff);
}

/* ************************************************************************** */
/* ************************************************************************** */
/* *************** MAIN FUNCTION ******************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
int main(int argc, char *argv[])
{
	float caseOutput[MAXDATASIZE*2];	//
	float xJoy=0.0f,yJoy=0.0f;							// axis x and y in float format between -1.0 and 1.0
	char Host_Dla[MAX_STRING_SIZE+1];				// host where is located the DLA Server
	int Port_Dla;											// port to access the DLA Server

	if(!activeKillSignals())	return (-1);
	if (!initParameters(argc,argv, Host_Dla, &Port_Dla))	return (-1);
	
	printf(" ");
	if ( !openCBR(Host_Cbr,Port_Cbr) )
	{
		printf("<ERROR> Couldn't open socket.\n");
		exit(0);
	}
	else
		printf("Connected to CBR server %s port: %d\n", Host_Cbr, Port_Cbr);
	
	for (;;)
	{
		usleep(WAIT_TIME);
	
		InputConnection(state_conn, state, 0, 0);
		InputConnection(track_conn, track, 0, 0);
		InputConnection(alt_command_conn, alt_comm, 0, 0);
	
		if (alt_comm[0] == 0)
		{	// if flag is down, updates connection data with a new value
			incr_x = track[1] - state[1 + (2 * Num_Sonars)+0];
			incr_y = track[2] - state[1 + (2 * Num_Sonars)+1];
			
			memset(caseInput,0,MAX_STRING_SIZE);
			memset(caseInputTmp,0,MAX_STRING_SIZE);
				
			int showcbr = 0;
			if (showcbr) printf("-> ");
			int cont_for = 0;
			for ( cont_for = 0; cont_for < Num_Sonars; cont_for++ )
			{
				//sprintf(aux_string, "%04d.", state[1 + Num_Sonars + cont_for]);
				float tmp = (float)(state[1 + Num_Sonars + cont_for]);
				if (tmp < 0.0f) tmp = 0.0f;
				if (tmp > 5000.0f) tmp = 5000.0f;
				sprintf(caseInputTmp, " %4.1f", tmp);
				if (showcbr) printf("[%s] ",caseInputTmp);
				strcat(caseInputTmp,"\n");
				strcat(caseInput, caseInputTmp);
			}
			sprintf(caseInputTmp, " %4.1f", (float)(incr_x));	// Posicion del target parcial
			if (showcbr) printf("[%s] ",caseInputTmp);
			strcat(caseInputTmp,"\n");
			strcat(caseInput, caseInputTmp);
			sprintf(caseInputTmp, " %4.1f", (float)(incr_y));
			if (showcbr) printf("[%s]\n",caseInputTmp);
			strcat(caseInputTmp,"\n");
			strcat(caseInput, caseInputTmp);
			//printf("dist_sensors2 = \n%s\n",caseInput);	

			txrxCBR(caseInput,caseOutput);

			if (showcbr)
			{
				printf("<- ");
				int i = 0;
				for(i = 0;i<(inputlength + outputlength);i++)
				{
					if ( i == inputlength)
						printf(" - ");
					printf("[%4.1f]  ",caseOutput[i]);
				}
				printf("\n");
			}
			xJoy = caseOutput[inputlength]/1000.0f;
			yJoy = -caseOutput[inputlength+1]/1000.0f;
			int numCase = (int)(caseOutput[inputlength+2]);
			printf("JOY_CBR = [%4.1f,%4.1f] %s CASE: %d\n",xJoy,yJoy,(yJoy > 0)?"I":"D", numCase);

			///OutputConnection(motors_conn, &motorsDla, 0, 0);
			xJoy = LIMITPOS(xJoy,1.0f);				// Traslational velocity limited between 0 and 1
			yJoy = LIMITSAT(yJoy,1.0f);				// Rotational velocity limited between -1 and 1.0
			/// XXX
			//printf("pos = (% 3d,% 3d) track = (% 3d,% 3d) incr = (% 3d,% 3d)\n",state[1 + (2 * Num_Sonars)+0], state[1 + (2 * Num_Sonars)+1], track[1],track[2], incr_x, incr_y);		//MMM
			alt_comm[0] = 1;
			alt_comm[1] = (int) rint(100.0f*xJoy);
			alt_comm[2] = (int) rint(100.0f*yJoy);
			printf("Enviando Comando [%d](%3d, %3d)\n",alt_comm[0],alt_comm[1],alt_comm[2]);
			OutputConnection(alt_command_conn, &alt_comm, 0, 0);	
		}
	}
}

/* ************************************************************************** */
/* Function:																						*/
/* 	void initParameters(int argc,char *argv)											*/
/* -------------------------------------------------------------------------- */
/* Description:																					*/
/* 	Initializes most of the variables of the program and show messages on	*/
/* 	console if there are problems with the initialization.						*/
/* -------------------------------------------------------------------------- */
/* Parameters: 																					*/
/* 																									*/
/* -------------------------------------------------------------------------- */
/* Returned value:																				*/
/* 	1 on success, 0 on error																*/
/* ************************************************************************** */
int initParameters(int argc,char *argv[], char *Host_Dla, int *Port_Dla)
{
	int val_init_mail;							// Value passed to mail without any posterior use
	int i;											// Auxiliar variable for loops
	//char aux_string[MAX_STRING_SIZE*2];		// Auxiliar variable 
	//char aux_string2[MAX_STRING_SIZE+1];	// Auxiliar variable 
	int val_param_mail;
	Connection *param_conn;
	char *Table_Parameter;
	
	usage_And_Read_Arguments(argc, argv,Host_Dla, Port_Dla);
	
	// READ DLA PARAMETERS
	Mailbox *param_mail = NewMailbox(Host_Dla, *Port_Dla, "param");
	WaitMailbox(param_mail, &val_param_mail);
	
	CloseMailbox(param_mail);
	//printf("val_param_mail = %d\n",val_param_mail);
	if ( (Table_Parameter = (char *)malloc(val_param_mail)) == NULL )
	{
		printf("malloc(%d)\n",val_param_mail);
		perror("ERROR [main] (malloc:Table_Parameter)");
		return(0);
	}
	
	param_conn = NewConnection(Host_Dla, *Port_Dla, "param", val_param_mail);
	InputConnection(param_conn, Table_Parameter, 0, 0);
	CloseConnection(param_conn);
	// READ PARAMS FROM TABLE_PARAMETER STORE ON DLA FROM START MODULE
	if (!getParameteri("NUM_SONARS",&Num_Sonars,Table_Parameter)) 					return(0);
	if (!getParameters("HOST_CBR",(char *)(&Host_Cbr),Table_Parameter))			return(0);
	if (!getParameteri("PORT_CBR",&Port_Cbr,Table_Parameter)) 						return(0);
	if (!getParameteri("NUMCBRINPUTS",&inputlength,Table_Parameter))	return(0);
	printf("HOST_CBR = %s\nPORT_CBR = %d\n", Host_Cbr, Port_Cbr);
	printf("NumCbrInputs = %d NumCbrOutputs = %d\n", inputlength, outputlength);
			
	//char parameter[MAX_STRING_SIZE+1];
	char num_sonar_string[3+1];
	for (i=0;i<4;i++)
		num_sonar_string[i] = 0;
	if ( (state = (int *)malloc((1 + (2 * Num_Sonars) + 4 + 5) * sizeof(int) + (2) * sizeof(int))) == NULL )
	{
		perror("ERROR [Navigation] (malloc:state)");
		return(0);
	}

	state_conn = NewConnection(Host_Dla, *Port_Dla, "state", (1 + (2 * Num_Sonars) + 4 + 5) * sizeof(int) + (2) * sizeof(int));
	track_conn = NewConnection(Host_Dla, *Port_Dla, "track", 3 * sizeof(int) + (2) * sizeof(int));
	alt_command_conn = NewConnection(Host_Dla, *Port_Dla, "alt_command", 3 * sizeof(int));
	alt_comm[0] = 0;
	alt_comm[1] = 0;
	alt_comm[2] = 0;
	OutputConnection(alt_command_conn, &alt_comm, 0, 0);	

	// WAIT STATE INITIALIZATION
	init_mail = NewMailbox(Host_Dla, *Port_Dla, "init");
	WaitMailbox(init_mail, &val_init_mail);
	CloseMailbox(init_mail);
	
	// READ FIRST POSITION
	InputConnection(state_conn, state, 0, 0);
	
	return(1);
}

/* ************************************************************************** */
/* Function:																						*/
/* 	int activeKillSignals()																	*/
/* -------------------------------------------------------------------------- */
/* Description:																					*/
/* 	Actives kill signal to be caught in order to finish the program 			*/
/* correctly closing all files and conections.											*/
/* -------------------------------------------------------------------------- */
/* Parameters: 																					*/
/* -------------------------------------------------------------------------- */
/* Returned value:																				*/
/* - 0 on error, 1 on success																	*/
/* ************************************************************************** */
int activeKillSignals()
{
	if ( signal(SIGINT, quitProgram) == SIG_ERR )
	{
		perror("\nERROR [main] (signal:SIGINT)");
		return(0);
	}
	if ( signal(SIGQUIT, quitProgram) == SIG_ERR )
	{
		perror("\nERROR [main] (signal:SIGQUIT)");
		return(0);
	}
	if ( signal(SIGTERM, quitProgram) == SIG_ERR )
	{
		perror("\nERROR [main] (signal:SIGQUIT)");
		return(0);
	}
	return (1);
}

/* ************************************************************************** */
/* Function:																						*/
/* 	void quitProgram(int signal)															*/
/* -------------------------------------------------------------------------- */
/* Description:																					*/
/* Function called when signal kill is catch. This function close all files	*/
/* and connections opened.																		*/
/* -------------------------------------------------------------------------- */
/* Parameters: 																					*/
/* - sig: Value of the signal catch.														*/
/* -------------------------------------------------------------------------- */
/* Returned value:																				*/
/* ************************************************************************** */
void quitProgram(int sig)
{
	if ( !closeCBR() )
		printf("<ERROR> Couldn't close CBR Server.\n");
	else
		printf("CBR Server closed correctly.\n");
	//	signal(sig, quitProgram);
	//time_t time_calendar;
	//struct tm time_universal;
	
	CloseConnection(state_conn);
	CloseConnection(track_conn);
	CloseConnection(alt_command_conn);
	exit(0);
}

void txrxCBR(char *envio,float* caseOutput)
{
	char aux_string[MAX_STRING_SIZE];
	memset(aux_string,0,MAX_STRING_SIZE);
	int i = 0;
	for (i = 0;i<inputlength+outputlength;i++)
		caseOutput[i] = -1.0;
	
	strcpy(aux_string,envio);
	//printf("Enviando: \n%s",aux_string);	
	
	////////////////// SENDING DATA TO CBR SERVER ///////////////////////
	if ( !txCBR(aux_string))
		printf("Error enviando datos.\n");

	////////////////// RECEIVING DATA TO CBR SERVER ///////////////////////
	int bytes = 0;
	if ( (bytes = rxCBR(aux_string,MAXDATASIZE)) == 0 )
		printf("Error recibiendo datos.\n");
	//printf("Rx = %d bytes\n", bytes);
	
	char * tmpPtr = aux_string;
	float f = 0.0f;
	//printf("*  ");
	for (i = 0;i<inputlength+outputlength;i++)
	{
		sscanf(tmpPtr++,"%f\n",&f);
		tmpPtr = strstr(tmpPtr, "\n");
		//printf("[%4.1f]  ", f);
		caseOutput[i] = f;
	}
}

/* ************************************************************************************** */
/* Function:																										*/
/* 	void usage_And_Read_Arguments(int argc, char *argv[],char *Host_Dla, int *Port_Dla) */
/* -------------------------------------------------------------------------------------- */
/* Description:																									*/
/* 	Show usage program and read arguments passed to it												*/
/* -------------------------------------------------------------------------------------- */
/* Parameters: - NONE.																							*/
/* -------------------------------------------------------------------------------------- */
/* Returned value: - NONE. 																					*/
/* ************************************************************************************** */
void usage_And_Read_Arguments(int argc, char *argv[],char *Host_Dla, int *Port_Dla)
{
	if ( argc == 1 )
	{
		printf("DLA Server: ");
		scanf("%s", Host_Dla);
		if ( strcasecmp(Host_Dla, "local") != 0 )
		{
			printf("DLA Port  : ");
			scanf("%s", aux_string);
			*Port_Dla = atoi(aux_string);
		}
		else
			*Port_Dla = 0;
	}
	else if ( argc == 2 )
	{
		strcpy(Host_Dla, argv[1]);
		printf("DLA Server %s\n", Host_Dla);
		if ( strcasecmp(Host_Dla, "local") != 0 )
		{
			printf("DLA Port  : ");
			scanf("%s", aux_string);
			*Port_Dla = atoi(aux_string);
		}
		else
			*Port_Dla = 0;
	}
	else if ( argc >= 3 )
	{
		strcpy(Host_Dla, argv[1]);
		printf("Server: %s\n", Host_Dla);
		if ( strcasecmp(Host_Dla, "local") != 0 )
		{
			printf("Port  : ");
			*Port_Dla = atoi(argv[2]);
			printf("%d\n", *Port_Dla);
		}
		else
			*Port_Dla = 0;
	}
	printf("\n");
}
