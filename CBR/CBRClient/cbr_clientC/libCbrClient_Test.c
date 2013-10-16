#include <stdio.h>
#include "libCbrClient.h"

#define MAXDATASIZE 100 				// max number of bytes we can get at once 

/* MAIN LOOP */
int main(int argc, char *argv[])
{
	int inputlength = 11;
	int outputlength = 3;
	char host[100];				// host where is located the CBR Server	
	int port = 5000;
	char aux_string[100];
	char tmp[10][inputlength];
	int i = 0;
	float caseOutput[inputlength+outputlength];
	strcpy(host,"172.16.75.90");	// host server address

	
	if (argc > 2)
	{
	    fprintf(stderr,"usage: client hostname\n");
	    exit(1);
	}
	else if (argc == 2)
	{
		port = atoi(argv[1]);
	}
	if ( !openCBR(host,port) )
	{
		printf("Error opening socket.\n");
		exit(0);
	}
	
	for(i = 0;i<10;i++)
		memset(tmp[i],0,inputlength);

	//5000 1214 2185 5000 3748 2999 5000 2424 2413 1958 5000 330 -1000 326
	//5000 1027 1076 1286 1864 5000 1129 1116 1009 924 5000 500 259 6
/*
	strcpy(tmp[0],"5000");		strcat(tmp[0],"\n");
	strcpy(tmp[1],"1027");		strcat(tmp[1],"\n");
	strcpy(tmp[2],"1076");		strcat(tmp[2],"\n");
	strcpy(tmp[3],"1286");		strcat(tmp[3],"\n");
	strcpy(tmp[4],"1864");		strcat(tmp[4],"\n");
	strcpy(tmp[5],"5000");		strcat(tmp[5],"\n");
	strcpy(tmp[6],"1129");		strcat(tmp[6],"\n");
	strcpy(tmp[7],"1116");		strcat(tmp[7],"\n");
	strcpy(tmp[8],"1009");		strcat(tmp[8],"\n");
	strcpy(tmp[9],"924");		strcat(tmp[9],"\n");
	strcpy(tmp[10],"5000");		strcat(tmp[10],"\n");
	
/**/
	strcpy(tmp[0],"5000");		strcat(tmp[0],"\n");
	strcpy(tmp[1],"1214");		strcat(tmp[1],"\n");
	strcpy(tmp[2],"2185");		strcat(tmp[2],"\n");
	strcpy(tmp[3],"5000");		strcat(tmp[3],"\n");
	strcpy(tmp[4],"3748");		strcat(tmp[4],"\n");
	strcpy(tmp[5],"2999");		strcat(tmp[5],"\n");
	strcpy(tmp[6],"5000");		strcat(tmp[6],"\n");
	strcpy(tmp[7],"2424");		strcat(tmp[7],"\n");
	strcpy(tmp[8],"2413");		strcat(tmp[8],"\n");
	strcpy(tmp[9],"1958");		strcat(tmp[9],"\n");
	strcpy(tmp[10],"5000");		strcat(tmp[10],"\n");
/**/	

	////////////////// SENDING DATA TO CBR SERVER ///////////////////////
	for(i = 0;i<inputlength;i++)
	{
		strcpy(aux_string,tmp[i]);		
		printf("Enviando: %s - %s",tmp[i],aux_string);
		if ( !txCBR(aux_string))
			printf("Error enviando datos.\n");
	}	
	////////////////// RECEIVING DATA TO CBR SERVER ///////////////////////
	//for(i = 0;i<(inputlength + outputlength);i++)
	//{
		if ( !rxCBR(aux_string,MAXDATASIZE))
			printf("Error recibiendo datos.\n");
	//}	
	//printf("aux_string = %s\n",aux_string);
	sscanf(aux_string,"%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",&caseOutput[0],&caseOutput[1],&caseOutput[2],&caseOutput[3],&caseOutput[4],&caseOutput[5],&caseOutput[6],&caseOutput[7],&caseOutput[8],&caseOutput[9],&caseOutput[10],&caseOutput[11],&caseOutput[12],&caseOutput[13]);
	printf("Read data: ");
	for(i = 0;i<(inputlength + outputlength);i++)
		printf("%0.1f ",caseOutput[i]);
	printf("\n\n");
	//5000 1027 1076 1286 1864 5000 1129 1116 1009 924 5000 500 259 6
	//printf("Deberia recibir: 5000 1027 1076 1286 1864 5000 1129 1116 1009 924 5000 500 259 6\n");
	printf("Deberia recibir: 5000 1214 2185 5000 3748 2999 5000 2424 2413 1958 5000 330 -1000 326 \n");
	//printf("dormiendo\n");
	//getchar();
	//usleep(2000000);
	//printf("despierto\n");
	if ( !closeCBR() )
		printf("Error cerrando socket.\n");
	printf("Fin %s\n",argv[0]);
	return 0;
}
