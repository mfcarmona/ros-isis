function [] = beginCBR(file_id,numInput,numOutput)
% Funcion que crea la cabecera del fichero CBR acorde al numInputs y numOutputs
% Como datos de entrada recibe el nombre de la base de datos y  el numero de entradas y salidas del vector de entrada/salida respectivamente.
% Como salida genera la cabecera del CBR y como nombres de las variables de entrada tendra INXXX y como salida OUTXXX con XXX un numero

% Guarda cabecera del CBR
fprintf(file_id,'CASELIB\n\nHEAD\nCI CARMEN\nET CARMEN_PLAYS\nINPUTP_ORDER\n');
% Nombre de cada variable de entrada del CBR
for( i=0:numInput-1 )
	fprintf(file_id,'P In%d\n',i);
end;
fprintf(file_id,'ENDINPUTP_ORDER\nTRAINEE CASE_ESPECIFIED\nOUTPUTP_ORDER\n');
% Nombre de cada variable de salida del CBR (vector de entrada mas el de salida propiamente dicho)
for( i=0:numInput-1 )
	fprintf(file_id,'P In%d\n',i);
end;
for( i=0:numOutput-1 )
	fprintf(file_id,'P Out%d\n',i);
end;
fprintf(file_id,'ENDOUTPUTP_ORDER\nENDHEAD\n\n\n');


% Comienzo cuerpo CBR
fprintf(file_id,'BODY\n\n');


