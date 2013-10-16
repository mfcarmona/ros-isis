function [] = saveCBR(file_id,caseInput,caseOutput,numCase)
% Funcion que almacena un caso CBR en la base de datos con nombre file_id
% Recibe como entrada el nombre del fichero de la base de datos, el vector de entrada, el vector de salida y el numero de caso correspondiente
% a este caso. Como salida guarda el caso. La salida esta formada por el vector de entrada, el de salida y al final el numero de caso.


fprintf(file_id,'CASE\n\n');

fprintf(file_id,'N %d\n',numCase);

fprintf(file_id,'INPUT\n');

for ( i = 1:length(caseInput) ) % Recorre todo el vector de entrada 
	fprintf(file_id,'%d ',caseInput(i));
end;
fprintf(file_id,'\n');

fprintf(file_id,'ENDINPUT\nTRAINEE\nENDTRAINEE\nOUTPUTP\n');

for ( i = 1:length(caseInput) ) % Recorre todo el vector de entrada 
	fprintf(file_id,'%d ',caseInput(i));
end;
for ( i = 1:length(caseOutput) ) % Recorre todo el vector de entrada 
	fprintf(file_id,'%d ',caseOutput(i));
end;

fprintf(file_id,'%d\n',numCase);

fprintf(file_id,'ENDOUTPUTP\nOUTPUTG\nENDOUTPUTG\nSOLUTION\nENDSOLUTION\nMOREINFO\n[T_PF_0001]\nENDMOREINFO\nENDCASE\n\n');
