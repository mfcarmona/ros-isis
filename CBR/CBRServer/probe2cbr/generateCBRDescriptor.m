function [] = generateCBRDescriptor(file_name,numInput,numOutput)
% Funcion que crea un descriptor de cbr acorde al numInputs y numOutputs

file_id = 1;
if ( length(file_name) ~= 0)
	file_id = fopen(file_name,'wt');
end;

fprintf(file_id,'\n');

% Guarda descriptor para cada input
for( i=0:numInput-1 )
	fprintf(file_id,'DESCRIPTOR\n');
	fprintf(file_id,'In%d\n',i);
	fprintf(file_id,'T S\n');
	fprintf(file_id,'U %% distancia recta origen (0,0) Puede ser positiva o negativa\n');
	fprintf(file_id,'V invalido			[ -2000, -2000,    -1] /\n');
	fprintf(file_id,'V near_1				[     0,   250,   250] /\n');
	fprintf(file_id,'V near_2				[   250,   500,   500] /\n');
	fprintf(file_id,'V near_3				[   500,   750, 	750] /\n');
	fprintf(file_id,'V near_4				[   750,  1000,  1000] /\n');
	fprintf(file_id,'V med_1				[  1000,  1250,  1250] /\n');
	fprintf(file_id,'V med_1				[  1250,  1500,  1500] /\n');
	fprintf(file_id,'V med_2				[  1500,  1750,  1750] /\n');
	fprintf(file_id,'V med_1				[  1500,  1750,  1750] /\n');
	fprintf(file_id,'V far_1				[  2000,  3000,  3000] /\n');
	fprintf(file_id,'V far_1				[  3000,  5000,  5000] /\n');
	fprintf(file_id,'Q Minima distancia de la recta al centro (0,0)\n');
	fprintf(file_id,'ENDDESCRIPTOR\n\n');
end;

%fprintf(file_id,'V near_0				[     0,   125,   125] /\n');
%fprintf(file_id,'V near_1				[   125,   250,   250] /\n');
%fprintf(file_id,'V near_2				[   250,   400,   400] /\n');
%fprintf(file_id,'V near_3				[   400,   650, 	650] /\n');
%fprintf(file_id,'V near_4				[   650,   900,   900] /\n');
%fprintf(file_id,'V med_1				[   900,  1250,  1250] /\n');
	

for( i=0:numOutput-2 )
	fprintf(file_id,'DESCRIPTOR\n');
	fprintf(file_id,'Out%d\n',i);
	fprintf(file_id,'T S\n');
	fprintf(file_id,'U %% Movimiento del usuario\n');
	fprintf(file_id,'V atras_derecha   [ -100, -50,  -1] /\n');
	fprintf(file_id,'V no              [   -1,   0,   1] /\n');
	fprintf(file_id,'V delante_izq     [  100,  50,   1] /\n');
	fprintf(file_id,'Q Movimiento delante detras. 0: No se mueve 1: Hacia delante -1: Hacia atras\n');
	fprintf(file_id,'ENDDESCRIPTOR\n');
end;

% Numero de caso
fprintf(file_id,'DESCRIPTOR\n');
fprintf(file_id,'Out%d\n',numOutput-1);
fprintf(file_id,'T S\n');
fprintf(file_id,'U integer\n');
fprintf(file_id,'V normal          [00000, 50000, 100000] /\n');
fprintf(file_id,'Q Case number in the CBR database.\n');
fprintf(file_id,'ENDDESCRIPTOR\n');

fclose(file_id);
