function [] = closeCBR(file_id)
% Función que finaliza el cuerpo de la base de datos CBR en el fichero indicado
% Como datos de entrada recibe el nombre del fichero de la base de datos

% Fin cuerpo CBR
fprintf(file_id,'ENDBODY\n\n');
fprintf(file_id,'ENDCASELIB\n\n');
fclose(file_id);
