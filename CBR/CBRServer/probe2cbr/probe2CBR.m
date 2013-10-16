function [] = probe2CBR(filenameProbe,filenameCBR, nOutputSensors);
% Función que pasa de formato probe a formato CBR
% fprintf(1,'Usage: probe2CBR(filenameprobe,filenamecbr\n')
% lee datos de probes y guarda formato CBR con numOutputSensors +2 datos: numOutputSensors sensores del sonar y partial target. Como salida la entrada y xjoy, yjoy y case

if (nargin == 0)
	help probe2CBR
	disp('');
	return;
end;

showinfo = 0;	% 1 = show info on console

NumSensors = 43;

numOutputSensors = -1;

if ( nargin ~= 2 && nargin ~= 3)
	fprintf(1,'Error: Not enough input parameters.\n');
	fprintf(1,'Usage: probe2CBR(filenameProbe,filenameCBR)\n');
	fprintf(1,'       probe2CBR(filenameProbe,filenameCBR, nCbrInputs)\n');
	return;
end

if ( nargin == 3 )
	numOutputSensors = nOutputSensors;
	if (nOutputSensors <= 2)
		fprintf(1,'Error: Number of outputs(%d) cannot be lower than 2\n', nOutputSensors);
		return;
	end
end


if ( ~fileattrib(filenameProbe,'+w'))	% 0 si el fichero no existe
	fprintf(1,'El fichero de entrada "%s" no existe.\n',filenameProbe)
	return;
end

[NumSensors,xjoy, yjoy, xReactive,yReactive, pos_x,pos_y,SOFT_Reactive,DIST_Reactive,SEC_Reactive,GLOB_Reactive,SOFT_Joy,DIST_Joy,SEC_Joy,GLOB_Joy,xComp,yComp,SOFT_Comp,DIST_Comp, SEC_Comp, GLOB_Comp, k, time_usec, angles,valSensors, distanceCovered, data_trace,targetx, targety] = Get_data(filenameProbe);
if ( numOutputSensors <= 2 || numOutputSensors > numOutputSensors)
	numOutputSensors = NumSensors;	% partial targetx and y (target - pos)
end
numCbrInputs = numOutputSensors+ 2;	% partial targetx and y (target - pos)

%targetx'
%targety'

fprintf(1,'Converting from "%s" (%d sensors) to "%s" (%d sensors)\n',filenameProbe,NumSensors,filenameCBR, numOutputSensors);

generateCBRDescriptor([filenameCBR '.descriptor'], numCbrInputs, 3);

filenameCBR_ID = 1;
if ( length(filenameCBR) ~= 0)
	filenameCBR_ID = fopen(filenameCBR,'wt');
end;
beginCBR(filenameCBR_ID,numCbrInputs,3);

num_total_datas= size(pos_x,1);
if ( numOutputSensors < NumSensors )
	step = -1125*2 / (numOutputSensors-1);
	angles_red = [1125:step:-1125];
end
for (muestra = 3:num_total_datas)
	if ( numOutputSensors == NumSensors)
		outputValSensors = valSensors(muestra,:);
	else
		[ang,outputValSensors] = reduceNumberOfSensors(angles_red,angles(muestra,:),valSensors(muestra,:),5000);
	end
	%fprintf(1,'angles = \n', 1); ang
	%fprintf(1,'valSensors = \n', 1); outputValSensors
	if ( (xjoy(muestra) ~= 0 ) && (yjoy(muestra) ~= 0 ) )
		saveCaseCBR(filenameCBR_ID,[ outputValSensors targetx(muestra)-pos_x(muestra) targety(muestra)-pos_y(muestra)],[xjoy(muestra) yjoy(muestra) ],muestra);		
	end;
	%pause(0.04);	%pause;
end

closeCBR(filenameCBR_ID);

