function[NumSensors,xJoy, yJoy,xReactive,yReactive,pos_x,pos_y,SOFT_Reactive,DIST_Reactive,SEC_Reactive,GLOB_Reactive,SOFT_Joy,DIST_Joy,SEC_Joy,GLOB_Joy,xComp,yComp,SOFT_Comp,DIST_Comp, SEC_Comp, GLOB_Comp, k, time_usec, angles,dist_angles, distance, data_trace,target_x, target_y] = Get_data(file_name)
% function [pos_x,pos_y,SOFT_Reactive,DIST_Reactive,SEC_Reactive,GLOB_Reactive,SOFT_Joy,DIST_Joy,SEC_Joy,GLOB_Joy,xComp,yComp,SOFT_Comp,DIST_Comp, SEC_Comp, GLOB_Comp, k, time_usec, angles,dist_angles, distance, data_trace] = Get_data(file_name)

if ( (nargin ~= 1) && (nargin ~= 2) )
	fprintf(1,'Usage: Get_Data(data_file)\n');
	return;
end

NumSensors = 43;

%fprintf(1,'\n<Get_Data> Reading data from %s.\n',file_name)
%pause


col = 1;

data_trace = load(file_name);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DETERMINACI�N DEL N�MERO DE SENSORES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Si hay 11 sensores, el numero de datos son 48
%% posx  posy  heading targetx targety angle xreac yreac->soft dist sec glob xjoy  yjoy->soft dist sec  glob xComp yComp->soft dist sec glob   k    time        angulos                                                  distancias -> angulos                                                              
% Si no, son 112-113 dependiendo de si son las pruebas de junio o julio (las de julio tienen un dato fantasma)
tamData = size(data_trace);

if (tamData(2) == 48)
	NumSensors = 11;
	fprintf(1,'Processing data from simulator with 11 sensors.\n');
end
if (tamData(2) == 58)
	fprintf(1,'Processing data from simulator with 11 sensors and extra data for CBR.\n');
end
if (tamData(2) == 112)
	fprintf(1,'Processing data from June 07 experiments.\n');
end
if (tamData(2) == 113)
	fprintf(1,'Processing data from July 07 experiments with extra data.\n');
end
if (tamData(2) > 115)
	fprintf(1,'Processing data from October 08 experiments.\n');
end		


%  pause;
num_total_datas = size (data_trace,col);
%  pause;
pos_x = data_trace(1:end,col);
col = col + 1;
%  pause;
pos_y = data_trace(1:end,col);
col = col + 1;
%  pause;
heading = data_trace(1:end,col);
col = col + 1;
%  pause;
target_x = data_trace(1:end,col);
col = col + 1;
%  pause;
target_y = data_trace(1:end,col);
col = col + 1;
%  pause;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DATO FANTASMA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% En los experimentos de julio hay un dato fantasma justo delante del angle (o eso juraria yo....)
tamData = size(data_trace);
if (tamData(2) == 113)
	col = col + 1;
end


angle = data_trace(1:end,col);
col = col + 1;
%  pause;
xReactive = data_trace(1:end,col);
col = col + 1;
%  pause;
yReactive = data_trace(1:end,col);
col = col + 1;

SOFT_Reactive = data_trace(1:end,col);
col = col + 1;
DIST_Reactive = data_trace(1:end,col);
col = col + 1;
SEC_Reactive = data_trace(1:end,col);
col = col + 1;
GLOB_Reactive = data_trace(1:end,col);
col = col + 1;
xJoy = data_trace(1:end,col);
col = col + 1;
yJoy = data_trace(1:end,col);
col = col + 1;
SOFT_Joy = data_trace(1:end,col);
col = col + 1;
DIST_Joy = data_trace(1:end,col);
col = col + 1;
SEC_Joy = data_trace(1:end,col);
col = col + 1;
GLOB_Joy = data_trace(1:end,col);
col = col + 1;
% En los experimentos de octubre se a�adieron mas columnas
% xCbr yCbr->soft  dist sec glob tSec tGlob case 0Cbr 
% Concretamente el formato entero es:
%%  posx   posy heading targetx targety angle xreac yreac->soft dist sec glob  xjoy  yjoy->soft dist sec glob xCbr yCbr->soft  dist sec glob tSec tGlob case 0Cbr xComp yComp->soft dist sec glob   k    time        angulos                                                  distancias -> angulos                                                              
if (tamData(2) > 115)
	col = col + 10;
end

xComp = data_trace(1:end,col);
col = col + 1;
yComp = data_trace(1:end,col);
col = col + 1;
SOFT_Comp = data_trace(1:end,col);
col = col + 1;
DIST_Comp = data_trace(1:end,col);
col = col + 1;
SEC_Comp = data_trace(1:end,col);
col = col + 1;
GLOB_Comp = data_trace(1:end,col);
col = col + 1;
k = data_trace(1:end,col);
col = col + 1;
time_usec = data_trace (1:end,col);
col = col + 1;
angles = data_trace (1:end, col + 0 : col + NumSensors - 1);
dist_angles = data_trace (1:end, col + NumSensors + 0: col + 2*NumSensors -1);

%for (cont_data = 2:num_total_datas)
%	angles(cont_data,:);
%	dist_angles(cont_data,:);
%	pause
%end


distance = zeros (num_total_datas,1);
distance(1) = sqrt(pos_x(1)*pos_x(1) + pos_y(1)*pos_y(1));
for (cont_data = 2:num_total_datas)
    incr_x = (pos_x(cont_data)-pos_x(cont_data-1));
	incr_y = (pos_y(cont_data)-pos_y(cont_data-1));
	distance(cont_data) = distance(cont_data-1) + sqrt( (incr_x*incr_x) + (incr_y*incr_y));
end

%fprintf(1,'\n<Get_Data> Ended.\n',file_name)

%med = 0;
%for (i=1:length(k))
%	med = med + k(i);
%end
%
%	med = med / length(k);
%med
%pause
