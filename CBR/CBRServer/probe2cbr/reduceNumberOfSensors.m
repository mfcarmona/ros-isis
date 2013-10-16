function [angles_red,dist_red] = reduceNumberOfSensors(vectorAngles,angles,dist,maximun)
% Funcion que ajusta los valores de los angulos y sus correspondientes distancias al vector de angulos dados.
% Es decir, si tenemos 22 sensores y queremos diezmar a 11, esta funcion lo hace cogiendo el minimo de cada distancia
% para el angulo correspondiente
% Los angulos seran equiespaciados, simetricos y ordenados de mayor a menor

if ( nargin ~= 3 && nargin ~= 4 )
	fprintf(1,'Usage: [angles_red,dist_red = redAngles(vectorAngles,angles,dist)\n');
	return;
end

vectorAngles = sort(vectorAngles);
angles_red = vectorAngles;								% Vector de angulos reducidos. Igual al de entrada, pues este indica los angulos finales
incr = (vectorAngles(2) - vectorAngles(1)) / 2;	% incremento de los angulos
dist_red = zeros(1,length(angles_red));			% Vector con la distancia minima correspondiente a cada angulo.
dist_red(:) = inf;										% Por defecto distancia infinita

%angles
%dist

for ( i = 1:length(angles) )
	for ( j = 1:length(angles_red) )
		if ( (angles(i) > angles_red(j) - incr) && (angles(i) <= angles_red(j) + incr) )
			if ( dist_red(j) > dist(i) )
				if ( dist(i) > 0)
					dist_red(j) = dist(i);
				end
			end
		break;
		end
	end
end


if ( nargin == 4 )
	for (i = 1:length(dist_red) )
		if (dist_red(i) > maximun)
			dist_red(i) = maximun;
		end
	end
end

