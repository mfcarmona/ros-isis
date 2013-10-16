# Desc: Hokuyo definition
#       Based upon sick.inc from Vaughan
# Author:  Manuel Fdez.

define hokuyo_sensor sensor
(				  
  range [ 0.2 4.0 ]
  pose [ 0 0 0.1 0 ]
  fov 270.0
  samples 768
  #samples 90 # still useful but much faster to compute
  color_rgba [ 0 0 1 0.15 ]
)

define hokuyo_common ranger
(
  color "blue"
  size [ 0.056 0.055 0.08  ] 	
 )

define hokuyo_base hokuyo_common
(
	block( 
         points 4
	 point[0] [0 0]
	 point[1] [0 1]
	 point[2] [1 1]
	 point[3] [1 0]
	 z [0 0.21]
	 )
)

define hokuyo_laser hokuyo_base ( hokuyo_sensor() )


