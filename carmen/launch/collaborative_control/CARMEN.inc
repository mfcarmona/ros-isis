
# Desc: Device definitions for wheelchair CARMEN
# Author: Manuel Fdez Carmona
# Based on pioneers.inc

# CARMEN standard configuration
define carmen position
(

  # Default color.
  color "blue"			
  
  # Differential steering model.
  drive "diff"		 	
  
  # Draw a nose so we can see which way it points
  gui_nose 1                  	
  
  # Can hit things.
  obstacle_return 1           	
  
  # reflects laser beams
  laser_return 1        
          
  # reflects sonar beams 
  ranger_return 1        
       	
  # Seen by blobfinders  
  blob_return 1               	
  
  # Seen as "1" fiducial finders
  fiducial_return 1           	
    	  
  # Start odometry at (0, 0, 0).    	  
  # localization_origin [0 0 0 0] 	

  # alternative odometric localization with simple error model
  localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution) 

  # Actual size
  size [1 0.6 0.4]

  # centro del objeto, con respecto a su posicion (x y z t)
  # origin [0.16 0.0 0.0 0.0] no queda bien asi... aunque es su centro autentico ...
  origin [0.25 0.0 0.0 0.0]

  # Estimated mass in KG
  mass 50.0 

  # laser frontal, se supone que el real esta boca abajo ...
   hokuyo_urg(   
	 pose [ 0.75 0 0.0 0 ]
     # ctrl "lasernoise"  # uncomment this line to run a laser noise generator
   )

  # laser trasero   
  hokuyo_urg( 
   	pose [ -0.3 0 0.0 -180 ]  
   	color "red"
        # ctrl "lasernoise"  # uncomment this line to run a laser noise generator
     )

  # Body shape:
 bitmap "./carmen_shadow.png"

  #block( 
  #  points 4
  #  # x y desde su posicion, puntos van counter clock wise
  #  point[0] [0.3   0.5]
  #  point[1] [-0.3   0.5]
  #  point[2] [-0.3   -0.5]
  #  point[3] [0.3   -0.5]
  #  # elevacion en z
  #  z [0 0.02]
  #)
  
)







