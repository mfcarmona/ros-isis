# Desc: Device definitions for CARMEN wheelchair
#       based on pioneer.inc from Vaughan et al.
###############################


define carmen_base position 
(
  color "blue"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  ranger_return 0.5            	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  # alternative odometric localization with simple error model
  # does not work with stage 4
  #localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  #odom_error [ 0.0 0.0 0.0 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   
)



define carmen_base_lasers carmen_base
(
 # Actual size
  size [1 0.6 0.4]

  # object movement center (x y z t)
  # origin [0.16 0.0 0.0 0.0] # this is its real center ... 
  origin [0.25 0.0 0.0 0.0]   # this actually works

  # Estimated mass in KG
  mass 50.0 
  
  # frontal is ranger:0 and  rear is ranger:1
  hokuyo_laser( pose [  0.75 0 0.0    0 ] color "black") 
  hokuyo_laser( pose [ -0.3  0 0.0 -180 ] color "red"  ) 
 )
 
define carmen_wheelchair carmen_base_lasers
(   
  # Body shape:
  bitmap "./bitmaps/carmen_shadow.png"
)
