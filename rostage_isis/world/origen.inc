# un origen de coordenadas

define origen position ( 	
  color "white"  
  size [0.100 0.100 0.00]  
  #bounding box alrededor del modelo
  gui_outline 1
  # no se puede mover
  gui_move 0
  # se puede atravesar
  obstacle_return 0
  #no es detectable por otros trastos
  fiducial_return 0
  laser_return 0
  gripper_return 0
  #bitmap "./bitmaps/origen.png"
  block( 
    points 3
    # x y desde su posicion, puntos van counter clock wise
    point[0] [1   0]
    point[1] [0  -1]    
    point[2] [0   1]
    # elevacion en z
    z [0 0.02]
  )
)
