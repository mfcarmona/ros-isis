
define hokuyo_laser laser
(
  # laser-specific properties
  # factory settings for urg04lx
  range_max 4.0
  fov 270.0
  samples 768

  #samples 90 # still useful but much faster to compute

  # generic model properties
  color "black"
  size [ 0.056 0.055 0.08 ] # dimensions from LMS200 data sheet	
)




