# Command to control the copter movement
#
# Roll/Pitch value position coordinate influences
# Neutral value is 0, conversation is done in mikrokopter node
#
#        |
#      --|-->(+) Roll
#        v
#       (+)
#      Pitch
#
Header header
int8  roll   # roll value, range [-128, 127]
int8  pitch  # pitch value, range [-128, 127]
int8  yaw    # yaw value, range [-128, 127]
uint8 thrust # thrust value, range [0, 255]