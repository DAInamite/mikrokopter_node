# Current motor status

Header header

int16 	stickPitch			# current pitch
int16 	stickRoll			# current roll
int16 	stickYaw			# current yaw
int16 	stickThrust			# current thrust
uint8 	motor1          	# front-motor, range [0, 255]
uint8 	motor2          	# rear-motor, range [0, 255]
uint8 	motor3          	# right-motor, range [0, 255]
uint8 	motor4          	# left-motor value, range [0, 255]
uint16 	hoverThrottle   	# neutral throttle point
bool 	externalControl		# true if external control via SERIAL is activated
bool 	calibrationDone		# true if calibration is already executed
bool    motorsArmed			# true if motors are armed
