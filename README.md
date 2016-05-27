# mikrokopter_node
ROS node for interfacing the MikroKopter FlightCtrl

The implementation requires a special version of the FlightCtrl firmware, see: 
https://github.com/cehberlin/MikroKopterFlightController

The node includes the following features.

Services for:
* Preflight calibration and persisted calibration
* Arming/Disarming motors
* Trigger Beep sounds on the flightcontroller
* Execute motor tests

Topics for:
* Publishing general status informations 
  * Current stick and motor values
  * Neutral throttle point
  * External control state
  * Calibration state
  * Arming state
* Publishing battery status
* Subscribing to external roll, pitch, yaw, thrust commands
