# To run from ros the velocity control:
 - first run the launch file 
 
```
	roslaunch rtt_soem_maxpos maxpos.launch 
```
 - then 3 services are available:
 
 	- `rosservice call MaxPos/BringOperational` to switch on
 	- `rosservice call /MaxPos/VelocityRamp [rpm] [max_acc]` to activate the velocity control,[rpm] is the velocity in round per minutes (takes into account the gear ratio), [max_acc] is a maximum accelleration that start from 1 and get to a bounded value, to check the meaning, with 10 you get a smooth start-up
 	- ` rosservice call /MaxPos/Disable ` to power off the motor.
 	
 	
 	