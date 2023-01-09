# Utilities

This package contains several namespaces with robot constants, math constants and general functions.
The namespaces are:
- RobotConstants
- MathConstants
- GeneralFunctions
- EncoderConstants
- Testing
- ControlConstants
- RobotLimits

In *Testing* several settings can be done. *USE_MPC*, *USE_OPEN_LOOP* and *USE_VFH_PP* are booleans to turn off / on the resepctive control approach. One of these three booleans should be on while the others should be off.  
*TEST_AVOIDANCE* will turn off the emergency stop to check the performance of the VFH_PP algorithm and *TRACKING_ONLY* ignores the lidar data such that only the tracking performance of VFH_PP can be analyzed.