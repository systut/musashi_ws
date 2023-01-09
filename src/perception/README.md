# Perception

This package contains algorithms for perception. The algorithm takes the pointcloud data ahead of the robot and calculates the surface normals using the pcl library. The surface normals are used to calculate the tangent in the direction of motion of the robot (currently straight motion is assummed) and with the tangent vector the inclination angle of the pointcloud point can be calculated. If the point violates the allowed slope angle or the allowed height it will be considered an obstacle and will be inflated.

## How to run
*rosrun perception slope_bump_detection_node*