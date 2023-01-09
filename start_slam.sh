source ./../cartographer_ws/devel_isolated/setup.bash
source ./devel/setup.bash --extend
# cd ~/catkin_ws/musashi_minibot_ws/src/cartographer_config/launch
roslaunch cartographer_config lumi_carto_slam_only_lidar.launch
