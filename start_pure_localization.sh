source ./cartographer_ws/devel_isolated/setup.bash
source ./devel/setup.bash --extend
# cd ~/nitra_ws/src/cartographer_config/launch 
roslaunch cartographer_config lumi_carto_pure_localization.launch
