# Exit immediately if a command exits with a non-zero status
set -e 

# Install cartographer

sudo apt-get update

sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

wait

mkdir ./../cartographer_ws

cd ./../cartographer_ws

wstool init src

wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall

wait

sudo apt-get install libceres-dev

sudo apt-get install lua5.2 liblua5.2-dev libluabind-dev

wait

wstool update -t src

sudo rosdep init

rosdep update

#rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

src/cartographer/scripts/install_abseil.sh

#wait

#sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp

wait

cd ./../cartographer_ws && catkin_make_isolated --install --use-ninja
