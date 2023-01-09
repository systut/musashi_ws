# Exit immediately if a command exits with a non-zero status
set -e

# clone rplidar_ros
repository_rplidar="https://github.com/Slamtec/rplidar_ros.git"

localFolder_rplidar="../src/rplidar_ros"

git clone "$repository_rplidar" "$localFolder_rplidar"
