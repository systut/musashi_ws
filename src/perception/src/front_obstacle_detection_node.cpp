#include "../include/perception/front_obstacle_detection.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "front_obstacle_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    FrontObstacleDetection detection(nh, private_nh);

    ros::Subscriber sub_odom = nh.subscribe("/odometry/filtered", 1, &FrontObstacleDetection::odometryCallback, &detection);
    ros::Subscriber sub_lidar = nh.subscribe("/cloud", 1, &FrontObstacleDetection::frontPointCloudCallback, &detection);

    // velodyne sim has 10hz
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        detection.publishFrontObstacleMap();

        loop_rate.sleep();
    }

    return 0;
}
