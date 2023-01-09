#ifndef front_obstacle_detection_H
#define front_obstacle_detection_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <std_msgs/Bool.h>

#include <eigen3/Eigen/Core>

#include <mutex>

#include "utilities/utilities.h"

class FrontObstacleDetection
{
public:
    FrontObstacleDetection(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;

        listener_ = new tf::TransformListener();
        // get first transform
        try
        {
            ros::Duration(1.0).sleep();
            listener_->lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base_);
            listener_->lookupTransform("/base_link", "/laser", ros::Time(0), transform_base_lidar_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        pub_front_obstacle_map_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("slope_map", 1);
        pub_cloud_test_ = private_nh_.advertise<sensor_msgs::PointCloud2>("test_crop", 1);
        pub_emg_stop_flag_ = private_nh_.advertise<std_msgs::Bool>("emg_stop_flag", 1);

        nh_.param("map_width", map_width_, 1.0);
        nh_.param("map_height", map_height_, RobotConstants::ROBOT_WIDTH + 0.1);
        nh_.param("cell_size", cell_size_, 0.05);

        map_metadata_msg_.resolution = cell_size_;
        map_metadata_msg_.width = static_cast<unsigned int>(map_width_ / cell_size_);
        map_metadata_msg_.height = static_cast<unsigned int>(map_height_ / cell_size_);

        std::cout << "Cell size:\t" << map_metadata_msg_.resolution
                  << "\tWidth:\t" << map_metadata_msg_.width
                  << "\tHeight:\t" << map_metadata_msg_.height << std::endl;

        // first get it in meter (cellsize times width) then it starts by minus half of that [bottom left corner]
	    map_metadata_msg_.origin.position.x = 0 + transform_base_lidar_.getOrigin().getX();
	    map_metadata_msg_.origin.position.y = 0 - map_height_ / 2;
        map_metadata_msg_.origin.position.z = 0; // ground/ lidar height?
        map_metadata_msg_.origin.orientation.w = 1;

        front_obstacle_map_msg_.info = map_metadata_msg_;
        front_obstacle_map_msg_.data.resize(map_metadata_msg_.width * map_metadata_msg_.height);
        front_obstacle_map_msg_.header.frame_id = "odom";

        for (unsigned long kk = 0; kk < front_obstacle_map_msg_.data.size(); ++kk)
        {
            front_obstacle_map_msg_.data[kk] = 0;
        }

        emergency_stop_flag_.data = false;
/*
        min_x_ = 0.0;
        max_x_ = map_width_;
        min_y_ = 0.0;//-map_height_ / 2 - transform_base_velo_.getOrigin().getY();
        max_y_ = map_height_;//map_height_ / 2 - transform_base_velo_.getOrigin().getY();

        tf::Transform orig_tf, point_tf;
        tf::poseMsgToTF(front_obstacle_map_msg_.info.origin, orig_tf);
        geometry_msgs::Pose point_pose;
        //point_pose.orientation.z = 1;

        point_pose.position.x = max_x_;
        point_pose.position.y = max_y_;
        tf::poseMsgToTF(point_pose, point_tf);
        tf::Transform combined_trans = transform_base_velo_.inverse() * transform_odom_base_.inverse() * orig_tf;
        point_tf = combined_trans * point_tf;
        max_x_ = point_tf.getOrigin().getX();
        max_y_ = point_tf.getOrigin().getY();

        point_pose.position.x = min_x_;
        point_pose.position.y = min_y_;
        tf::poseMsgToTF(point_pose, point_tf);
        point_tf = transform_base_velo_.inverse() * transform_odom_base_.inverse() * orig_tf * point_tf;
        min_x_ = point_tf.getOrigin().getX();
        min_y_ = point_tf.getOrigin().getY();

        std::cout << "MinMax:[minx, maxx, miny, maxy] = [" << min_x_ << ", " << max_x_ << ", " << min_y_ << ", " << max_y_ << "]" << std::endl;

        std::cout << "BaseVelo z and OdomBase z:\t[" << transform_base_velo_.getOrigin().getZ() << ", " <<
                     transform_odom_base_.getOrigin().getZ() << "]" << std::endl;
*/
        // set crop values
        //boxFilter_.setMin(Eigen::Vector4f(min_x_, min_y_, -100.0, 1.0f));
        //boxFilter_.setMax(Eigen::Vector4f(max_x_, max_y_, 100.0f, 1.0f));
        min_x_ = 0.0;
        max_x_ = map_width_;
        min_y_ = -map_height_ / 2;// - transform_base_velo_.getOrigin().getY();
        max_y_ = map_height_ / 2;// - transform_base_velo_.getOrigin().getY();
        double additional_translation = (map_height_ / 2 - transform_base_lidar_.getOrigin().getY() / 2) / 2;
        Eigen::Vector3f translation(0, -transform_base_lidar_.getOrigin().getY() - additional_translation, 0);
        double roll, pitch, yaw;
        transform_base_lidar_.getBasis().getRPY(roll, pitch, yaw);
        Eigen::Vector3f rotation(0, 0, -yaw);

        boxFilter_.setRotation(rotation);
        boxFilter_.setTranslation(translation);
        boxFilter_.setMin(Eigen::Vector4f(min_x_, min_y_, -0.10635f, 1.0f));
        boxFilter_.setMax(Eigen::Vector4f(max_x_, max_y_, 0.6f - 0.10635f, 1.0f));

        // initialize cropped cloud
        cropped_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    /// get pointcloud and crop it
    void frontPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &front_lidar_msg);

    void odometryCallback(const nav_msgs::OdometryConstPtr odom_msg);

    void publishFrontObstacleMap();

    void atRange(nav_msgs::OccupancyGrid costmap_);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformListener *listener_;

    ros::Publisher pub_front_obstacle_map_;
    ros::Publisher pub_cloud_test_;
    ros::Publisher pub_emg_stop_flag_;

    nav_msgs::OccupancyGrid front_obstacle_map_msg_;
    nav_msgs::MapMetaData map_metadata_msg_;
    sensor_msgs::PointCloud2 test_msg_;

    tf::StampedTransform transform_odom_base_;
    tf::StampedTransform transform_base_lidar_;

    std_msgs::Bool emergency_stop_flag_;

    double map_width_;
    double map_height_;
    double cell_size_;

    float min_x_, min_y_, max_x_, max_y_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_;
    pcl::CropBox<pcl::PointXYZ> boxFilter_;

    Eigen::Vector3d pose_;

    const signed char INFLATE_ROBOT_COST = 25;
    const signed char INFLATE_RECTANGLE_COST = 50;
    const signed char INFLATE_SQUARE_COST = 75;

    const double ROBOT_RADIUS = 0.808;
    const double RECTANGLE_RADIUS = 0.509;
    const double SQUARE_RADIUS = 0.340;

    void updateMapInfo();

    void transformCloudPoints(const pcl::PointXYZ &point, Eigen::Vector3d &coordinates);

    inline bool inGrid(int cell_x, int cell_y);

    void updateAllOctants(double x, double y, double *obstacle_x, double *obstacle_y, signed char inflated_value);

    void midpointCircleAlgorithm(double obstacle_x, double obstacle_y, double radius, signed char inflate_cost);

    void updateInflatedValues(double x, double y, signed char inflate_value);


};

#endif //(SLOPE_BUMP_DETECTION_H)
