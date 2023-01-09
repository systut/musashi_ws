#include "../include/perception/front_obstacle_detection.h"

void FrontObstacleDetection::frontPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &front_lidar_msg)
{
    //std::cout << "frame:\t" << front_velo_msg->header.frame_id << std::endl;

    // convert cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*front_lidar_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //std::cout << "Coordinates of first point:\t[" << temp_cloud->points[0]._PointXYZ::x << ", "
    //                                              << temp_cloud->points[0]._PointXYZ::y << ", "
    //                                              << temp_cloud->points[0]._PointXYZ::z << "]" << std::endl;

    // crop cloud
    boxFilter_.setInputCloud(temp_cloud);
    boxFilter_.filter(*cropped_cloud_);
}

void FrontObstacleDetection::odometryCallback(const nav_msgs::OdometryConstPtr odom_msg)
{
    nav_msgs::Odometry odom = *odom_msg;
    double yaw = tf::getYaw(odom.pose.pose.orientation);
    yaw = GeneralFunctions::wrapTo2Pi(yaw);
    pose_ << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, yaw;
}

void FrontObstacleDetection::publishFrontObstacleMap()
{
    // updatet map pose
    updateMapInfo();

    pcl::toROSMsg(*cropped_cloud_, test_msg_);
    pub_cloud_test_.publish(test_msg_);

    if (cropped_cloud_->size() == 0)
    {
        ROS_INFO("Cropped pointcloud is empty.");
        pub_front_obstacle_map_.publish(front_obstacle_map_msg_);
        emergency_stop_flag_.data = false;
        pub_emg_stop_flag_.publish(emergency_stop_flag_);
        return;
    }

    int cell_x, cell_y;
    unsigned int index;
    double x, y, z;
    /*
    for (unsigned int kk = 0; kk < cropped_cloud_->size(); ++kk)
    {
        Eigen::Vector3d coordinates;
        transformCloudPoints(cropped_cloud_->points[kk], coordinates);
        // get cell pos x and y
        cell_x = static_cast<int>(coordinates.x() / front_obstacle_map_msg_.info.resolution);
        cell_y = static_cast<int>(coordinates.y() / front_obstacle_map_msg_.info.resolution);
        // map index
        index = static_cast<unsigned int>(cell_x) + front_obstacle_map_msg_.info.width * static_cast<unsigned int>(cell_y);
        if (inGrid(cell_x, cell_y))
        {
            front_obstacle_map_msg_.data[index] = 50;
        }
    }
    */
    // create normal estimation class
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cropped_cloud_);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    //ne.setRadiusSearch (0.35);
    ne.setRadiusSearch (0.02);
    // Compute the features
    ne.compute(*cloud_normals);
    if (cloud_normals->size() == cropped_cloud_->size())
    {
        //ROS_INFO("Size of normal cloud equals to input cloud:\t%d.", cropped_cloud_->size());
    }
    else
    {
        ROS_INFO("Size of normal cloud does not match size of input cloud.");
    }
    // get the perpendicular vector of the robot heading
    // FIXME is this correct?
    //Eigen::Vector3d pose_perp(-pose_(1), pose_(0), 0.0);
    // Eigen::Vector3d pose_perp = Eigen::Vector3d::UnitY();
    // normal vector for each cloud point
    // Eigen::Vector3d normal_vec;
    // normal vector of XY-plane
    // Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    // coordinates in the right frame
    Eigen::Vector3d coordinates;
    // iterate through cropped cloud
    for (unsigned int ii = 0; ii < cloud_normals->size(); ++ii)
    {
        // get cell coorindates
        transformCloudPoints(cropped_cloud_->points[ii], coordinates);
        // get cell pos x and y
        cell_x = static_cast<int>(coordinates.x() / front_obstacle_map_msg_.info.resolution);
        cell_y = static_cast<int>(coordinates.y() / front_obstacle_map_msg_.info.resolution);
        // map index
        index = static_cast<unsigned int>(cell_x) + front_obstacle_map_msg_.info.width * static_cast<unsigned int>(cell_y);
        if (!inGrid(cell_x, cell_y) || front_obstacle_map_msg_.data[index] == 100)
        {
            // no need to make calculations with this point
            continue;
        }
        // calculate normal direction (currently normal of robot heading later possible robot trajectories)
        // TODO
        // normal_vec << cloud_normals->points[ii].normal_x, cloud_normals->points[ii].normal_y ,cloud_normals->points[ii].normal_z;
        // get the cross product and therefore the surface tangent
        // Eigen::Vector3d tangent = normal_vec.cross(pose_perp);
        // TODO check if correct tangent

        // calculate the inclination angle
        // double abs_dot_prod = abs(tangent.dot(plane_normal));
        // angle = asin(|dot(plane_normal, tangent)| / plane_normal_normalized * tangent_normalized)
        // plane_normal = [0 0 1] plane_normal_normalized = 1
        // double angle = asin(abs_dot_prod / (tangent.norm())) * 180 / MathConstants::PI;
        //std::cout << "The slope angle = " << angle << std::endl;
        // check if it is bump or slope or slope which violates constraints
        // slope

        // if (angle <= 6.0 && angle >= 0.1)
        // {
        //     //std::cout << "The slope angle = " << angle << " deg" << std::endl;
        //     // traversable point
        //     // -> do nothing
        //     continue;
        // }
        // else
        // {
        //     //std::cout << "The height is = " << coordinates.z() << "m" << std::endl;
        //     //std::cout << "The slope angle = " << angle << " deg" << std::endl;
        //     // traversable if height is okay
        //     if (coordinates.z() <= 0.03)
        //     {
        //         // traversable
        //         // -> do nothing
        //         continue;
        //     }
        //     else
        //     {

        // violation
        front_obstacle_map_msg_.data[index] = 100;
        x = coordinates.x();
        y = coordinates.y();
        // inflate obstacle
        double radius = front_obstacle_map_msg_.info.resolution;
        
        while (radius <= ROBOT_RADIUS)
        {
            if (radius <= SQUARE_RADIUS)
            {
            midpointCircleAlgorithm(x, y, radius, INFLATE_SQUARE_COST);
            }
            else if (radius <= RECTANGLE_RADIUS)
            {
            midpointCircleAlgorithm(x, y, radius, INFLATE_RECTANGLE_COST);
            }
            else
            {
            midpointCircleAlgorithm(x, y, radius, INFLATE_ROBOT_COST);
            }
            radius += front_obstacle_map_msg_.info.resolution;
        }
            // }
        // }
    }
        /*
        // bump
        else if (angle >= 45.0 && angle <= 90.0)
        {
            std::cout << "The height is = " << coordinates.z() << "m" << std::endl;
            // traversable if height is okay
            if (coordinates.z() <= 0.03)
            {
                // traversable
                // -> do nothing
                continue;
            }
            else
            {
                // violation
                // get cell pos x and y
                cell_x = static_cast<int>(coordinates.x() / front_obstacle_map_msg_.info.resolution);
                cell_y = static_cast<int>(coordinates.y() / front_obstacle_map_msg_.info.resolution);
                // map index
                index = static_cast<unsigned int>(cell_x) + front_obstacle_map_msg_.info.width * static_cast<unsigned int>(cell_y);
                if (inGrid(cell_x, cell_y))
                {
                    front_obstacle_map_msg_.data[index] = 100;
                }
            }
        }
        // violation
        else
        {
            std::cout << "The slope angle = " << angle << "deg" << std::endl;
            // get cell pos x and y
            cell_x = static_cast<int>(coordinates.x() / front_obstacle_map_msg_.info.resolution);
            cell_y = static_cast<int>(coordinates.y() / front_obstacle_map_msg_.info.resolution);
            // map index
            index = static_cast<unsigned int>(cell_x) + front_obstacle_map_msg_.info.width * static_cast<unsigned int>(cell_y);
            if (inGrid(cell_x, cell_y))
            {
                front_obstacle_map_msg_.data[index] = 100;
            }
        }
    }
    */

    
    pub_front_obstacle_map_.publish(front_obstacle_map_msg_);

    atRange(front_obstacle_map_msg_);

    // reset map
    for (unsigned int kk = 0; kk < front_obstacle_map_msg_.info.width * front_obstacle_map_msg_.info.height; ++kk)
    {
        front_obstacle_map_msg_.data[kk] = 0;
    }

}

void FrontObstacleDetection::updateMapInfo()
{   
    try
    {
        listener_->lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base_);
        listener_->lookupTransform("/base_link", "/laser", ros::Time(0), transform_base_lidar_);
        //std::cout << "Transform succesful" << std::endl;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    // run the following commented code to rotate the map with the robot orientation
    // update the position and orientation of the map based on the current robots position and ori
    
    tf::Transform transform_odom_lidar = transform_odom_base_ * transform_base_lidar_;

    double roll, pitch, yaw;
    //transform_odom_lidar.getBasis().getRPY(roll, pitch, yaw);
    transform_odom_base_.getBasis().getRPY(roll, pitch, yaw);

    //ROS_INFO("%f", yaw);
    map_metadata_msg_.origin.position.x = transform_odom_lidar.getOrigin().getX()
        - sin(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.height/2 - transform_base_lidar_.getOrigin().getY());
    map_metadata_msg_.origin.position.y = transform_odom_lidar.getOrigin().getY()
        + cos(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.height/2 - transform_base_lidar_.getOrigin().getY());
    map_metadata_msg_.origin.position.z = 0; // ground
    //map_metadata_msg_.origin.orientation.w = transform_odom_velo.getRotation().getW();
    //map_metadata_msg_.origin.orientation.x = transform_odom_velo.getRotation().getX();
    //map_metadata_msg_.origin.orientation.y = transform_odom_velo.getRotation().getY();
    //map_metadata_msg_.origin.orientation.z = transform_odom_velo.getRotation().getZ();
    map_metadata_msg_.origin.orientation.w = transform_odom_base_.getRotation().getW();
    map_metadata_msg_.origin.orientation.x = transform_odom_base_.getRotation().getX();
    map_metadata_msg_.origin.orientation.y = transform_odom_base_.getRotation().getY();
    map_metadata_msg_.origin.orientation.z = transform_odom_base_.getRotation().getZ();
    front_obstacle_map_msg_.info = map_metadata_msg_;
    // the following code is for a non rotating map
    /*
    map_metadata_msg_.origin.position.x = transform_odom_base->getOrigin().getX() - double(map_metadata_msg_.resolution) * map_metadata_msg_.width / 2;
    map_metadata_msg_.origin.position.y = transform_odom_base->getOrigin().getY() - double(map_metadata_msg_.resolution) * map_metadata_msg_.height / 2;
    front_obstacle_map_msg_.info = map_metadata_msg_;
    */
}

void FrontObstacleDetection::transformCloudPoints(const pcl::PointXYZ &point, Eigen::Vector3d &coordinates)
{
    tf::Transform orig_tf, point_tf;
    tf::poseMsgToTF(front_obstacle_map_msg_.info.origin, orig_tf);
    geometry_msgs::Pose point_pose;
    point_pose.position.x = point.x;
    point_pose.position.y = point.y;
    point_pose.position.z = point.z;
    tf::poseMsgToTF(point_pose, point_tf);
    point_tf = orig_tf.inverse() * transform_odom_base_ * transform_base_lidar_ * point_tf;

    coordinates << point_tf.getOrigin().getX(), point_tf.getOrigin().getY(), point_tf.getOrigin().getZ();


    /*
    double roll, pitch, yaw;
	transform_odom_base_.getBasis().getRPY(roll, pitch, yaw);
	double x_base = point.x + transform_base_velo_.getOrigin().x();
	double y_base = point.y + transform_base_velo_.getOrigin().y();
	double z_base = point.z + transform_base_velo_.getOrigin().z();

	// rotate coordinate
    coordinates << cos(yaw) * x_base - sin(yaw) * y_base,
                   sin(yaw) * x_base + cos(yaw) * y_base,
                   z_base;
    */
}

inline bool FrontObstacleDetection::inGrid(int cell_x, int cell_y)
{
    if ((cell_x < 0 || cell_x >= static_cast<signed int>(front_obstacle_map_msg_.info.width))
	        || (cell_y < 0 || cell_y >= static_cast<signed int>(front_obstacle_map_msg_.info.height)))
	{
	    return false;
	}
	else {return true;}
}

void FrontObstacleDetection::updateAllOctants(double x, double y, double *obstacle_x, double *obstacle_y, signed char inflated_value)
{
	double x_obst, y_obst;

	// 1 octant (x, y)
	x_obst = x + *obstacle_x;
	y_obst = y + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 2 octant (y, x)
	x_obst = y + *obstacle_x;
	y_obst = x + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 3 octant (-y, x)
	x_obst = -y + *obstacle_x;
	y_obst = x + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 4 octant (-x, y)
	x_obst = -x + *obstacle_x;
	y_obst = y + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 5 octant (-x, -y)
	x_obst = -x + *obstacle_x;
	y_obst = -y + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 6 octant (-y, -x)
	x_obst = -y + *obstacle_x;
	y_obst = -x + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 7 octant (y, -x)
	x_obst = y + *obstacle_x;
	y_obst = -x + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);

	// 8 octant (x, -y)
	x_obst = x + *obstacle_x;
	y_obst = -y + *obstacle_y;
	updateInflatedValues(x_obst, y_obst, inflated_value);
}

void FrontObstacleDetection::midpointCircleAlgorithm(double obstacle_x, double obstacle_y, double radius, signed char inflate_cost)
{
	// start position for midpoint(0, 0)
	double x = radius;
	double y = 0.0f;

	updateAllOctants(x, y, &obstacle_x, &obstacle_y, inflate_cost);

	//iteration in the first octant
	while (x > y)
	{
	    // currently px = local_map_msg_.info.resolution
	    // x+1^2 = x^2 - 2y - 1 (1px) = x^2 - 2y px - px^2
	    x = sqrtf(x * x - 2 * y * front_obstacle_map_msg_.info.resolution - (front_obstacle_map_msg_.info.resolution * front_obstacle_map_msg_.info.resolution));
	    // y+1^2 = (y + 1)^2 (1px) -> y+1 = y + 1px
	    y = y + front_obstacle_map_msg_.info.resolution;

	    updateAllOctants(x, y, &obstacle_x, &obstacle_y, inflate_cost);
	}
}

void FrontObstacleDetection::updateInflatedValues(double x, double y, signed char inflate_value)
{
	// get cell pos x and y
	int cell_x = static_cast<int>(x / front_obstacle_map_msg_.info.resolution);
    int cell_y = static_cast<int>(y / front_obstacle_map_msg_.info.resolution);

	// map index
	unsigned int index = static_cast<unsigned int>(cell_x) + front_obstacle_map_msg_.info.width * static_cast<unsigned int>(cell_y);
	if (inGrid(cell_x, cell_y))
	{
	    // check if cell is already assigned with a higher value
	    if (front_obstacle_map_msg_.data[index] < inflate_value)
	        front_obstacle_map_msg_.data[index] = inflate_value;
	}
}

void FrontObstacleDetection::atRange(nav_msgs::OccupancyGrid costmap_)
{
    // calculate cell coordinates for emergency stop
    double x_pos = 1.0; //[m]
    double y_pos = 0.0; //[m]

    double distance;

    for (unsigned int cell_x = 0; cell_x < costmap_.info.width; ++cell_x)
    {
        for (unsigned int cell_y = 0; cell_y < costmap_.info.height; ++cell_y)
        {
            unsigned int index = cell_x + cell_y * costmap_.info.width;
            if (costmap_.data[index] == 100)
            {
                distance = cell_x * costmap_.info.resolution;
                if (distance < 1.1)
                {
                    std::cout << "Emergency stop active" << std::endl;
                    emergency_stop_flag_.data = true;
                    pub_emg_stop_flag_.publish(emergency_stop_flag_);
                    return;
                }
                else
                {
                    std::cout << "Safe. Continue Tracking." << std::endl;
                    emergency_stop_flag_.data = false;
                    pub_emg_stop_flag_.publish(emergency_stop_flag_);
                    return;
                }
            }
        }
    }
    // Safe return false
    emergency_stop_flag_.data = false;
    pub_emg_stop_flag_.publish(emergency_stop_flag_);
    return;
}