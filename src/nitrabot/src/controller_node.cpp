#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>

#include <eigen3/Eigen/Core>

#include <boost/make_shared.hpp>

// #include "../include/mpc_ackerman.h"
#include "../include/pure_pursuit.h"

#include <cmath>
#include <mutex>

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        init(nh, private_nh);
        std::cout << "Init succesfull" << std::endl;
        initController();
        std::cout << "Controller init succesfull" << std::endl;
        ros::Duration(1).sleep();
    }

    void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        std::cout << "Call time odom:\t" << now << "sec" << std::endl;
        // std::cout << odom_msg->pose.pose.position.x << std::endl;
        // std::cout << "Call Odom" << std::endl;
        // store robot pose
        odom_msg_ = odom_msg;
    }

    void emgFlagCallback(const std_msgs::Bool::ConstPtr& flag_msg)
    {
        std::lock_guard<std::mutex> lock_flag(flag_mutex_);
        ROS_INFO("I heard: [%s]", flag_msg->data ? "true" : "false");
        emergency_stop_flag_ = flag_msg->data;
    }

    void callController(const ros::TimerEvent &event)
    {
        double now = ros::Time::now().toSec();
        //std::cout << "Call time:\t" << last_ - now << "sec" << std::endl;
        last_ = now;
        //std::cout << "Call Controller" << std::endl;


        // check if ok to execute
        //if (collision_msg_ == nullptr || encoder_msg_ == nullptr || odom_msg_ == nullptr)
        if (odom_msg_ == nullptr)
        {
            ROS_WARN("CONTROLLER_NODE_WARN: No messages received from odom or encoder.");
            return;
        }

        // take data
        // pose
        double yaw = tf::getYaw(odom_msg_->pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        robot_pose_ << odom_msg_->pose.position.x, odom_msg_->pose.position.y, yaw;
        //std::cout << "yaw:" << yaw << std::endl;
        odom_mutex_.unlock();

        // EMG STOP
        if (emergency_stop_flag_)
        {
            std::cout << "Stop Robot" << std::endl;
            // send stop signal to controller

            pure_pursuit_controller_.stopMotion(robot_pose_);
 
            std::cout << "Robot stopped" << std::endl;
            ROS_WARN("EMG STOP ACTIVATED");
        }
        else
        {
            std::cout << "Follow Trajectory" << std::endl;
            pure_pursuit_controller_.control(robot_pose_);
        }
        
        flag_mutex_.unlock();
        
        std::cout << "Function call time:\t" << ros::Time::now().toSec() - now << "sec" << std::endl;
    }

private:
    ros::NodeHandle nh_, private_nh_;
    // subscriber to robot localization and encoder data
    ros::Subscriber odom_sub_;
    ros::Subscriber flag_sub_;

    // vector of robot pose (x, y, theta)
    // messages
    geometry_msgs::PoseStamped::ConstPtr odom_msg_;
    nav_msgs::PathPtr vfh_path_;
    sdv_msgs::Trajectory traj_msg_;

    bool simulation_;
    bool emergency_stop_flag_;

    // steer is in rad and wheel_vel is in m/s
    Eigen::Vector3d robot_pose_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex odom_mutex_, encoder_mutex_, flag_mutex_, collision_mutex_;
    // object of the controller
    // MpcAckerman mpc_controller_;
    PurePursuit pure_pursuit_controller_;

    int update_counter_;

    // timer
    double last_;

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;
        emergency_stop_flag_ = false;
        // check parameter server
        nh.param("controller_sampling_time", sampling_time_, 0.05); //[s]

        boost::shared_ptr<sdv_msgs::Trajectory const> traj_msg;

        traj_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/offline_trajectory_node/trajectory", nh_);
        if(traj_msg != NULL){
            traj_msg_ = *traj_msg;
        }

        // subscriber
        ROS_WARN("CONTROLLER_NODE_WARN: Subscribe 'odom' and 'encoder_odom' conditions");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 4, &ControllerNode::odometryCallback, this);
        odom_sub_ = nh_.subscribe("/tracked_pose", 4, &ControllerNode::odometryCallback, this);
        flag_sub_ = nh_.subscribe("/front_obstacle_detection_node/emg_stop_flag", 1, &ControllerNode::emgFlagCallback, this);
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);
    }

    void initController()
    {
        // mpc_controller_ = MpcAckerman(nh_, private_nh_, sampling_time_);
        pure_pursuit_controller_ = PurePursuit(nh_, private_nh_, sampling_time_, traj_msg_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ControllerNode controller(nh, private_nh);

    ros::AsyncSpinner s(4);
    s.start();
    ros::waitForShutdown();
    
    return 0;
}
