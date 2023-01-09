#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int16MultiArray.h>


class TestFeedForwardControl 
{
    private:
        
        ros::Publisher velocity_publisher_;
        
        ros::NodeHandle nh_;
        
        // Reference
        Eigen::MatrixXd ref_;
        
        std_msgs::Int16MultiArray velocity_msgs_;
        
        ros::Timer controller_timer_; 

        unsigned long counter_;

        void readReference()
        {
            std::ifstream ref_file("trajectory_with_rpm.csv");
            if (ref_file.good())
            {
                // Count lines and initialize reference matrix accordingly
                long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
                                                 std::istreambuf_iterator<char>(), '\n');
                // first line contains column headers
                --num_ref_points;
                ref_ = Eigen::MatrixXd(8, num_ref_points);

                // Clear bad state from counting lines and reset iterator
                ref_file.clear();
                ref_file.seekg(0);

                std::string line;
                std::getline(ref_file,line);    // read first line (column headers)

                long col_idx = 0;
                while(std::getline(ref_file, line))
                {
                    std::stringstream lineStream(line);
                    std::string cell;
                    long row_idx = 0;
                    while(std::getline(lineStream, cell, ','))
                    {
                        ref_(row_idx,col_idx) = std::stof(cell);
                        ++row_idx;
                    }
                    ++col_idx;
                }
            }
            ref_file.close();
        }

        void execute(const ros::TimerEvent& event)
        {
            velocity_msgs_.data.resize(2);

            std::cout << "Step " << counter_ << " :" << std::endl;

            if (counter_ == ref_.cols()-1)
            {
                velocity_msgs_.data[0] = 0.0;
                
                velocity_msgs_.data[1] = 0.0;
                
                velocity_publisher_.publish(velocity_msgs_);

                ros::shutdown();
            }

            Eigen::Vector2d input_ref = ref_.block(6, counter_, 2, 1);
            
            std::cout << input_ref << std::endl;
            //vl
            velocity_msgs_.data[0] = input_ref(0);
            //vr
            velocity_msgs_.data[1] = input_ref(1);

            ++counter_;

            velocity_publisher_.publish(velocity_msgs_);
        }
    
    public:
        TestFeedForwardControl(ros::NodeHandle nh)
        {
            nh_ = nh;
            
            readReference();

            counter_ = 0;

            controller_timer_ = nh_.createTimer(ros::Duration(0.05), &TestFeedForwardControl::execute, this);

            velocity_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("cmd_vel", 1000);

        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_feed_forward_control");

    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(0);

    spinner.start();
    
    TestFeedForwardControl controller(nh);
    
    ros::waitForShutdown();
}

