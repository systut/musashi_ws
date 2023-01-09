#include "../include/pure_pursuit.h"

// void PurePursuit::read_reference()
// {
//     std::ifstream ref_file("global_trajectory.csv");

//     //Debug for error opening file
//     if (!ref_file) {
//         std::cout << "Error opening file"<<std::endl;
//     }

//     // Count lines and initialize reference matrix accordingly
//     long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
//                                         std::istreambuf_iterator<char>(), '\n');
                                        
//     // first line contains column headers
//     --num_ref_points;

//     ref_ = Eigen::MatrixXd(8, num_ref_points);

//     // Clear bad state from counting lines and reset iterator
//     ref_file.clear();
//     ref_file.seekg(0);

//     std::string line;
//     std::getline(ref_file,line);    // read first line (column headers)

//     long col_idx = 0;
//     while(std::getline(ref_file, line))
//     {
//         std::stringstream lineStream(line);
//         std::string cell;
//         long row_idx = 0;
//         while(std::getline(lineStream, cell, ','))
//         {
//             ref_(row_idx,col_idx) = std::stof(cell);
//             ++row_idx;
//         }
//         ++col_idx;
//     }

//     ref_file.close();

    
//     // pose_ref_ = ref_.block(1, 1, 3, num_ref_points-1);

//     vfh_goal = ref_.block(1, num_ref_points-1, 3, 1);
//     // vfh_heading_path = ;
//     // v_r, v_l
//     // w,v
// }

void PurePursuit::read_reference()
{
    ref_ = Eigen::MatrixXd(4, traj_.points.size());
    for(int ii=0; ii<traj_.points.size(); ii++)
    {
        ref_(1,ii)= traj_.points.at(ii).x;
        ref_(2,ii)= traj_.points.at(ii).y;
        ref_(3,ii)= traj_.points.at(ii).heading;
    }

    // pose_ref_ = ref_.block(1, 1, 3, num_ref_points-1);

    vfh_goal = ref_.block(1, ref_.cols()-1, 3, 1);
    // vfh_heading_path = ;
    // v_r, v_l
    // w,v
}

double PurePursuit::getLinearVelocity(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal)
{
    // get distance vector
    Eigen::Vector2d distance = (vfh_goal.head<2>() - rear_axis_pose.head<2>()).normalized();
    // normal vector depends on turn direction
    double angle_change = GeneralFunctions::wrapToPi(GeneralFunctions::wrapTo2Pi(vfh_goal.z()) - GeneralFunctions::wrapTo2Pi(rear_axis_pose.z()));
    double theta_shift = vfh_goal.z() + M_PI_2 * GeneralFunctions::sgn<double>(angle_change);
    Eigen::Vector2d normal_vector(std::cos(theta_shift), std::sin(theta_shift));

    // get epsilon
    double epsilon = distance.dot(normal_vector);
    // std::cout << "Epsilon: " << std::abs(epsilon) << std::endl;
    // calculate velocity
    double lin_vel = TrajectoryParameters::PATH_VEL_LIM + std::abs(epsilon) * (TrajectoryParameters::PATH_VEL_MIN - TrajectoryParameters::PATH_VEL_LIM);
    // std::cout << "Velocity: " << lin_vel << std::endl;
    return lin_vel;
}

void PurePursuit::control(const Eigen::Vector3d &rear_axis_pose)
{
    // calculate error
    Eigen::Vector3d error = vfh_goal - rear_axis_pose;
    // std::cout << "Error:\n" << error << std::endl;
    // check if at goal
    // if (atGoal(error))
    // {
    //     std::cout << "At goal" << std::endl;
    //     // stopMotion();
    //     return;
    // }

    // bool pub_path = false;
    // if (vfh_path_pub_.getNumSubscribers() > 0)
    // {
    //     vfh_pp_path_msg_.poses.clear();
    //     vfh_pp_path_msg_.poses.resize(10 * ControlConstants::HORIZON);
    //     pub_path = true;
    // }
    std::cout << "goal:\n" << vfh_goal << std::endl;
    Eigen::Vector2d robot_position = rear_axis_pose.head<2>();
    Eigen::Vector2d vfh_position;
    double distance, alpha, ang_vel;

    double lin_vel_pp;

    // double lin_vel = lin_vel_pp;
    double lin_vel;

    // double extend;

    // get current action command from VFH heading path
    for (long ii = counter_; ii < ref_.cols(); ++ii)
    //for (long ii = 1; ii < ref_.cols(); ++ii)
    {
        vfh_position = ref_.block(1, ii, 2, 1);
        distance = (vfh_position - robot_position).norm();
        alpha = getAlpha(rear_axis_pose, vfh_position);
        lin_vel_pp = getLinearVelocity(rear_axis_pose, ref_.block(1, ii, 3, 1));
        

        // double lin_vel = lin_vel_pp;
        lin_vel = std::min(lin_vel_pp, TrajectoryParameters::PATH_VEL_LIM);

        //std::cout << vfh_position << std::endl;

        // calculate velcoitiesd for point which satisfy pp lookahead  // prevent getting a point >90deg which makes robot turn backwards
        if (distance >= ControlConstants::PURE_PURSUIT_LOOKAHEAD && std::abs(alpha) < 1.57)
        {   
            // std::cout << "distance:\n" << distance << std::endl;
            // std::cout << "angvel:\n" << ang_vel << std::endl;
            std::cout << "goal_current:\n" << ref_.block(1, ii, 3, 1) << std::endl;
            counter_ = ii;
            break;
        }

        else if(LastPoint(error))
        {
            std::cout << "goal_current:\n" << ref_.block(1, ii, 3, 1) << std::endl;
            ROS_WARN("----LAST PATH POINT----");
            counter_ = ref_.cols()-1;
            // if(distance < ControlConstants::PURE_PURSUIT_LOOKAHEAD)
            // {
            //     extend = ControlConstants::PURE_PURSUIT_LOOKAHEAD - distance;
            //     if(ref_(1, ref_.cols()-1) - ref_(1, ref_.cols()-1-40) > 0)
            //     {
            //         vfh_position(1) = vfh_position(1) + extend;
            //         std::cout << "goal1" << vfh_position << std::endl;
            //     }

            //     if(ref_(1, ref_.cols()-1) - ref_(1, ref_.cols()-1-40) < 0)
            //     {
            //         vfh_position(1) = vfh_position(1) - extend;
            //         std::cout << "goal2" << vfh_position << std::endl;
            //     }

            //     if(ref_(2, ref_.cols()-1) - ref_(2, ref_.cols()-1-40) > 0)
            //     {
            //         vfh_position(2) = vfh_position(2) + extend;
            //         std::cout << "goal3" << vfh_position << std::endl;
            //     }

            //     if(ref_(2, ref_.cols()-1) - ref_(2, ref_.cols()-1-40) < 0)
            //     {
            //         vfh_position(2) = vfh_position(2) - extend;
            //         std::cout << "goal4" << vfh_position << std::endl;
            //     }
            //     alpha = getAlpha(rear_axis_pose, vfh_position);
            //     distance = (vfh_position - robot_position).norm();
            // }
            
            break;
        }
        // std::cout << "counter:\n" << counter_ << std::endl;
        // std::cout << "ii:\n" << ii << std::endl;
    }

    // distance from goal(for reducing vel) temp
    double distance2 = (vfh_goal.head<2>() - robot_position).norm();

    if(distance2 <= POS_THRESHOLD2)
    {
        std::cout << "Near goal" << std::endl;
        lin_vel = TrajectoryParameters::PATH_VEL_LIM/2;
    }

    if(distance2 <= POS_THRESHOLD3)
    {
        std::cout << "Near goal2" << std::endl;
        lin_vel = 0.01;
    }

    
    // std::cout << "distance:\n" << distance << std::endl;
    // std::cout << "angvel:\n" << ang_vel << std::endl;

    // calculate constraints for linear velocity
    lin_vel = linearVelConstraint(lin_vel, alpha);
    lin_vel = linearAccConstraint(lin_vel, alpha);

    // calculate angular velocity
    ang_vel = 2 * lin_vel * std::sin(alpha) / distance;

    prev_lin_vel_ = lin_vel;

    if(atGoal(error))
    {
        std::cout << "At goal" << std::endl;
        lin_vel = 0.0;
        ang_vel = 0.0;
        flag = 1;
    }
    if(flag == 1)
    {
        std::cout << "At goal" << std::endl;
        lin_vel = 0.0;
        ang_vel = 0.0;
    }

    // calculate actions for SDV
    Eigen::Vector2d input(lin_vel, ang_vel);

    rpm_convert_publish(input);
    // getVfhControlCmd(input);

    generateCSV(rear_axis_pose, input);
}

void PurePursuit::stopMotion(const Eigen::Vector3d &rear_axis_pose)
{
    Eigen::Vector2d input(0.0, 0.0);

    prev_lin_vel_ = 0.0;
    // getVfhControlCmd(input);
    rpm_convert_publish(input);

    generateCSV(rear_axis_pose, input);
}

bool PurePursuit::atGoal(const Eigen::Vector3d &error) const
{   
    if(std::abs(error(2))>6.0)
    {
        if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && (std::abs(error(2)) - MathConstants::TWOPI) < HEADING_THRESHOLD)
        {
            return true;
        }
    }
    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && std::abs(error(2)) < HEADING_THRESHOLD)
    {
        return true;
    }
    return false;
}

bool PurePursuit::LastPoint(const Eigen::Vector3d &error) const
{   
    if(std::abs(error(2))>6.0)
    {
        if (std::abs(error(0)) < ControlConstants::PURE_PURSUIT_LOOKAHEAD && std::abs(error(1)) < ControlConstants::PURE_PURSUIT_LOOKAHEAD && (std::abs(error(2)) - MathConstants::TWOPI) < HEADING_THRESHOLD)
        {
            return true;
        }
    }
    if (std::abs(error(0)) < ControlConstants::PURE_PURSUIT_LOOKAHEAD && std::abs(error(1)) < ControlConstants::PURE_PURSUIT_LOOKAHEAD && std::abs(error(2)) < HEADING_THRESHOLD)
    {
        return true;
    }
    return false;
}

inline void PurePursuit::getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state)
{
    if (GeneralFunctions::isEqual(input.y(), 0.0))
    {
        state << prev_state.x() + input.x() * sampling_time_ * std::cos(prev_state.z()),
                 prev_state.y() + input.y() * sampling_time_ * std::sin(prev_state.z()),
                 prev_state.z() + 0.0;
    }
    else
    {
        double radius = input.x() / input.y();
        state << prev_state.x() - radius * std::sin(prev_state.z()) + radius * std::sin(prev_state.z() + input.y() * sampling_time_),
                 prev_state.y() + radius * std::cos(prev_state.z()) - radius * std::cos(prev_state.z() + input.y() * sampling_time_),
                 GeneralFunctions::wrapTo2Pi(prev_state.z() + input.y() * sampling_time_);
    }
}

inline double PurePursuit::getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position)
{
    Eigen::Vector2d vec1(std::cos(robot_pose.z()), std::sin(robot_pose.z()));
    Eigen::Vector2d vec2 = vfh_position - robot_pose.head<2>();

    // "https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/issues/index.htm"
    double angle = std::atan2(vec2(1), vec2(0)) - std::atan2(vec1(1), vec1(0));
    if (angle > M_PI){angle -= 2 * M_PI;}
    else if (angle < -M_PI) {angle += 2 * M_PI;}
    //std::cout << "Angle:\t" << angle << std::endl;
    return angle;
}

void PurePursuit::getVfhControlCmd(Eigen::Vector2d &input)
{
    double angular_vel_ref = input(1);
    double linear_vel_ref = input(0);

    if (std::abs(angular_vel_ref) < 0.001)
    {
        double angular_vel_ref = 0.0;

    }

    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    vel_msgs_.linear.x = linear_vel_ref;
    vel_msgs_.angular.z = angular_vel_ref;
    velocity_publisher_.publish(vel_msgs_);
}

// wheel velocities are in rad/s
void PurePursuit::rpm_convert_publish(Eigen::Vector2d &input)
{
    static double wheel_velocities [2];
    Eigen::Vector2d v_rpm;

    velocity_msgs_.data.resize(2);

    double angular_robot_vel = input(1);
    double linear_robot_vel = input(0);

    if (std::abs(angular_robot_vel) < 0.001)
    {
        double angular_robot_vel = 0.0;

    }
    
    double left_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) - (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
    wheel_velocities[0] = left_wheel_vel;

    double right_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) + (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
    wheel_velocities[1] = right_wheel_vel;
    // left
    // v_rpm[0] = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10) / 10;
    // value in *10
    v_rpm(0) = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10);
    // right
    v_rpm(1) = round((wheel_velocities[1] * 60/MathConstants::TWOPI) * 10);
    
    velocity_publisher_ = nh_.advertise<std_msgs::Int16MultiArray>("cmd_vel", 1000);

    //vl
    velocity_msgs_.data[0] = v_rpm(0);
    //vr
    velocity_msgs_.data[1] = v_rpm(1);

    velocity_publisher_.publish(velocity_msgs_);
}

void PurePursuit::generateCSV(const Eigen::Vector3d &pose, const Eigen::Vector2d &input)
{
    std::ofstream export_data;

   // Eigen::Vector3d error = pose - ref_.block(1, counter_, 3, 1);
   // Eigen::Vector3d state_ref = ref_.block(1, counter_, 3, 1);

    export_data.open(filename_, std::ios::out|std::ios::app);
    for (long ii = 0; ii < pose.rows(); ++ii)
    {
        export_data << pose(ii) << ", ";
    }
    for (long ii = 0; ii < input.rows(); ++ii)
    {
        export_data << input(ii);
        if (ii != (input.rows()-1))
        {
            export_data << ", ";
        }
    }
  //  for (long ii = 0; ii < error.rows(); ++ii)
  //  {
  //      export_data << error(ii) << ", ";
  //  }
  //  for (long ii = 0; ii < state_ref.rows(); ++ii)
  //  {
  //      export_data << state_ref(ii);
  //      if (ii != (state_ref.rows()-1))
  //      {
  //          export_data << ", ";
  //      }
  //  }
    export_data << std::endl;
}