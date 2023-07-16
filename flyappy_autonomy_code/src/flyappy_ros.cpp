#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      pub_position_(nh.advertise<geometry_msgs::Vector3>("/flyappy_pos", QUEUE_SIZE)),
      pub_map_(nh.advertise<nav_msgs::OccupancyGrid>("/flyappy_map", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
    // initialize state
    flyappy_.set_acceleration(0, 0);
    flyappy_.set_velocity(0, 0);
    flyappy_.set_initial_position(0, 0);

    // initializing constant map information
    flyappy_.map_.resolution = flyappy_.height_pixel_size;
    flyappy_.map_.first_wall_max_x = -1;
    flyappy_.map_.first_wall_min_x = 1000;

    // path planning initializing
    flyappy_.path_.goal_pos[0] = flyappy_.laser_data_.range_max+1;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = 20;
    flyappy_.pid_.kd[0] = 0.0;
    flyappy_.pid_.kd[1] = 150;

    // initialize state
    flyappy_.states_ = init;
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    flyappy_.set_velocity(msg->x, -msg->y);
    flyappy_.set_position(flyappy_.get_velocity()[0] * flyappy_.periode, flyappy_.get_velocity()[1] * flyappy_.periode);

    // publish position
    geometry_msgs::Vector3 pos;
    pos.x = flyappy_.get_position()[0];
    pos.y = flyappy_.get_position()[1];
    pub_position_.publish(pos);
    
    // updating time steps variable
    ++flyappy_.num_time_steps_;
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    updateLaserData(msg);

    if (flyappy_.states_ == move_forward)
    {
        wallDataUpdate();
        flyappy_.fuzzy_control();
        flyappy_.find_target_y();
        flyappy_.pid_control_y();
    }
    else if (flyappy_.states_ == init)
    {
        flyappy_.find_upper_lower_boundary();
        if (flyappy_.path_.lower_bound_set && flyappy_.path_.upper_bound_set)
            flyappy_.states_ = move_forward;
    }

    // send command    
    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = flyappy_.get_acceleration()[0];
    acc_cmd.y = flyappy_.get_acceleration()[1];
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_ = {};
    // initialize state
    flyappy_.set_acceleration(0, 0);
    flyappy_.set_velocity(0, 0);
    flyappy_.set_initial_position(0, 0);

    // initializing constant map information
    flyappy_.map_.resolution = flyappy_.height_pixel_size;
    flyappy_.map_.first_wall_max_x = -1;
    flyappy_.map_.first_wall_min_x = 1000;

    // path planning initializing
    flyappy_.path_.goal_pos[0] = flyappy_.laser_data_.range_max+1;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = 20; // kp for y
    flyappy_.pid_.kd[0] = 0.0; // kd for x
    flyappy_.pid_.kd[1] = 200; // kd for y

    // states
    flyappy_.states_ = init;
}

/*****Data Processing Functions*****/
void FlyappyRos::updateLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    flyappy_.laser_data_.angle_min = msg->angle_min;
    flyappy_.laser_data_.angle_max = msg->angle_max;
    flyappy_.laser_data_.angle_increment = msg->angle_increment;
    flyappy_.laser_data_.scan_time = msg->scan_time;
    flyappy_.laser_data_.time_increment = msg->time_increment;
    flyappy_.laser_data_.range_min = msg->range_min;
    flyappy_.laser_data_.range_max = msg->range_max;

    flyappy_.laser_data_.range_max_current = 0; // initial value
    flyappy_.laser_data_.range_min_current = flyappy_.laser_data_.range_max + 1; // initial value
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        flyappy_.laser_data_.ranges[i] = msg->ranges[i];
        flyappy_.laser_data_.intensities[i] = msg->intensities[i];

        // find current smallest range
        if (msg->ranges[i] > flyappy_.laser_data_.range_max_current)
        {
            flyappy_.laser_data_.range_max_current = msg->ranges[i];
            flyappy_.laser_data_.index_range_max_current = i;
        }

        // find current biggest range
        if (msg->ranges[i] < flyappy_.laser_data_.range_min_current)
        {
            flyappy_.laser_data_.range_min_current = msg->ranges[i];
            flyappy_.laser_data_.index_range_min_current = i;
        }
    }
}

void FlyappyRos::wallDataUpdate()
{
    std::vector<float> x_dist_temp;
    std::vector<float> y_index_obstacles;

    for (size_t i = 0; i < flyappy_.laser_data_.ranges.size(); ++i)
    {
        float x = flyappy_.get_position()[0] + flyappy_.laser_data_.ranges[i]*std::cos((i - 4)*flyappy_.laser_data_.angle_increment);
        float y = flyappy_.get_position()[1] + flyappy_.laser_data_.ranges[i]*std::sin((i - 4)*flyappy_.laser_data_.angle_increment);
    }
}