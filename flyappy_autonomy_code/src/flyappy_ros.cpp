#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;
constexpr float FRONT_MARGIN = 0.2;
constexpr float BACK_MARGIN = 0.15;
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
    flyappy_.map_.resolution = 300;
    flyappy_.map_.first_wall = std::vector<int8_t>(flyappy_.map_.resolution, 0);
    flyappy_.map_.second_wall = std::vector<int8_t>(flyappy_.map_.resolution, 0);
    flyappy_.map_.first_wall_max_x = 0;
    flyappy_.map_.first_wall_min_x = 999;
    flyappy_.map_.second_wall_max_x = 0;
    flyappy_.map_.second_wall_min_x = 999;
    flyappy_.map_.front_obstacle_max_x = 0;
    flyappy_.map_.front_obstacle_min_x = 999;
    flyappy_.map_.front_obstacle_update = true;

    // path planning initializing
    flyappy_.path_.goal_pos[0] = 0;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = 60;
    flyappy_.pid_.kd[0] = 0.0;
    flyappy_.pid_.kd[1] = 1000;

    // initialize state
    flyappy_.states_ = init;
    flyappy_.prev_states_ = init;
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
    wallDataUpdate();

    if (flyappy_.map_.front_obstacle_update)
    {
        flyappy_.map_.front_obstacle_max_x = flyappy_.map_.first_wall_max_x + FRONT_MARGIN;
        flyappy_.map_.front_obstacle_min_x = flyappy_.map_.first_wall_min_x - BACK_MARGIN;
    }
    if (flyappy_.map_.first_wall_min_x == 999)
    {
        flyappy_.prev_states_ = flyappy_.states_;
        flyappy_.states_ = init;
    }
    else if (flyappy_.get_position()[0] < flyappy_.map_.front_obstacle_min_x)
    {
        flyappy_.prev_states_ = flyappy_.states_;
        flyappy_.states_ = move_forward;
    }
    else if (flyappy_.get_position()[0] >= flyappy_.map_.front_obstacle_min_x && flyappy_.get_position()[0] <= flyappy_.map_.front_obstacle_max_x && flyappy_.map_.front_obstacle_update == true)
    {
        flyappy_.prev_states_ = flyappy_.states_;
        flyappy_.states_ = transition;
    }
    else if (flyappy_.get_position()[0] >= flyappy_.map_.front_obstacle_min_x && flyappy_.get_position()[0] <= flyappy_.map_.front_obstacle_max_x && flyappy_.map_.front_obstacle_update == false)
    {
        flyappy_.prev_states_ = flyappy_.states_;
        flyappy_.states_ = move_forward;
    }
    else if (flyappy_.get_position()[0] > flyappy_.map_.front_obstacle_max_x)
    {
        flyappy_.map_.front_obstacle_update = true;
        flyappy_.prev_states_ = flyappy_.states_;
        flyappy_.states_ = move_forward;
        flyappy_.map_.front_obstacle_max_x = flyappy_.map_.first_wall_max_x + FRONT_MARGIN;
        flyappy_.map_.front_obstacle_min_x = flyappy_.map_.first_wall_min_x - BACK_MARGIN;
    }

    if (flyappy_.states_ == move_forward or flyappy_.states_ == transition)
    {
        flyappy_.fuzzy_control();
        flyappy_.find_target_y();
        flyappy_.pid_control_y();
    }
    else if (flyappy_.states_ == init)
    {
        flyappy_.find_upper_lower_boundary();
        if (flyappy_.path_.lower_bound_set && flyappy_.path_.upper_bound_set)
        {
            flyappy_.prev_states_ = flyappy_.states_;
            flyappy_.states_ = move_forward;
        }
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
    flyappy_.map_.resolution = 300;
    flyappy_.map_.first_wall = std::vector<int8_t>(flyappy_.map_.resolution, 0);
    flyappy_.map_.second_wall = std::vector<int8_t>(flyappy_.map_.resolution, 0);
    flyappy_.map_.first_wall_max_x = 0;
    flyappy_.map_.first_wall_min_x = 999;
    flyappy_.map_.second_wall_max_x = 0;
    flyappy_.map_.second_wall_min_x = 999;
    flyappy_.map_.front_obstacle_max_x = 0;
    flyappy_.map_.front_obstacle_min_x = 999;
    flyappy_.map_.front_obstacle_update = true;


    // path planning initializing
    flyappy_.path_.goal_pos[0] = 0;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = 60; // kp for y
    flyappy_.pid_.kd[0] = 0.0; // kd for x
    flyappy_.pid_.kd[1] = 1000; // kd for y

    // states
    flyappy_.states_ = init;
    flyappy_.prev_states_ = init;
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

// TODO: Find out why there are some obstacles detected on the map although there is free space. 
//       This is probably the reason why the whole program is not working. Being not able to map
//       the data correctly results into wrong gap detection => which in the end leads to the explore mode.
void FlyappyRos::wallDataUpdate()
{
    std::vector<float> x_dist_temp;
    std::vector<int> y_index_obstacles;

    for (size_t i = 0; i < flyappy_.laser_data_.ranges.size(); ++i)
    {
        float x = flyappy_.get_position()[0] + flyappy_.laser_data_.ranges[i]*std::cos((i - 4)*flyappy_.laser_data_.angle_increment);
        float y = flyappy_.get_position()[1] - flyappy_.laser_data_.ranges[i]*std::sin((i - 4)*flyappy_.laser_data_.angle_increment);

        if (y < flyappy_.path_.lower_bound && y > flyappy_.path_.upper_bound)
        {
            x_dist_temp.push_back(x);
            int y_index = (y - flyappy_.path_.upper_bound) / (flyappy_.path_.lower_bound - flyappy_.path_.upper_bound) * flyappy_.map_.resolution; // index = 0 => at top
            y_index_obstacles.push_back(y_index);
        }
    }

    if (x_dist_temp.size() > 0)
    {
        float x_range = *std::max_element(x_dist_temp.begin(), x_dist_temp.end()) - *std::min_element(x_dist_temp.begin(), x_dist_temp.end());

        if (x_range < flyappy_.wall_width)
        {
            if (*std::min_element(x_dist_temp.begin(), x_dist_temp.end()) > flyappy_.map_.second_wall_min_x - 0.2)
            {
                flyappy_.map_.first_wall = flyappy_.map_.second_wall;
                flyappy_.map_.first_wall_max_x = flyappy_.map_.second_wall_max_x;
                flyappy_.map_.first_wall_min_x = flyappy_.map_.second_wall_min_x;
                flyappy_.map_.front_obstacle_update = false;
                
                // reseting second wall
                flyappy_.map_.second_wall = std::vector<int8_t>(flyappy_.map_.resolution, 0);
                flyappy_.map_.second_wall_max_x = 0;
                flyappy_.map_.second_wall_min_x = 999;
            }
            flyappy_.map_.first_wall_max_x = std::max(*std::max_element(x_dist_temp.begin(), x_dist_temp.end()), flyappy_.map_.first_wall_max_x);
            flyappy_.map_.first_wall_min_x = std::min(*std::min_element(x_dist_temp.begin(), x_dist_temp.end()), flyappy_.map_.first_wall_min_x);
            
            for (size_t i = 0; i < y_index_obstacles.size(); ++i)
            {
                flyappy_.map_.first_wall[y_index_obstacles[i]] = 100;
            }
        } else
        {
            float first_second_border_x = (float)(*std::max_element(x_dist_temp.begin(), x_dist_temp.end()) + *std::min_element(x_dist_temp.begin(), x_dist_temp.end()))/2;
            std::vector<float> front_x;
            std::vector<float> back_x;
            
            for (size_t i = 0; i < x_dist_temp.size(); ++i)
            {
                if (x_dist_temp[i] < first_second_border_x)
                {
                    front_x.push_back(x_dist_temp[i]);
                    flyappy_.map_.first_wall[y_index_obstacles[i]] = 100;
                } else if (x_dist_temp[i] >= first_second_border_x)
                {
                    back_x.push_back(x_dist_temp[i]);
                    flyappy_.map_.second_wall[y_index_obstacles[i]] = 100;
                }
            }

            flyappy_.map_.first_wall_max_x = std::max(*std::max_element(front_x.begin(), front_x.end()), flyappy_.map_.first_wall_max_x);
            flyappy_.map_.first_wall_min_x = std::min(*std::min_element(front_x.begin(), front_x.end()), flyappy_.map_.first_wall_min_x);
            flyappy_.map_.second_wall_max_x = std::max(*std::max_element(back_x.begin(), back_x.end()), flyappy_.map_.second_wall_max_x);
            flyappy_.map_.second_wall_min_x = std::min(*std::min_element(back_x.begin(), back_x.end()), flyappy_.map_.second_wall_min_x);
        }
    }

    // publish map data
    std::vector<int8_t> grid;
    grid.insert(grid.begin(), flyappy_.map_.second_wall.begin(), flyappy_.map_.second_wall.end());
    grid.insert(grid.end(), flyappy_.map_.first_wall.begin(), flyappy_.map_.first_wall.end());
    nav_msgs::OccupancyGrid map_visualisation;
    map_visualisation.data = grid;
    map_visualisation.info.height = flyappy_.map_.resolution;
    map_visualisation.info.resolution = 50;
    map_visualisation.info.width = 2;
    pub_map_.publish(map_visualisation);
}