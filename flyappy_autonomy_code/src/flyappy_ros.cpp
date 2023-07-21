#include "flyappy_autonomy_code/flyappy_ros.hpp"

#define DEBUG false

constexpr uint32_t QUEUE_SIZE = 5u;
constexpr int MAP_RESOLUTION = 512;
constexpr float LASER_RANGE_MAX = 3.54;
constexpr float FRONT_MARGIN = 0.2;
constexpr float BACK_MARGIN = 0.15;
constexpr float KPY = 60;
constexpr float KDY = 1000;

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
    flyappy_.map_.resolution = MAP_RESOLUTION;
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
    flyappy_.path_.goal_pos[0] = 0.5;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = KPY;
    flyappy_.pid_.kd[0] = 0.0;
    flyappy_.pid_.kd[1] = KDY;

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
    wallDataUpdate();

    #if debug
    switch (flyappy_.states_)
    {
        case 0:
            std::cout << "Current state: init" << std::endl;
            break;
        case 1:
            std::cout << "Current state: transition" << std::endl;
            break;
        case 2: 
            std::cout << "Current state: move_forward" << std::endl;
            break;
        case 3:
            std::cout << "Current state: search_gap" << std::endl;
            break;
        case 4:
            std::cout << "Current state: explore" << std::endl;
            break;
        case 5:
            std::cout << "Current state: gap_found" << std::endl;
            break;
    }
    std::cout << "explored: " << flyappy_.explored << " " << flyappy_.stuck_counter_ << std::endl;
    #endif

    if (flyappy_.map_.front_obstacle_update)
    {
        flyappy_.map_.front_obstacle_max_x = flyappy_.map_.first_wall_max_x + FRONT_MARGIN;
        flyappy_.map_.front_obstacle_min_x = flyappy_.map_.first_wall_min_x - BACK_MARGIN;
    }

    if (flyappy_.explored && flyappy_.get_velocity()[0] < flyappy_.vel_limit_x/12)
    {
        flyappy_.explored = false;
    }

    if (flyappy_.map_.first_wall_min_x == 999)
    {
        flyappy_.states_ = init;
    }
    else if (flyappy_.get_position()[0] < flyappy_.map_.front_obstacle_min_x)
    {
        flyappy_.states_ = move_forward;
    }
    else if (flyappy_.get_position()[0] >= flyappy_.map_.front_obstacle_min_x && flyappy_.get_position()[0] <= flyappy_.map_.front_obstacle_max_x && flyappy_.map_.front_obstacle_update == true)
    {
        flyappy_.states_ = transition;
    }
    else if (flyappy_.get_position()[0] >= flyappy_.map_.front_obstacle_min_x && flyappy_.get_position()[0] <= flyappy_.map_.front_obstacle_max_x && flyappy_.map_.front_obstacle_update == false)
    {
        flyappy_.states_ = search_gap;
    }
    else if (flyappy_.get_position()[0] > flyappy_.map_.front_obstacle_max_x)
    {
        flyappy_.map_.front_obstacle_update = true;
        flyappy_.explored = false;
        flyappy_.explore_top_ = false;
        flyappy_.explore_bottom_ = false;
        flyappy_.states_ = move_forward;
        flyappy_.map_.front_obstacle_max_x = flyappy_.map_.first_wall_max_x + FRONT_MARGIN;
        flyappy_.map_.front_obstacle_min_x = flyappy_.map_.first_wall_min_x - BACK_MARGIN;
    }

    if (flyappy_.states_ == move_forward || flyappy_.states_ == transition || flyappy_.states_ == search_gap)
    {
        if (flyappy_.get_velocity()[0] < flyappy_.vel_limit_x/12 && (flyappy_.states_ == transition || flyappy_.states_ == search_gap) && !flyappy_.explored)
        {
            ++flyappy_.stuck_counter_;
            if (flyappy_.stuck_counter_ > 60)
            {
                flyappy_.up_down_exploration();
                flyappy_.pid_control_y();
            } else
            {
                flyappy_.find_target_y();
                flyappy_.pid_control_y();
                flyappy_.fuzzy_control();
            }
        } else
        {
            if (flyappy_.stuck_counter_ > 0 && flyappy_.stuck_counter_ < 60)
            {
                flyappy_.pid_control_y();
                flyappy_.set_acceleration(-3, flyappy_.get_acceleration()[1]);
            }
            else 
            {
                flyappy_.find_target_y();
                flyappy_.pid_control_y();
                flyappy_.fuzzy_control();
            }
        }
    }
    else if (flyappy_.states_ == init)
    {
        if (flyappy_.path_.lower_bound_set && flyappy_.path_.upper_bound_set && flyappy_.map_.first_wall_min_x != 999)
        {
            flyappy_.states_ = move_forward;
        }else
        {
            if (flyappy_.path_.lower_bound_set && flyappy_.path_.upper_bound_set)
            {
                flyappy_.pid_control_y();
                flyappy_.fuzzy_control();
            } else 
            {
                flyappy_.find_upper_lower_boundary();
            }
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
    flyappy_.map_.resolution = MAP_RESOLUTION;
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
    flyappy_.path_.goal_pos[0] = 0.5;
    flyappy_.path_.goal_pos[1] = 0;
    flyappy_.path_.prev_pos_error[0] = 0;
    flyappy_.path_.prev_pos_error[1] = 0;

    // PID initializing
    flyappy_.pid_.kp[0] = 0; // kp for x
    flyappy_.pid_.kp[1] = KPY; // kp for y
    flyappy_.pid_.kd[0] = 0.0; // kd for x
    flyappy_.pid_.kd[1] = KDY; // kd for y

    // states
    flyappy_.states_ = init;
    flyappy_.explored = false;
    // reseting time
    flyappy_.num_time_steps_ = 0;
    flyappy_.reset_counter_ = 0;
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
    flyappy_.laser_data_.range_max = LASER_RANGE_MAX;

    flyappy_.laser_data_.range_max_current = 0; // initial value
    flyappy_.laser_data_.range_min_current = flyappy_.laser_data_.range_max + 1; // initial value
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
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
    std::vector<int> y_index_obstacles;

    for (unsigned int i = 0; i < flyappy_.laser_data_.ranges.size(); ++i)
    {
        if (flyappy_.laser_data_.ranges[i] < flyappy_.laser_data_.range_max)
        {
            float x = flyappy_.get_position()[0] + flyappy_.laser_data_.ranges[i]*std::cos((i - 4)*flyappy_.laser_data_.angle_increment);
            float y = flyappy_.get_position()[1] - flyappy_.laser_data_.ranges[i]*std::sin((i - 4)*flyappy_.laser_data_.angle_increment);
            if (y < flyappy_.path_.lower_bound && y > flyappy_.path_.upper_bound)
            {
                #if debug
                std::cout << "angle increment: " << flyappy_.laser_data_.angle_increment << "range: " << flyappy_.laser_data_.ranges[i] << " y pos: " << flyappy_.get_position()[1] << " y_component: " << flyappy_.laser_data_.ranges[i]*std::sin((i - 4)*flyappy_.laser_data_.angle_increment) << " sin(alpha): " << std::sin((i - 4)*flyappy_.laser_data_.angle_increment) << " y" << i << ": " << y << " lower bound: " << flyappy_.path_.lower_bound << " upper bound: " << flyappy_.path_.upper_bound << std::endl;
                #endif
                x_dist_temp.push_back(x);
                int y_index = (y - flyappy_.path_.upper_bound) / (flyappy_.path_.lower_bound - flyappy_.path_.upper_bound) * flyappy_.map_.resolution; // index = 0 => at top
                y_index_obstacles.push_back(y_index);
            }
        }
    }

    if (x_dist_temp.size() > 0)
    {
        float x_range = *std::max_element(x_dist_temp.begin(), x_dist_temp.end()) - *std::min_element(x_dist_temp.begin(), x_dist_temp.end());

        #if debug
        std::cout << "x range: " << x_range << " pos x: " << flyappy_.get_position()[0] << " pos y: " << flyappy_.get_position()[1] << " goal y: " << flyappy_.path_.goal_pos[1] << " min_x: " << *std::min_element(x_dist_temp.begin(), x_dist_temp.end()) << " 2nd wall min x: " << flyappy_.map_.second_wall_min_x << std::endl;
        #endif
        if (x_range < flyappy_.wall_width)
        {
            if (*std::min_element(x_dist_temp.begin(), x_dist_temp.end()) > flyappy_.map_.second_wall_min_x - 0.2)
            {
                flyappy_.map_.first_wall = flyappy_.map_.second_wall;
                flyappy_.map_.first_wall_max_x = flyappy_.map_.second_wall_max_x;
                flyappy_.map_.first_wall_min_x = flyappy_.map_.second_wall_min_x;
                flyappy_.map_.front_obstacle_update = false;
                ++flyappy_.reset_counter_;
                std::cout << "RESET" << flyappy_.reset_counter_ << std::endl;
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