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
    map_.initialized = false;
    map_.resolution = 1;
    map_.origin[0] = 0;
    map_.origin[0] = 0;

    // path planning initializing
    path_.goal_pos[0] = laser_data_.range_max+1;
    path_.goal_pos[1] = 0;
    path_.prev_pos_error[0] = 0;
    path_.prev_pos_error[1] = 0;

    // PID initializing
    pid_.kp[0] = 0; // kp for x
    pid_.kp[1] = 20;
    pid_.kd[0] = 0.0;
    pid_.kd[1] = 150;
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
    
    ROS_INFO_STREAM("velocity" << flyappy_.get_velocity()[0]);
    // updating time steps variable
    ++flyappy_.num_time_steps_;
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    updateLaserData(msg);
    updateMapData();
    
    geometry_msgs::Vector3 acc_cmd;
    // flyappy_.pid_control_testing();
    flyappy_.fuzzy_control();
    // flyappy_.pid_control_x();
    acc_cmd.x = flyappy_.get_acceleration()[0];
    // ROS_INFO_STREAM(acc_cmd.x << " y:" << acc_cmd.y);
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
    map_.initialized = false;
    map_.resolution = 1;
    map_.origin[0] = 0;
    map_.origin[0] = 0;

    // path planning initializing
    path_.goal_pos[0] = laser_data_.range_max+1;
    path_.goal_pos[1] = 0;
    path_.prev_pos_error[0] = 0;
    path_.prev_pos_error[1] = 0;

    // PID initializing
    pid_.kp[0] = 0; // kp for x
    pid_.kp[1] = 20; // kp for y
    pid_.kd[0] = 0.0; // kd for x
    pid_.kd[1] = 200; // kd for y
}

/*****Data Processing Functions*****/
void FlyappyRos::updateLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_data_.angle_min = msg->angle_min;
    laser_data_.angle_max = msg->angle_max;
    laser_data_.angle_increment = msg->angle_increment;
    laser_data_.scan_time = msg->scan_time;
    laser_data_.time_increment = msg->time_increment;
    laser_data_.range_min = msg->range_min;
    laser_data_.range_max = msg->range_max;

    laser_data_.range_max_current = 0; // initial value
    laser_data_.range_min_current = laser_data_.range_max + 1; // initial value
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        laser_data_.ranges[i] = msg->ranges[i];
        laser_data_.intensities[i] = msg->intensities[i];

        // find current smallest range
        if (msg->ranges[i] > laser_data_.range_max_current)
        {
            laser_data_.range_max_current = msg->ranges[i];
            laser_data_.index_range_max_current = i;
        }

        // find current biggest range
        if (msg->ranges[i] < laser_data_.range_min_current)
        {
            laser_data_.range_min_current = msg->ranges[i];
            laser_data_.index_range_min_current = i;
        }
    }

    // ROS_INFO_STREAM("laser distance: " << laser_data_.ranges[4]);
}

void FlyappyRos::updateMapData()
{
    nav_msgs::OccupancyGrid grid_message; // used for visualization

    // setting origin & locate bird on map
    if (!map_.initialized)
    {
        float top_to_bird_distance_y = std::abs(laser_data_.ranges[8] * std::sin(4 * laser_data_.angle_increment)); // top laser ray: 4 = 8 - 4; 9 = index and 4 = middle number
        float bottom_to_bird_distance_y = std::abs(laser_data_.ranges[0] * std::sin(-4 * laser_data_.angle_increment));
        ROS_INFO_STREAM("Top: " << top_to_bird_distance_y);
        ROS_INFO_STREAM("Bottom: " << bottom_to_bird_distance_y);
        
        if (top_to_bird_distance_y == laser_data_.range_max)

        // locate bird on map
        flyappy_.set_initial_position(0.0, top_to_bird_distance_y);

        // define map size
        // map_.height = (top_to_bird_distance_y + bottom_to_bird_distance_y) / 0.01;
        // ROS_INFO_STREAM("height: " << map_.height);
        // map_.width = 400; // 400 pixels are equivalent to 4 meters which is enough for the obstacle detection
        // map_.occupancy_matrix = -Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Ones(map_.width, map_.height);
        // ROS_INFO_STREAM("pos: " << flyappy_.get_position());
        // ROS_INFO_STREAM("min: " << laser_data_.range_min_current);
        // ROS_INFO_STREAM("index: " << laser_data_.index_range_min_current);
        // ROS_INFO_STREAM("angle: " << (laser_data_.index_range_min_current - 4) * laser_data_.angle_increment);

        map_.initialized = true;
    }
    
    for (size_t i = 0; i < laser_data_.ranges.size(); ++i)
    {
        if (laser_data_.ranges[i] < laser_data_.range_max)
        {
            float pos_x = laser_data_.ranges[i] * std::cos((i - 4) * laser_data_.angle_increment);
            float pos_y = -laser_data_.ranges[i] * std::sin((i - 4) * laser_data_.angle_increment);

            // conversion to grid coordinates
            int grid_x = (flyappy_.get_position()[0] + pos_x) / flyappy_.meters_per_grid_box;
            int grid_y = (flyappy_.get_position()[1] + pos_y) / flyappy_.meters_per_grid_box;
            
            // assign a 100 to the location (grid_x, grid_y) to indicate obstacles
            // ROS_INFO("HELLO1");
            // ROS_INFO_STREAM("Grid_x: " << grid_x << "Grid_y: " << grid_y);
            // map_.occupancy_matrix(grid_x, grid_y) = 100;
            // ROS_INFO("HELLO2");
        } 
        else 
        {
            float pos_x = laser_data_.ranges[i] * std::cos((i - 4) * laser_data_.angle_increment);
            float pos_y = -laser_data_.ranges[i] * std::sin((i - 4) * laser_data_.angle_increment);

            // conversion to grid coordinates
            int grid_x = (flyappy_.get_position()[0] + pos_x) / flyappy_.meters_per_grid_box;
            int grid_y = (flyappy_.get_position()[1] + pos_y) / flyappy_.meters_per_grid_box;

            // assign a zero to the location (grid_x, grid_y) to indicate free space
            // ROS_INFO("HELLO3");
            // ROS_INFO_STREAM("Grid_x: " << grid_x << "Grid_y: " << grid_y);
            // map_.occupancy_matrix(grid_x, grid_y) = 0;
            // ROS_INFO("HELLO4");
        }
    }

    // turn struct data into nav_msgs version
    grid_message.info.height = map_.height;
    grid_message.info.width = map_.width;
    grid_message.info.resolution = map_.resolution;
    grid_message.info.origin.position.x = map_.origin[0];
    grid_message.info.origin.position.y = map_.origin[1];
    grid_message.info.origin.position.z = 0;
    grid_message.info.origin.orientation.w = 1;
    grid_message.info.origin.orientation.x = 0;
    grid_message.info.origin.orientation.y = 0;
    grid_message.info.origin.orientation.z = 0;
    grid_message.info.map_load_time = ros::Time::now();

    // for (size_t i = 0; i < map_.height; ++i)
    // {
    //     for (size_t j = 0; j < map_.width; ++j)
    //     {
    //         grid_message.data.push_back(map_.occupancy_matrix(i, j));
    //     }
    // }

    // pubish map
    pub_map_.publish(grid_message);
    grid_message.data.clear();
}