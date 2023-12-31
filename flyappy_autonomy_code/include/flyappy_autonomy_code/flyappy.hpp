#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <initializer_list>

// TODO: write brief comments for all functions
struct LaserScanData
{
  float angle_min;
  float angle_max;
  float angle_increment;
  float scan_time;
  float time_increment;
  float range_min; // smallest possible range
  float range_max; // largest possible range
  float range_min_current; // the smallest measured range currently
  int index_range_min_current; // the index of the smallest measured range currently
  float range_max_current; // the biggest measured range currently
  int index_range_max_current; // the index of the biggest measured range currently
  std::array<float, 9> ranges;
  std::array<float, 9> intensities; // not needed
};

struct Map
{
  bool front_obstacle_update;
  float resolution;
  float first_wall_max_x;
  float first_wall_min_x;
  float second_wall_max_x;
  float second_wall_min_x;
  float front_obstacle_max_x;
  float front_obstacle_min_x;
  std::vector<int8_t> first_wall;
  std::vector<int8_t> second_wall;
};

struct Path
{
  bool upper_bound_set = false;
  bool lower_bound_set = false;
  float upper_bound = 0;
  float lower_bound = 0;
  Eigen::Vector2d goal_pos;
  Eigen::Vector2d prev_pos_error;
};

struct PID
{
  Eigen::Vector2d kp;
  Eigen::Vector2d ki;
  Eigen::Vector2d kd;
};

enum States
{
  init,
  transition,
  move_forward,
  search_gap,
  explore,
  gap_found
};

class FuzzyMembershipFunctions
{
  public:
    // constructor
    FuzzyMembershipFunctions(const std::string &name, const std::vector<float> &params)
    {
      this->membership_name_ = name;
      this->parameters_ = params;
    }

    // membership function general
    float mfc(float x)
    {
      if (this->membership_name_ == "zmf")
        return this->z_function(x);
      else if (this->membership_name_ == "smf")
        return this->s_function(x);
      else if (this->membership_name_ == "trapmf")
        return this->trapezoidal_function(x);
      else if (this->membership_name_ == "trimf")
        return this->triangular_function(x);
      else if (this->membership_name_ == "gaussmf")
        return this->gaussian_function(x);
      else
        return 0;
    }

    // get area of function
    float area_mfc(float height)
    {
      if (this->membership_name_ == "trimf")
      {
        float width = std::abs(this->parameters_[0] - this->parameters_[2]);
        return width*height*(1 - (float)height/2);
      }
      else
        return 0;
    }

    // public variables
    std::vector<float> parameters_;

  private:
    // private variables
    std::string membership_name_;

    // membership functions
    float z_function(float x)
    {
      if (x <= this->parameters_[0])
        return 1;
      else if (x >= this->parameters_[0] && x <= (this->parameters_[0]+this->parameters_[1])/2)
        return 1 - 2*std::pow((x-this->parameters_[0])/(this->parameters_[1]-this->parameters_[0]), 2);
      else if (x >= (this->parameters_[0]+this->parameters_[1])/2 && x <= this->parameters_[1])
        return 2*std::pow((x-this->parameters_[1])/(this->parameters_[1]-this->parameters_[0]), 2);
      else if (x >= this->parameters_[1])
        return 0;
      else
        return 0;
    }

    float s_function(float x)
    {
      if (x <= this->parameters_[0])
        return 0;
      else if (x >= this->parameters_[0] && x <= (this->parameters_[0]+this->parameters_[1])/2)
        return 2*std::pow((x-this->parameters_[0])/(this->parameters_[1]-this->parameters_[0]), 2);
      else if (x >= (this->parameters_[0]+this->parameters_[1])/2 && x <= this->parameters_[1])
        return 1 - 2*std::pow((x-this->parameters_[1])/(this->parameters_[1]-this->parameters_[0]), 2);
      else if (x >= this->parameters_[1])
        return 1;
      else
        return 1;
    }

    float trapezoidal_function(float x)
    {
      float min_value = std::min<float>(std::min<float>((x - this->parameters_[0])/(this->parameters_[1] - this->parameters_[0]), 1), (this->parameters_[3] - x)/(this->parameters_[3] - this->parameters_[2]));
      return std::max<float>(min_value, 0);
    }

    float triangular_function(float x)
    {
      float min_value = std::min<float>((x-this->parameters_[0])/(this->parameters_[1]-this->parameters_[0]), (this->parameters_[2]-x)/(this->parameters_[2]-this->parameters_[1]));
      return std::max<float>(min_value, 0);
    }

    // [sigma, mu] = [0, 1]
    float gaussian_function(float x)
    {
      return std::exp(-std::pow(x - this->parameters_[1], 2)/(2*std::pow(this->parameters_[0], 2)));
    }
};

class Flyappy
{
  public:
    Flyappy();
    void set_acceleration(float ax, float ay);
    Eigen::Vector2d get_acceleration();
    void set_velocity(float vx, float vy);
    Eigen::Vector2d get_velocity();
    void set_position(float px, float py);
    Eigen::Vector2d get_position();
    void set_initial_position(float px, float py);

    // high level control
    void find_target_y();
    void fuzzy_control();
    void find_upper_lower_boundary();
    void up_down_exploration();
    void pid_control_x();
    void pid_control_y();

    // for testing
    void pid_control_testing();
    
    // Global variables
    int num_time_steps_ = 0;
    int reset_counter_ = 0;
    int stuck_counter_ = 0;
    bool explored = false;
    bool explore_top_ = false;
    bool explore_bottom_ = false;

    // global constant variables we should be 
    // initiliazed at compile time
    static constexpr int fps = 30;
    static constexpr float periode = (float)1/fps;
    static constexpr float acc_limit_x = 3;
    static constexpr float acc_limit_y = 35;
    static constexpr float meters_per_grid_box = 0.01;
    static constexpr int width_pixel_size = 432;
    static constexpr int height_pixel_size = 512;
    static constexpr float vel_limit_x = 2;
    static constexpr float safety_margin = 0.75;
    static constexpr float wall_width = 0.6;
    static constexpr float gate_height = 0.5;
    static constexpr float flyappy_size_height = 24*0.01;
    static constexpr float range_threshold = 1.7;
    
    // struct instances
    States states_;
    Map map_;
    Path path_;
    PID pid_;
    LaserScanData laser_data_;


  private:

    Eigen::Vector2d acceleration_; // in meters/s²
    Eigen::Vector2d velocity_; // in meters/s
    Eigen::Vector2d position_; // in meters

    // fuzzification membership functions
    FuzzyMembershipFunctions mfc_near_ = FuzzyMembershipFunctions(std::string("zmf"), std::vector<float>({0.4, 0.7}));
    FuzzyMembershipFunctions mfc_closing_ = FuzzyMembershipFunctions(std::string("trapmf"), std::vector<float>({0.5, 0.7, 1.0, 1.5}));
    FuzzyMembershipFunctions mfc_far_ = FuzzyMembershipFunctions(std::string("smf"), std::vector<float>({1, 2.5}));
    FuzzyMembershipFunctions vel_very_slow_ = FuzzyMembershipFunctions(std::string("zmf"), std::vector<float>({vel_limit_x/12, vel_limit_x/9}));
    FuzzyMembershipFunctions vel_slow_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({vel_limit_x/10, vel_limit_x/6, vel_limit_x/4.5}));
    FuzzyMembershipFunctions vel_medium_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({vel_limit_x/5.5, vel_limit_x/2.1, 3*vel_limit_x/4}));
    FuzzyMembershipFunctions vel_fast_ = FuzzyMembershipFunctions(std::string("smf"), std::vector<float>({vel_limit_x/2.2, vel_limit_x/1.2}));
    
    // defuzzification membership functions
    FuzzyMembershipFunctions fast_breaking_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({-3*acc_limit_x/2, -acc_limit_x, -acc_limit_x/2}));
    FuzzyMembershipFunctions medium_breaking_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({-acc_limit_x, -acc_limit_x/2, 0}));
    FuzzyMembershipFunctions slow_breaking_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({-acc_limit_x/3, -acc_limit_x/6, 0}));
    FuzzyMembershipFunctions slow_acc_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({0, acc_limit_x/6, acc_limit_x/3}));
    FuzzyMembershipFunctions med_acc_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({0, acc_limit_x/2, acc_limit_x}));
    FuzzyMembershipFunctions fast_acc_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({acc_limit_x/2, acc_limit_x, 3*acc_limit_x/2}));
    FuzzyMembershipFunctions go_up_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({0, acc_limit_y, 2*acc_limit_y}));
    FuzzyMembershipFunctions go_down_ = FuzzyMembershipFunctions(std::string("trimf"), std::vector<float>({-2*acc_limit_y, acc_limit_y, 0}));
};
