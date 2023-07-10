#pragma once
#include <eigen3/Eigen/Dense>
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

LaserScanData laser_data_;

struct Map
{
  bool initialized;
  float resolution;
  int width;
  int height;
  Eigen::Vector2d origin; // TODO: setting the origin_x to where the bird is and y is in the middle of the frame
  Eigen::MatrixXi occupancy_matrix;
};

Map map_;

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

    // global constant variables we should be 
    // initiliazed at compile time
    static constexpr int fps = 30;
    static constexpr float periode = (float)1/fps;
    static constexpr float acc_limit_x = 3;
    static constexpr float acc_limit_y = 35;
    static constexpr float meters_per_grid_box = 0.01;
    static constexpr int width_pixel_size = 432;
    static constexpr int height_pixel_size = 512;

  private:
    Eigen::Vector2d acceleration_; // in meters/s²
    Eigen::Vector2d velocity_; // in meters/s
    Eigen::Vector2d position_; // in meters
};
