#include "flyappy_autonomy_code/flyappy.hpp"

Flyappy::Flyappy() = default;

void Flyappy::set_acceleration(float ax, float ay)
{
    this->acceleration_[0] = ax;
    this->acceleration_[1] = ay;
}

Eigen::Vector2d Flyappy::get_acceleration()
{
    return this->acceleration_;
}

void Flyappy::set_velocity(float vx, float vy)
{
    this->velocity_[0] = vx;
    this->velocity_[1] = vy;
}

Eigen::Vector2d Flyappy::get_velocity()
{
    return this->velocity_;
}

void Flyappy::set_initial_position(float px, float py)
{
    this->position_[0] = px;
    this->position_[1] = py;
}

void Flyappy::set_position(float px, float py)
{
    this->position_[0] += px;
    this->position_[1] += py;
}

Eigen::Vector2d Flyappy::get_position()
{
    return this->position_;
}