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

void Flyappy::pid_control_x()
{
    float current_error = path_.goal_pos[0] - laser_data_.ranges[4];
    this->acceleration_[0] = pid_.kp[0] * current_error + pid_.kd[0] * (current_error - path_.prev_pos_error[0]);  
    path_.prev_pos_error[0] = current_error;
}

void Flyappy::pid_control_y()
{
    float current_error = path_.goal_pos[1] - this->position_[1];
    this->acceleration_[1] = -(pid_.kp[1] * current_error + pid_.kd[1] * (current_error - path_.prev_pos_error[1]));  
    path_.prev_pos_error[1] = current_error;
}

void Flyappy::fuzzy_control()
{
    // weighting of front lasers
    float w_upper_middle_laser = 0.333;
    float w_middle_laser = 0.333;
    float w_lower_middle_laser = 0.333;

    // fuzzyfication
    std::array<std::array<float, 3>, 3> fuzzy_laser_distances;
    //// lower middle laser
    std::cout << "upper distance" << laser_data_.ranges[3]*std::cos(-laser_data_.angle_increment) << std::endl;
    fuzzy_laser_distances[0][0] = this->mfc_closing_.mfc(laser_data_.ranges[3]*std::cos(-laser_data_.angle_increment));
    fuzzy_laser_distances[0][1] = this->mfc_near_.mfc(laser_data_.ranges[3]*std::cos(-laser_data_.angle_increment));
    fuzzy_laser_distances[0][2] = this->mfc_far_.mfc(laser_data_.ranges[3]*std::cos(-laser_data_.angle_increment));

    //// middle laser
    std::cout << "middle distance" << laser_data_.ranges[4] << std::endl;
    fuzzy_laser_distances[1][0] = this->mfc_closing_.mfc(laser_data_.ranges[4]);
    fuzzy_laser_distances[1][1] = this->mfc_near_.mfc(laser_data_.ranges[4]);
    fuzzy_laser_distances[1][2] = this->mfc_far_.mfc(laser_data_.ranges[4]);

    //// upper middle laser
    std::cout << "lower distance" << laser_data_.ranges[3]*std::cos(-laser_data_.angle_increment) << std::endl;
    fuzzy_laser_distances[2][0] = this->mfc_closing_.mfc(laser_data_.ranges[5]*std::cos(laser_data_.angle_increment));
    fuzzy_laser_distances[2][1] = this->mfc_near_.mfc(laser_data_.ranges[5]*std::cos(laser_data_.angle_increment));
    fuzzy_laser_distances[2][2] = this->mfc_far_.mfc(laser_data_.ranges[5]*std::cos(laser_data_.angle_increment));
    
    //// weighted average
    std::array<float, 3> fuzzy_laser_variables; // 0 = near, 1 = closing, 2 = far
    fuzzy_laser_variables[0] = fuzzy_laser_distances[0][0]*w_lower_middle_laser + fuzzy_laser_distances[1][0]*w_middle_laser + fuzzy_laser_distances[2][0]*w_upper_middle_laser;
    fuzzy_laser_variables[1] = fuzzy_laser_distances[0][1]*w_lower_middle_laser + fuzzy_laser_distances[1][1]*w_middle_laser + fuzzy_laser_distances[2][1]*w_upper_middle_laser;
    fuzzy_laser_variables[2] = fuzzy_laser_distances[0][2]*w_lower_middle_laser + fuzzy_laser_distances[1][2]*w_middle_laser + fuzzy_laser_distances[2][2]*w_upper_middle_laser;
    
    // for (size_t i = 0; i < fuzzy_laser_variables.size(); ++i)
    // {
    //     std::cout << fuzzy_laser_variables[i] << std::endl;
    // }
    
    //// fuzzify velocity measurment
    std::array<float, 4> fuzzy_vel_x_variables; // [0, 1, 2, 3] = [very slow, slow, medium, fast]
    fuzzy_vel_x_variables[0] = this->vel_very_slow_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[1] = this->vel_slow_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[2] = this->vel_medium_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[3] = this->vel_fast_.mfc(this->velocity_[0]);

    // for (size_t i = 0; i < fuzzy_vel_x_variables.size(); ++i)
    // {
    //     std::cout << fuzzy_vel_x_variables[i] << std::endl;
    // }

    // inference
    // [0, 1, 2, 3, 4, 5] = [fast break, med break, slow break, slow acc, med acc, fast acc]
    std::array<float, 6> output_fuzzy_variables = {0,0,0,0,0,0};
    //// 1. Rule: If (((far && closing) or closing or (closing && near)) && not very slow) or near then fast breaking
    output_fuzzy_variables[0] = std::max(std::min(std::max({std::min(fuzzy_laser_variables[2], fuzzy_laser_variables[1]), fuzzy_laser_variables[1], std::min(fuzzy_laser_variables[1], fuzzy_laser_variables[0])}), 1 - fuzzy_vel_x_variables[0]), fuzzy_laser_variables[0]);
    // output_fuzzy_variables[0] = std::min(fuzzy_laser_variables[0], 1 - fuzzy_vel_x_variables[0]);
    //// 2. Rule: If far && (very slow or slow or medium) then fast accelerating
    output_fuzzy_variables[5] = std::min({fuzzy_laser_variables[2], std::max({fuzzy_vel_x_variables[0], fuzzy_vel_x_variables[1], fuzzy_vel_x_variables[2]})});
    //// 3. Rule: If (closing && not slow) or (closing && near && not very slow) then medium breaking
    // output_fuzzy_variables[1] = std::max(std::min(fuzzy_laser_variables[1], 1 - fuzzy_vel_x_variables[1]), std::min({fuzzy_laser_variables[1], fuzzy_laser_variables[0], 1 - fuzzy_vel_x_variables[0]}));
    //// 4. Rule: If (closing && very slow) then medium acceleration.
    // output_fuzzy_variables[5] = std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[0]);
    //// 5. Rule: if (closing && very slow) then fast accelerating.
    // output_fuzzy_variables[5] = std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[0]);
    // //// 1. Rule: If (near && medium) or (near && fast) or (closing && fast) or (far && closing) then fast break.
    // output_fuzzy_variables[0] = std::max({std::min(fuzzy_laser_variables[0], fuzzy_vel_x_variables[2]), std::min(fuzzy_vel_x_variables[0], fuzzy_vel_x_variables[3]), std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[2]), std::min(fuzzy_laser_variables[2], fuzzy_laser_variables[1])});
    // //// 2. Rule: If (near && slow) or (closing && medium) then medium breaking.
    // output_fuzzy_variables[1] = std::max({std::min(fuzzy_laser_variables[0], fuzzy_vel_x_variables[1]), std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[2])});
    // //// 3. Rule: If (near && very slow) or (far && fast) then slow breaking.
    // output_fuzzy_variables[2] = std::max({std::min(fuzzy_laser_variables[0], fuzzy_vel_x_variables[0]), std::min(fuzzy_laser_variables[2], fuzzy_vel_x_variables[3])});
    // //// 4. Rule: If (closing && slow) or (far && medium) then slow accelerating.
    // output_fuzzy_variables[3] = std::max({std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[1]), std::min(fuzzy_laser_variables[2], fuzzy_vel_x_variables[2])});
    // //// 5. Rule: If (closing && very slow) or (far && slow) then medium accelerating.
    // output_fuzzy_variables[4] = std::max({std::min(fuzzy_laser_variables[1], fuzzy_vel_x_variables[0]), std::min(fuzzy_laser_variables[2], fuzzy_vel_x_variables[1])});
    // //// 6. Rule: If (far && very_slow) then fast accelerating.
    // output_fuzzy_variables[5] = std::max({fuzzy_laser_variables[2], fuzzy_vel_x_variables[0]});

    // //// 1. Rule: If distance is far then fast acceleration.
    // output_fuzzy_variables[5] = fuzzy_laser_variables[2];
    // //// 2. Rule: If distance is far and closing and slow velocity then slow acceleration
    // output_fuzzy_variables[4] = std::min<float>(std::min<float>(fuzzy_laser_variables[1], fuzzy_laser_variables[2]), fuzzy_vel_x_variables[0]);
    // //// 3. Rule: If velocity is fast or very slow then stop accelerating.
    // output_fuzzy_variables[3] = std::min<float>(fuzzy_vel_x_variables[3], fuzzy_vel_x_variables[0]);
    // //// 4. If distance is far and closing then slow breaking.
    // output_fuzzy_variables[2] = std::min<float>(fuzzy_laser_variables[1], fuzzy_laser_variables[2]);
    // //// 5. If distance is is closing and near then medium breaking.
    // output_fuzzy_variables[1] = std::min<float>(fuzzy_laser_variables[1], fuzzy_laser_variables[0]);
    // //// 6. If distance is distance closing or near then fast breaking.
    // output_fuzzy_variables[0] = std::min<float>(fuzzy_laser_variables[1], fuzzy_laser_variables[0]);

    // for (size_t i = 0; i < output_fuzzy_variables.size(); ++i)
    // {
    //     std::cout << output_fuzzy_variables[i] << std::endl;
    // } 
    // defuzzification
    std::array<float, 6> areas;// [0,1,2,3,4,5] = [fast breaking, medium breaking, slow breaking, stop acc, slow acc, fast acc]
    areas[0] = (float)this->fast_breaking_.area_mfc(output_fuzzy_variables[0])/2;
    areas[1] = this->medium_breaking_.area_mfc(output_fuzzy_variables[1]);
    areas[2] = this->slow_breaking_.area_mfc(output_fuzzy_variables[2]);
    areas[3] = this->slow_acc_.area_mfc(output_fuzzy_variables[3]);
    areas[4] = this->med_acc_.area_mfc(output_fuzzy_variables[4]);
    areas[5] = (float)this->fast_acc_.area_mfc(output_fuzzy_variables[5])/2;

    float sum = 0;
    for (size_t i = 0; i < areas.size(); ++i)
    {
        sum += areas[i];
        // std::cout << areas[i] << std::endl;
    }
    float weighted_sum = areas[0]*this->fast_breaking_.parameters_[1] + 
                         areas[1]*this->medium_breaking_.parameters_[1] +
                         areas[2]*this->slow_breaking_.parameters_[1] +
                         areas[3]*this->slow_acc_.parameters_[1] +
                         areas[4]*this->med_acc_.parameters_[1] +
                         areas[5]*this->fast_acc_.parameters_[1];
    if (sum > 0)
        this->acceleration_[0] = (float)weighted_sum/sum;
    else
        this->acceleration_[0] = 0;
}

void Flyappy::path_planning()
{

}

// testing functions
void Flyappy::pid_control_testing()
{
    if (std::abs(path_.goal_pos[1] - this->position_[1]) < 0.05 && this->num_time_steps_% 60 == 0)
    {
        float random_number = (float)std::rand()/RAND_MAX;
        if (random_number > 0.5)
            path_.goal_pos[1] = random_number;
        else
            path_.goal_pos[1] = -random_number;
    }
    this->pid_control_y();
}