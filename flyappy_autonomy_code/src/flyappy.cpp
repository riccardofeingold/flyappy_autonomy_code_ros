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
    std::array<std::array<float, 3>, 9> fuzzy_laser_distances;
    for (size_t i = 0; i < fuzzy_laser_distances.size(); ++i)
    {
        fuzzy_laser_distances[i][0] = this->mfc_closing_.mfc(laser_data_.ranges[i]*std::cos((i - 4)*laser_data_.angle_increment));
        fuzzy_laser_distances[i][1] = this->mfc_near_.mfc(laser_data_.ranges[i]*std::cos((i - 4)*laser_data_.angle_increment));
        fuzzy_laser_distances[i][2] = this->mfc_far_.mfc(laser_data_.ranges[i]*std::cos((i - 4)*laser_data_.angle_increment));
    }

    //// weighted average
    std::array<float, 3> fuzzy_laser_variables; // 0 = near, 1 = closing, 2 = far
    fuzzy_laser_variables[0] = fuzzy_laser_distances[3][0]*w_lower_middle_laser + fuzzy_laser_distances[4][0]*w_middle_laser + fuzzy_laser_distances[5][0]*w_upper_middle_laser;
    fuzzy_laser_variables[1] = fuzzy_laser_distances[3][1]*w_lower_middle_laser + fuzzy_laser_distances[4][1]*w_middle_laser + fuzzy_laser_distances[5][1]*w_upper_middle_laser;
    fuzzy_laser_variables[2] = fuzzy_laser_distances[3][2]*w_lower_middle_laser + fuzzy_laser_distances[4][2]*w_middle_laser + fuzzy_laser_distances[5][2]*w_upper_middle_laser;
    
    //// fuzzify velocity measurment
    std::array<float, 4> fuzzy_vel_x_variables; // [0, 1, 2, 3] = [very slow, slow, medium, fast]
    fuzzy_vel_x_variables[0] = this->vel_very_slow_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[1] = this->vel_slow_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[2] = this->vel_medium_.mfc(this->velocity_[0]);
    fuzzy_vel_x_variables[3] = this->vel_fast_.mfc(this->velocity_[0]);

    // inference: Rules
    // [0, 1, 2, 3, 4, 5] = [fast break, med break, slow break, slow acc, med acc, fast acc]
    std::array<float, 6> output_fuzzy_variables = {0,0,0,0,0,0};
    //// 1. Rule: If (((far && closing) or closing or (closing && near)) && not very slow) or near then fast breaking
    output_fuzzy_variables[0] = std::max(std::min(std::max({std::min(fuzzy_laser_variables[2], fuzzy_laser_variables[1]), fuzzy_laser_variables[1], std::min(fuzzy_laser_variables[1], fuzzy_laser_variables[0])}), 1 - fuzzy_vel_x_variables[0]), fuzzy_laser_variables[0]);
    //// 2. Rule: If far && (very slow or slow or medium) then fast accelerating
    output_fuzzy_variables[5] = std::min({fuzzy_laser_variables[2], std::max({fuzzy_vel_x_variables[0], fuzzy_vel_x_variables[1], fuzzy_vel_x_variables[2]})});
    
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
        sum += areas[i];

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

void Flyappy::find_target_y()
{

    if (states_ == transition)
        return;
    else if (states_ == move_forward)
    {
        int optimal_gate_height = gate_height/(path_.lower_bound - path_.upper_bound) * map_.resolution;

        std::vector<int8_t> first_wall_copy = map_.first_wall;
        first_wall_copy.insert(first_wall_copy.begin(), 100);
        first_wall_copy.push_back(100);
        std::vector<int> signs(first_wall_copy.size());
        std::vector<int> zero_starts;
        std::vector<int> zero_stops;
        // difference: out[i] = vec[i+1] - vec[i]
        for (size_t i = 0; i < first_wall_copy.size(); ++i)
        {
            if (i < first_wall_copy.size() - 1)
            {
                signs[i] = first_wall_copy[i+1] - first_wall_copy[i];
                if (signs[i] == -100)
                    zero_starts.push_back(i);
                else if (signs[i] == 100)
                    zero_stops.push_back(i);
            }
        }

        std::vector<int> zero_count(zero_starts.size());
        for (size_t i = 0; i < zero_count.size(); ++i)
        {
            zero_count[i] = zero_stops[i] - zero_starts[i];
        }

        std::vector<int> zero_selection(zero_count.size());
        std::vector<int> zero_selection_index;
        int biggest_gap = 0;
        for (size_t i = 0; i < zero_count.size(); ++i)
        {
            zero_selection[i] = zero_count[i] - optimal_gate_height;
            if (zero_selection[i] < 0.1*optimal_gate_height && zero_selection[i] > 0)
                zero_selection_index.push_back(i);
        }
        if (zero_selection_index.size() == 1)
        {
            std::cout << "only one" << std::endl;
            float middle = (float)(zero_starts[zero_selection_index[0]] + zero_stops[zero_selection_index[0]])/2;
            path_.goal_pos[1] = (float)middle / map_.resolution * (path_.lower_bound - path_.upper_bound) + path_.upper_bound;
        } else if (zero_selection_index.size() > 1)
        {
            std::cout << "choose closest" << std::endl;
            std::vector<float> goals;
            float min = 999;
            for (size_t i = 0; i < zero_selection_index.size(); ++i)
            {
                float middle = (float)(zero_starts[zero_selection_index[i]] + zero_stops[zero_selection_index[i]])/2;
                float goal = (float)middle / map_.resolution * (path_.lower_bound - path_.upper_bound) + path_.upper_bound;
                goals.push_back(goal);
            }
            for (size_t i = 0; i < goals.size(); ++i)
            {
                if (std::abs(goals[i] - this->position_[1]) < min)
                {
                    min = std::abs(goals[i] - this->position_[1]);
                    path_.goal_pos[1] = goals[i];
                }
            }
        }else
        {
            std::cout << "explore" << std::endl;
            int max_count = 0;
            int max_count_index = 0;
            for (size_t i = 0; i < zero_count.size(); ++i)
            {
                if (zero_count[i] > max_count)
                {
                    max_count = zero_count[i];
                    max_count_index = i;
                }
            }
            float middle = (float)(zero_starts[max_count_index] + zero_stops[max_count_index])/2;
            path_.goal_pos[1] = middle / map_.resolution * (path_.lower_bound - path_.upper_bound) + path_.upper_bound;
        }
    }
}

void Flyappy::find_upper_lower_boundary()
{
    // set upper bound
    float y = -laser_data_.ranges[8]*std::sin(4*laser_data_.angle_increment);
    if (laser_data_.ranges[8] > 3.4 && !path_.upper_bound_set)
    {
        path_.goal_pos[1] -= 0.1;
    }
    else if (laser_data_.ranges[8] <= 3.4 && !path_.upper_bound_set)
    {
        path_.upper_bound = this->get_position()[1] + y + safety_margin;
        path_.upper_bound_set = true;
    }
    // set lower bound
    y = -laser_data_.ranges[0]*std::sin(-4*laser_data_.angle_increment);
    if (path_.upper_bound_set && !path_.lower_bound_set && laser_data_.ranges[0] > 3.4)
    {
        path_.goal_pos[1] += 0.1;
    }
    else if (path_.upper_bound_set && !path_.lower_bound_set && laser_data_.ranges[0] <= 3.4)
    {
        path_.lower_bound = this->get_position()[1] + y - safety_margin;
        path_.lower_bound_set = true;
        path_.goal_pos[1] = 0;
    }
    pid_control_y();
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