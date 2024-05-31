#include "imu_frame_checker.hpp"
#include <memory>
#include <numeric>
#include <deque>
#include <numeric>

namespace imu_frame_checker
{
    ImuFrameChecker::ImuFrameChecker() : Node("imu_frame_checker")
    {
        start_time_ = this->now();
        last_time_ = this->now();
        velocity_x_ = 0.0;
        position_x_ = 0.0;
        velocity_y_ = 0.0;
        position_y_ = 0.0;
        velocity_z_ = 0.0;
        position_z_ = 0.0;
        sample_count_ = 0;
        window_size_ = 10;
        imu_sub_ = this->create_subscription<ImuMsg>("/sensing/imu/imu_data", 10, std::bind(&ImuFrameChecker::imu_callback, this, std::placeholders::_1));
        
    }

    void ImuFrameChecker::imu_callback(const ImuMsg::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "I heard angular velocity x: '%f'", msg->angular_velocity.x);
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        if ((current_time - start_time_).seconds() <= 5.0)
        {
            RCLCPP_INFO(this->get_logger(), "機体を静止させてください");
            acceleration_bias_x_ += msg->linear_acceleration.x;
            acceleration_bias_y_ += msg->linear_acceleration.y;
            acceleration_bias_z_ += msg->linear_acceleration.z;
            sample_count_++;

        }else if (((current_time - start_time_).seconds() > 5.0) && (sample_count_ != 0))
        {
            RCLCPP_INFO(this->get_logger(), "バイアス校正を行います");
            acceleration_bias_x_ /= sample_count_;
            acceleration_bias_y_ /= sample_count_;
            acceleration_bias_z_ /= sample_count_;
            RCLCPP_INFO(this->get_logger(), "accelaration bias x : '%f'", acceleration_bias_x_);
            RCLCPP_INFO(this->get_logger(), "accelaration bias y : '%f'", acceleration_bias_y_);
            RCLCPP_INFO(this->get_logger(), "accelaration bias z : '%f'", acceleration_bias_z_);
            sample_count_ = 0;
        }else if (((current_time - start_time_).seconds() > 5.0) && ((current_time - start_time_).seconds() <= 8.0) && (sample_count_ == 0))
        {
            RCLCPP_INFO(this->get_logger(), "機体を前進させてください");
            double filtered_accel_x = applyLowPassFilter(accel_x_buffer_, msg->linear_acceleration.x);
            double filtered_accel_y = applyLowPassFilter(accel_y_buffer_, msg->linear_acceleration.y);
            double filtered_accel_z = applyLowPassFilter(accel_z_buffer_, msg->linear_acceleration.z);

            velocity_x_ += (filtered_accel_x - acceleration_bias_x_) * dt;
            position_x_ += velocity_x_ * dt;
            velocity_y_ += (filtered_accel_y - acceleration_bias_y_) * dt;
            position_y_ += velocity_y_ * dt;
            velocity_z_ += (filtered_accel_z - acceleration_bias_z_) * dt;
            position_z_ += velocity_z_ * dt;
        }else if (((current_time - start_time_).seconds() > 8.0))
        {
            if ((position_x_ > 0.0) && (position_x_ > std::abs(position_y_)*3) && (position_x_ > std::abs(position_z_)*3))
            {
                RCLCPP_INFO(this->get_logger(), "elapsed time: '%f'", (current_time - start_time_).seconds());
                RCLCPP_INFO(this->get_logger(), "position x: '%f'", position_x_);
                RCLCPP_INFO(this->get_logger(), "position y: '%f'", position_y_);
                RCLCPP_INFO(this->get_logger(), "position z: '%f'", position_z_);
                RCLCPP_INFO(this->get_logger(), "IMU frame is correct");
                //rclcpp::shutdown();
            }else{
                RCLCPP_INFO(this->get_logger(), "elapsed time: '%f'", (current_time - start_time_).seconds());
                RCLCPP_INFO(this->get_logger(), "position x: '%f'", position_x_);
                RCLCPP_INFO(this->get_logger(), "position y: '%f'", position_y_);
                RCLCPP_INFO(this->get_logger(), "position z: '%f'", position_z_);
                RCLCPP_INFO(this->get_logger(), "IMU frame is not correct");
                //rclcpp::shutdown();
            }
        }

        /*

        // output aberage angular velocity every 10seconds
        angular_velocity_x_.push_back(msg->angular_velocity.x);
        if ((current_time - start_time_).seconds() >= 10.0)
        {
            double sum = std:#include <deque>:accumulate(angular_velocity_x_.begin(), angular_velocity_x_.end(), 0.0);
            double average = angular_velocity_x_.empty() ? 0.0 : sum / angular_velocity_x_.size();
            start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Average angular velocity x: '%f'", average);
        }

        */
    }


    double ImuFrameChecker::applyLowPassFilter(std::deque<double>& buffer, double new_value) {
        buffer.push_back(new_value);
        if (buffer.size() > window_size_) {
            buffer.pop_front();
        }
        double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
        return sum / buffer.size();
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imu_frame_checker::ImuFrameChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}