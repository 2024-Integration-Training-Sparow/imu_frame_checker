#ifndef IMU_FRAME_CHECKER_HPP_
#define IMU_BIAS_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>

namespace imu_frame_checker
{
    class ImuFrameChecker : public rclcpp::Node
    {
        public:
            ImuFrameChecker();
        
        private:
            using ImuMsg = sensor_msgs::msg::Imu;
            rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
            void imu_callback(const ImuMsg::SharedPtr msg);
            void imu_bias_correction();
            double applyLowPassFilter(std::deque<double>& buffer, double new_value);

            std::vector<double> angular_velocity_x_;
            rclcpp::Time start_time_;
            rclcpp::Time last_time_;
            double velocity_x_;
            double velocity_y_;
            double velocity_z_;
            double position_x_;
            double position_y_;
            double position_z_;
            double acceleration_bias_x_;
            double acceleration_bias_y_;
            double acceleration_bias_z_;
            size_t sample_count_;
            size_t window_size_;
            std::deque<double> accel_x_buffer_, accel_y_buffer_, accel_z_buffer_;
    };
}

#endif