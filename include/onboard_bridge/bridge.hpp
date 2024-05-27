#ifndef BRIDGE_HPP_
#define BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include <string.h>
#include <chrono>

class Bridge : public rclcpp::Node
{
    public:
        Bridge();

        // Subscribers callbacks
        void vehicle_command_callback(const px4_msgs::msg::VehicleCommand & msg);
        void offboard_control_mode_callback(const px4_msgs::msg::OffboardControlMode & msg);
        void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint & msg);

    private:
        std::string name_;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_sub_;
        rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_sub_;
        rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;

        // Forward Publishers
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
};

#endif