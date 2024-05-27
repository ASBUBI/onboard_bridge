#include "onboard_bridge/bridge.hpp"

Bridge::Bridge() : Node("bridge")
{
    this->declare_parameter<std::string>("drone_name", "");
    this->get_parameter<std::string>("drone_name", name_);

    // Subscribers
    this->vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/"+name_+"/fmu/in/vehicle_command", rclcpp::SensorDataQoS(),
        std::bind(&Bridge::vehicle_command_callback, this, std::placeholders::_1)
    );

    this->offboard_control_mode_sub_ = this->create_subscription<px4_msgs::msg::OffboardControlMode>(
        "/"+name_+"/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS(),
        std::bind(&Bridge::offboard_control_mode_callback, this, std::placeholders::_1)
    );

    this->trajectory_setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/"+name_+"/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS(),
        std::bind(&Bridge::trajectory_setpoint_callback, this, std::placeholders::_1)
    );

    // Publishers
    this->vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", rclcpp::SensorDataQoS());
    this->offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
    this->trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
}

/**
 * CALLBACKS
*/

void Bridge::vehicle_command_callback(const px4_msgs::msg::VehicleCommand & msg)
{
    vehicle_command_pub_->publish(msg);
}

void Bridge::offboard_control_mode_callback(const px4_msgs::msg::OffboardControlMode & msg)
{
    offboard_control_mode_pub_->publish(msg);
}

void Bridge::trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint & msg)
{
    trajectory_setpoint_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}