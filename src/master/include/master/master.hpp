#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_utils/help_logger.hpp"

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ui_test;

    HelpLogger logger;

    Master();
    ~Master();

    void callback_routine();

    void process_transmitter();
};

#endif // MASTER_HPP