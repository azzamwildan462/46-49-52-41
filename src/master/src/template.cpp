#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

class Template : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;

    Template()
        : Node("template")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Template::callback_routine, this));

        logger.info("Template node initialized");
    }

    ~Template()
    {
    }

    void callback_routine()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_template = std::make_shared<Template>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_template);
    executor.spin();

    return 0;
}