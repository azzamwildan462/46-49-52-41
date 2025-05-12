#include "master/master.hpp"

Master::Master()
    : Node("master")
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Master::callback_routine, this));

    pub_ui_test = this->create_publisher<std_msgs::msg::String>("ui_test", 10);

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_routine()
{
    process_transmitter();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}