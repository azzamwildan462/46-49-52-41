#include "master/master.hpp"

void Master::process_transmitter()
{
    static int counter = 0;
    std_msgs::msg::String msg;
    msg.data = "Hello World " + std::to_string(counter++);
    pub_ui_test->publish(msg);
}