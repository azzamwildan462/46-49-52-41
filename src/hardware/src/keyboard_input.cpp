#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>

class KeyboardNode : public rclcpp::Node
{
public:
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_key_pressed;

    KeyboardNode() : Node("keyboard_node")
    {
        pub_key_pressed = this->create_publisher<std_msgs::msg::Int16>("/key_pressed", 1);
        configureTerminal(); // Configure terminal for non-canonical mode
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // Check every 20 ms
            std::bind(&KeyboardNode::timerCallback, this));
    }

    ~KeyboardNode()
    {
        resetTerminal(); // Restore terminal to original settings
    }

private:
    void timerCallback()
    {
        getKeyPress();
    }

    void getKeyPress()
    {
        char key_buf;
        int bytesRead = read(STDIN_FILENO, &key_buf, 1);
        if (bytesRead > 0)
        {
            std_msgs::msg::Int16 msg;
            msg.data = key_buf;
            pub_key_pressed->publish(msg); // Publish the key pressed
            return;                        // Return the key pressed
        }
        return; // No key pressed
    }

    void configureTerminal()
    {
        // Save current terminal settings
        tcgetattr(STDIN_FILENO, &original_termios_);

        // Configure terminal for non-canonical, non-blocking mode
        termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        // Set stdin to non-blocking mode
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void resetTerminal()
    {
        // Restore original terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);

        // Reset stdin to blocking mode
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) & ~O_NONBLOCK);
    }

    termios original_termios_; // Store original terminal settings
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
