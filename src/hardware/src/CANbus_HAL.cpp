#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "std_msgs/msg/int16.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#define MAX_CAN_DEVICES 2
#define CAN_COBID_RX 0x580
#define CAN_COBID_TX 0x600
#define CAN_COBID_TX_DEFAULT 0x200

class CANbusHAL : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_wheel_encoder;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_delta_encoder_wheel_counter;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_feedback_steering;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_wheel;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_target_steering;

    HelpLogger logger;

    // Configs
    // =======================================================
    std::string if_name;
    int bitrate = 125000;
    int routine_period_ms = 20; // ms
    int max_can_recv_error_counter = 10;
    int can_timeout_us = 100000; // us
    int id_driver_wheel = 0x04;
    int id_driver_steering = 0x08;
    int max_target_steering = 70;

    int socket_can = -1;

    int16_t target_pwm_wheel = 0;
    int16_t target_steering = 0; // Derajat target = target_steering * 0.01
    int16_t encoder_wheel_counter = 0;
    int16_t delta_encoder_wheel_counter = 0;
    int16_t feedback_steering = 0; // Derajat feedback = feedback_steering * 0.01

    // Data kalkulasi
    uint8_t counter_ditemukan = 0;
    uint16_t counter_can_recv_error = 0;

    CANbusHAL()
        : Node("CANusb_HAL")
    {
        //----Parameters
        this->declare_parameter<std::string>("if_name", "can0");
        this->get_parameter("if_name", if_name);

        this->declare_parameter<int>("bitrate", 125000);
        this->get_parameter("bitrate", bitrate);

        this->declare_parameter<int>("routine_period_ms", 20);
        this->get_parameter("routine_period_ms", routine_period_ms);

        this->declare_parameter<int>("max_can_recv_error_counter", 10);
        this->get_parameter("max_can_recv_error_counter", max_can_recv_error_counter);

        this->declare_parameter<int>("can_timeout_us", 100000);
        this->get_parameter("can_timeout_us", can_timeout_us);

        this->declare_parameter<int>("id_driver_wheel", 0x04);
        this->get_parameter("id_driver_wheel", id_driver_wheel);

        this->declare_parameter<int>("id_driver_steering", 0x08);
        this->get_parameter("id_driver_steering", id_driver_steering);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Publihers
        pub_wheel_encoder = this->create_publisher<std_msgs::msg::Int16>("wheel_encoder", 1);
        pub_feedback_steering = this->create_publisher<std_msgs::msg::Int16>("feedback_steering", 1);
        pub_delta_encoder_wheel_counter = this->create_publisher<std_msgs::msg::Int16>("delta_encoder_wheel_counter", 1);

        //----Subscribers
        sub_pwm_wheel = this->create_subscription<std_msgs::msg::Int16>(
            "pwm_wheel", 1, std::bind(&CANbusHAL::callback_pwm_wheel, this, std::placeholders::_1));
        sub_target_steering = this->create_subscription<std_msgs::msg::Int16>(
            "target_steering", 1, std::bind(&CANbusHAL::callback_target_steering, this, std::placeholders::_1));

        //----Init
        char cmd[100];
        sprintf(cmd, "sudo ip link set %s up type can bitrate %d", if_name.c_str(), bitrate);
        system(cmd);

        socket_can = can_init(if_name.c_str());
        if (socket_can < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
            rclcpp::shutdown();
        }

        //----Timers
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(routine_period_ms), std::bind(&CANbusHAL::callback_routine, this));

        logger.info("CANbusHAL node initialized");
    }

    // =============================================================================================================================

    void callback_pwm_wheel(const std_msgs::msg::Int16::SharedPtr msg)
    {
        target_pwm_wheel = msg->data;
    }

    void callback_target_steering(const std_msgs::msg::Int16::SharedPtr msg)
    {
        target_steering = msg->data;

        if (target_steering > max_target_steering)
            target_steering = max_target_steering;
        else if (target_steering < -max_target_steering)
            target_steering = -max_target_steering;
    }

    ~CANbusHAL()
    {
    }

    void callback_routine()
    {
        send_sync();
        poll_can();
        send_can();
        transmit_all();
    }

    // =============================================================================================================================

    void send_can()
    {
        struct can_frame frame;
        frame.can_id = CAN_COBID_TX_DEFAULT;
        frame.can_dlc = 8;

        memset(frame.data, 0, sizeof(frame.data));

        frame.data[0] = id_driver_wheel | id_driver_steering;

        int16_t pwm_abs = abs(target_pwm_wheel);
        uint8_t pwm_wheel_enable = 1;
        if (pwm_abs < 10)
            pwm_wheel_enable = 0;

        frame.data[1] = pwm_abs;
        frame.data[2] = (pwm_abs >> 8) & 0xFF;
        frame.data[2] |= (target_pwm_wheel >> 8) & 0b1000000;
        frame.data[2] |= (pwm_wheel_enable << 7);

        int16_t steering_abs = abs(target_steering);
        uint8_t steering_enable = 1;

        frame.data[3] = steering_abs;
        frame.data[4] = (steering_abs >> 8) & 0xFF;
        frame.data[4] |= (target_steering >> 8) & 0b1000000;
        frame.data[4] |= (steering_enable << 7);

        if (write(socket_can, &frame, sizeof(frame)) != sizeof(frame))
        {
            logger.error("Failed to send CAN frame");
        }
    }

    void poll_can()
    {
        counter_ditemukan = 0;
        while (counter_ditemukan < MAX_CAN_DEVICES && counter_can_recv_error < max_can_recv_error_counter)
        {
            // Read the CAN frame
            struct can_frame frame;
            fd_set read_fds;
            struct timeval timeout;
            int retval;

            // Set up the file descriptor set
            FD_ZERO(&read_fds);
            FD_SET(socket_can, &read_fds);

            // Set timeout values
            timeout.tv_sec = 0;
            timeout.tv_usec = can_timeout_us;

            // Wait for data to be available on the set
            retval = select(socket_can + 1, &read_fds, NULL, NULL, &timeout);

            if (retval == -1)
            {
                logger.error("Select error on CAN socket");
                counter_can_recv_error++;
                continue;
            }
            else if (retval == 0)
            {
                logger.warn("CAN socket read timeout");
                counter_can_recv_error++;
                continue;
            }

            // Data is available, read it
            parse_can_frame(&frame);
        }
    }

    void parse_can_frame(struct can_frame *frame)
    {
        if (frame->can_id == (CAN_COBID_RX + (uint8_t)id_driver_wheel))
        {
            static int16_t prev_encoder_wheel_counter = 0;

            encoder_wheel_counter = (frame->data[0] << 8) | frame->data[1];
            delta_encoder_wheel_counter = encoder_wheel_counter - prev_encoder_wheel_counter;
            prev_encoder_wheel_counter = encoder_wheel_counter;

            counter_ditemukan++;
        }
        else if (frame->can_id == (CAN_COBID_RX + (uint8_t)id_driver_steering))
        {
            feedback_steering = (frame->data[0] << 8) | frame->data[1];
            counter_ditemukan++;
        }
    }

    void send_sync()
    {
        struct can_frame frame;
        frame.can_id = 0x80;
        frame.can_dlc = 0;

        if (write(socket_can, &frame, sizeof(frame)) != sizeof(frame))
        {
            logger.error("Failed to send CAN frame");
        }
    }

    void transmit_all()
    {
        std_msgs::msg::Int16 msg_wheel_encoder;
        msg_wheel_encoder.data = encoder_wheel_counter;
        pub_wheel_encoder->publish(msg_wheel_encoder);

        std_msgs::msg::Int16 msg_delta_encoder_wheel_counter;
        msg_delta_encoder_wheel_counter.data = delta_encoder_wheel_counter;
        pub_delta_encoder_wheel_counter->publish(msg_delta_encoder_wheel_counter);

        std_msgs::msg::Int16 msg_feedback_steering;
        msg_feedback_steering.data = feedback_steering;
        pub_feedback_steering->publish(msg_feedback_steering);
    }

    int8_t can_init(const char *interface)
    {
        int8_t s;
        struct sockaddr_can addr;
        struct ifreq ifr;

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("Error while opening socket");
            return -1;
        }

        strcpy(ifr.ifr_name, interface);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Error in socket bind");
            return -1;
        }

        return s;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_CANusb_HAL = std::make_shared<CANbusHAL>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_CANusb_HAL);
    executor.spin();

    return 0;
}