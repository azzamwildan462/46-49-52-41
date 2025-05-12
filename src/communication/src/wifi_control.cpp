#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/system_utils.hpp"

class WiFIControl : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;

    //----
    // Static Configs
    std::string hotspot_ssid = "Hotspot";
    std::string hotspot_password = "12345678";
    std::string wifi_ssid = "*";
    std::string user = "invalid";

    bool is_wifi_connected = false;
    bool is_hotspot_on = false;

    WiFIControl()
        : Node("WiFi_Control")
    {
        user = getenv("SUDO_USER");

        this->declare_parameter("hotspot_ssid", hotspot_ssid);
        this->get_parameter("hotspot_ssid", hotspot_ssid);

        this->declare_parameter("hotspot_password", hotspot_password);
        this->get_parameter("hotspot_password", hotspot_password);

        this->declare_parameter("wifi_ssid", wifi_ssid);
        this->get_parameter("wifi_ssid", wifi_ssid);

        this->declare_parameter("user", user);
        this->get_parameter("user", user);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (user == "invalid")
        {
            logger.error("Invalid user!");
            rclcpp::shutdown();
        }

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20000), std::bind(&WiFIControl::callback_routine, this));

        logger.info("WiFIControl %s node initialized", user.c_str());
    }

    ~WiFIControl()
    {
    }

    void callback_routine()
    {
        std::string wifi_connected_network = execute_command("iwgetid -r");

        is_wifi_connected = false;
        if (wifi_ssid != "*")
        {
            if (wifi_connected_network == wifi_ssid)
            {
                is_wifi_connected = true;
            }
        }
        else if (wifi_connected_network != "")
        {
            is_wifi_connected = true;
        }

        if (!is_wifi_connected)
        {
            logger.info("WiFi is not connected, scanning for target network");

            std::string available_wifi_networks = execute_command("nmcli -t -f SSID dev wifi list | sort | uniq", ";");
            std::string connected_wifi_ssid = execute_command("nmcli connection show | grep wifi | awk -F  '  +' '{print $1}'", ";");
            std::vector<std::string> available_wifi_ssid_list = str_explode(available_wifi_networks, ';');
            std::vector<std::string> connected_wifi_ssid_list = str_explode(connected_wifi_ssid, ';');

            is_hotspot_on = false;

            // Check if available_wifi_ssid_list == available_wifi_ssid_list
            for (std::string &available_wifi_ssid : available_wifi_ssid_list)
            {
                if (available_wifi_ssid == hotspot_ssid)
                {
                    is_hotspot_on = true;
                    continue;
                }

                /**
                 * Ketika tersedia WiFi yang hidup dan Credentialnya telah terdaftar
                 * Dipastikan bahwa WiFi berhasil konek
                 */
                if (std::find(connected_wifi_ssid_list.begin(), connected_wifi_ssid_list.end(), available_wifi_ssid) != connected_wifi_ssid_list.end())
                {
                    logger.info("Connecting to %s", available_wifi_ssid.c_str());
                    execute_command("sudo -u " + user + " nmcli connection up \"" + available_wifi_ssid + "\"");
                    is_wifi_connected = true;
                    break;
                }
            }

            /* Ketika hotspot mati dan belum konek wifi */
            if (!is_hotspot_on && !is_wifi_connected)
            {
                logger.info("Turning on Hotspot");
                execute_command("nmcli device wifi hotspot ssid " + hotspot_ssid + " password " + hotspot_password);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_WiFi_Control = std::make_shared<WiFIControl>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_WiFi_Control);
    executor.spin();

    return 0;
}