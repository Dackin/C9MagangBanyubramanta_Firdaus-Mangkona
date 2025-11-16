#include <chrono>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/custom.hpp"

using std::placeholders::_1;

class SercomNode : public rclcpp::Node
{
public:
    SercomNode()
    : Node("sercom_node"),
      io_(),
      serial_(io_)
    {
        std::string port = "/tmp/ttyV1";

        try {
            serial_.open(port);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
        }
        catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s", e.what());
        }

        subs = this->create_subscription<interfaces::msg::Custom>(
            "/cmd_vel",
            10,
            std::bind(&SercomNode::callbackCmd, this, _1)
        );
    }

private:
    void callbackCmd(const interfaces::msg::Custom::SharedPtr msg)
    {
        std::string frame =
            std::to_string(msg->x) + "," +
            std::to_string(msg->y) + "," +
            std::to_string(msg->depth) + "," +
            std::to_string(msg->yaw) + "," +
            msg->pesan + "\n";

        boost::asio::write(serial_, boost::asio::buffer(frame));
        RCLCPP_INFO(this->get_logger(), "Sent: %s", frame.c_str());
    }

    // ASIO
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;

    // Subscription
    rclcpp::Subscription<interfaces::msg::Custom>::SharedPtr subs;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SercomNode>());
    rclcpp::shutdown();
    return 0;
}