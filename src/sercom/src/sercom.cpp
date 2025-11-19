#include <chrono>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/custom.hpp"

// Struct ControlFrame sesuai dengan STM32 (Total 16 Byte)
struct ControlFrame {
    int16_t thrust_dr; // Down Right (Vertical)
    int16_t thrust_dl; // Down Left (Vertical)
    int16_t thrust_tr; // Top Right (Horizontal - Front Right?)
    int16_t thrust_tl; // Top Left (Horizontal - Front Left?)
    int16_t thrust_br; // Bottom Right (Horizontal - Back Right?)
    int16_t thrust_bl; // Bottom Left (Horizontal - Back Left?)
    int16_t unused_1;  
    int16_t unused_2;  
} __attribute__((packed));

using std::placeholders::_1;

class SercomNode : public rclcpp::Node
{
public:
    SercomNode()
    : Node("sercom_node"),
      io_(),
      serial_(io_)
    {
        std::string port = "/dev/ttyUSB0"; // Pastikan port sesuai

        try {
            serial_.open(port);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
        }
        catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s", e.what());
        }

        // Subscribe ke cmd_vel
        subs = this->create_subscription<interfaces::msg::Custom>(
            "/cmd_vel",
            10,
            std::bind(&SercomNode::callbackCmd, this, _1)
        );
    }

private:
    void callbackCmd(const interfaces::msg::Custom::SharedPtr msg)
    {
        ControlFrame data_to_send = {0};

        // 1. Ambil data dari Controller (interfaces/msg/Custom)
        int16_t input_x = static_cast<int16_t>(msg->x);     // Surge (Maju/Mundur)
        int16_t input_y = static_cast<int16_t>(msg->y);     // Sway (Kanan/Kiri)
        int16_t input_yaw = static_cast<int16_t>(msg->yaw); // Yaw (Putar)
        
        // Catatan: Di controller.cpp, depth range-nya 0-10. 
        // Jika STM32 butuh nilai PWM besar, kalikan di sini. 
        // Misal dikali 50 agar range menjadi 0-500.
        int16_t input_depth = static_cast<int16_t>(msg->depth * 50); 

        // 2. MIXING ALGORITHM (Kinematics)
        // Logika ini menggabungkan X, Y, Yaw ke 4 motor horizontal
        // Dan Depth ke 2 motor vertikal.
        // Tanda (+/-) harus disesuaikan dengan arah putaran baling-baling robot Anda.

        // --- Horizontal Thrusters (Asumsi konfigurasi X-Config / Vectored) ---
        // Front Left (TL)  = Maju + Geser Kanan + Putar Kanan
        data_to_send.thrust_tl = input_x + input_y + input_yaw; 
        
        // Front Right (TR) = Maju - Geser Kanan - Putar Kanan
        data_to_send.thrust_tr = input_x - input_y - input_yaw;

        // Back Left (BL)   = Maju - Geser Kanan + Putar Kanan
        data_to_send.thrust_bl = input_x - input_y + input_yaw;

        // Back Right (BR)  = Maju + Geser Kanan - Putar Kanan
        data_to_send.thrust_br = input_x + input_y - input_yaw;

        // --- Vertical Thrusters (Depth) ---
        // Menggerakkan robot ke atas/bawah
        data_to_send.thrust_dl = input_depth; // Down Left
        data_to_send.thrust_dr = input_depth; // Down Right

        // 3. Kirim Data ke STM32 via Serial
        size_t sent_bytes = boost::asio::write(serial_, 
            boost::asio::buffer(&data_to_send, sizeof(ControlFrame)));

        // 4. Logging untuk Debugging
        RCLCPP_INFO(this->get_logger(), 
            "In[X:%d Y:%d Z:%d Yaw:%d] -> Out[TL:%d TR:%d BL:%d BR:%d DL:%d]", 
            input_x, input_y, msg->depth, input_yaw,
            data_to_send.thrust_tl, data_to_send.thrust_tr,
            data_to_send.thrust_bl, data_to_send.thrust_br, data_to_send.thrust_dl);
    }

    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
    rclcpp::Subscription<interfaces::msg::Custom>::SharedPtr subs;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SercomNode>());
    rclcpp::shutdown();
    return 0;
}