#include <chrono>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/custom.hpp"

// Gunakan struct untuk memastikan urutan data dan ukuran 16 byte
struct ControlFrame {
    // Kita hanya akan mengisi 6 nilai int16_t pertama (12 byte)
    // dan membiarkan 4 byte sisanya kosong/nol, sehingga total 16 byte
    int16_t thrust_dr; // rxBuffer[0], rxBuffer[1]
    int16_t thrust_dl; // rxBuffer[2], rxBuffer[3] <-- Kita akan fokus di sini (pakai 'y')
    int16_t thrust_tr; // rxBuffer[4], rxBuffer[5]
    int16_t thrust_tl; // rxBuffer[6], rxBuffer[7]
    int16_t thrust_br; // rxBuffer[8], rxBuffer[9]
    int16_t thrust_bl; // rxBuffer[10], rxBuffer[11]
    int16_t unused_1;  // rxBuffer[12], rxBuffer[13]
    int16_t unused_2;  // rxBuffer[14], rxBuffer[15]
} __attribute__((packed)); // Penting: memastikan compiler tidak menambah padding

using std::placeholders::_1;

class SercomNode : public rclcpp::Node
{
public:
    // ... (Konstruktor sama, inisialisasi serial port)
    SercomNode()
    : Node("sercom_node"),
      io_(),
      serial_(io_)
    {
        std::string port = "/dev/ttyUSB0"; // Default port

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
        ControlFrame data_to_send = {0}; // Inisialisasi semua ke nol

        // Fokus: Gunakan nilai Y (msg->y) untuk menggerakkan Thruster Depan Kiri (thrust_dl)
        // Thruster Depan Kiri menggunakan rxBuffer[2] dan rxBuffer[3] di STM32
        data_to_send.thrust_dl = static_cast<int16_t>(msg->y); 
        
        // Thruster lainnya diatur ke 1500 (nilai netral setelah dikonversi di STM32)
        // Anda bisa set ke 0 di sini, karena 0 di ROS -> 1500 di STM32
        data_to_send.thrust_dr = 0; 
        data_to_send.thrust_tr = 0; 
        data_to_send.thrust_tl = 0; 
        data_to_send.thrust_br = 0; 
        data_to_send.thrust_bl = 0; 
        
        // Kirim 16 byte biner
        size_t sent_bytes = boost::asio::write(serial_, 
            boost::asio::buffer(&data_to_send, sizeof(ControlFrame)));

        // Output log untuk debugging
        RCLCPP_INFO(this->get_logger(), 
            "Sent %zu bytes. Y: %d -> DL: %d", 
            sent_bytes, msg->y, data_to_send.thrust_dl);
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