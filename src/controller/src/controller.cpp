#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/custom.hpp"

using namespace std::chrono_literals;

const int maxDepth = 10;
const int minDepth = 0;
const int maxYaw = 180;
const int minYaw = -180;

int x = 0; int y = 0; unsigned int depth = 0; int yaw;

const char* pesan;

class ControllNode : public rclcpp::Node{

public:
  ControllNode() : Node("controllNode"){
    publisher_= this->create_publisher<interfaces::msg::Custom>("custom_msg", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ControllNode::controllerCallback, this, std::placeholders::_1));
  }

private:

    rclcpp::Publisher<interfaces::msg::Custom>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    int nilaiSekarang(int masuk, int max, int min, int sekarang){
      sekarang += masuk;
      if(sekarang > max){sekarang = max;}
      if(sekarang < min){sekarang = min;}

      return sekarang;
    }

    const char* nilaiPesan(int masuk){
      if(masuk == 1){return "Jelajah Jaladhi Jaghana Jaya!";}
      else{return "";}
    }

    void controllerCallback(const sensor_msgs::msg::Joy &msg){

      x = (-1 * msg.axes[0] * 250);
      y = (msg.axes[1] * 250);
      depth = nilaiSekarang(-1 * msg.axes[4], maxDepth, minDepth, depth);
      yaw = nilaiSekarang(-1 * msg.axes[3], maxYaw, minYaw, yaw);
      pesan = nilaiPesan(msg.buttons[0]);

      auto custom_msg = interfaces::msg::Custom();
      custom_msg.x = x;
      custom_msg.y = y;
      custom_msg.depth = depth;
      custom_msg.yaw = yaw;
      custom_msg.pesan = pesan;

      publisher_->publish(custom_msg);
    }
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllNode>());
  rclcpp::shutdown();
  return 0;
}