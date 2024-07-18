#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "lib/pid.h"

class PIDFeedbackLoop: public rclcpp::Node
{
public:
  PIDFeedbackLoop()
  : Node("pid_controller"), pid(0.15625, 0.03, 0.0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("pid_output", 2);
    auto topic_callback =
      [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
        double line_x_coord = msg->data;
        double offset = (frame_width/2) - line_x_coord; // error

        double output = pid.calculate(offset);

        auto message = std_msgs::msg::Float64();
        message.data = output;
        this->publisher_->publish(message);
      };
    subscription_ = this->create_subscription<std_msgs::msg::Float64>("line_location", 2, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  static const int frame_width = 640;
  PIDController pid;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDFeedbackLoop>());
  rclcpp::shutdown();
  return 0;
}