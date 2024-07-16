#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CamPublisher : public rclcpp::Node
{
public:
  CamPublisher()
  : Node("video_publisher"), cap(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("videofeed", 2);

    if (cap.isOpened()) {
      auto timer_callback =
      [this]() -> void {
        cap >> frame;

        cropTopOfFrame(frame);

        if (frame.empty())
          RCLCPP_WARN(this->get_logger(), "Frame is empty");

        // Convert frame to sensor_msgs::msg::Image 
        std_msgs::msg::Header header; 
        header.stamp = this->now(); 
        cv_bridge::CvImage cv_image(header, "bgr8", frame);

        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
        publisher_->publish(*msg);
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    }
  }

  // Keeps the bottom 20% of the frame's height
  void cropTopOfFrame(cv::Mat &frame) {
    int height = frame.rows;
    int width = frame.cols;

    int height_cutoff = static_cast<int>(height * 0.8);

    cv::Rect roi(0, height_cutoff, width, height-height_cutoff);
    frame = frame(roi);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  cv::VideoCapture cap;
  cv::Mat frame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamPublisher>());
  rclcpp::shutdown();
  return 0;
}
