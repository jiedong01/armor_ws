#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        // 创建发布者，发布类型为 sensor_msgs/msg/Image，主题名称为 "image_raw"
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("color/image_raw", 10);

        // 打开相机
        cap_ = cv::VideoCapture(0); // 0 通常是默认的相机设备
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            return;
        }

        // 定时器设置，每30ms发布一次图像
        timer_ = this->create_wall_timer(
            30ms, std::bind(&CameraPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;  // 获取下一帧
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Camera frame empty, restarting...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置相机到开头
            return;
        }

        // 将 OpenCV 图像转换为 ROS 消息
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // 发布图像消息
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing frame... x: %d y: %d", frame.rows, frame.cols);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;  // 用于读取相机
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}