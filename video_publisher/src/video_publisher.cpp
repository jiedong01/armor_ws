#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher()
        : Node("video_publisher")
    {
        // 创建发布者，发布类型为 sensor_msgs/msg/Image，主题名称为 "image_raw"
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // 打开视频文件
        cap_ = cv::VideoCapture("resource/video.avi"); // 替换为你的测试视频路径
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video file");
            return;
        }

        // 定时器设置，每30ms发布一次图像
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&VideoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;  // 获取下一帧
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video ended, restarting...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置视频文件到开头
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
    cv::VideoCapture cap_;  // 用于读取视频
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
