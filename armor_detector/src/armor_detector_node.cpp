#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tasks/detector.hpp"
#include "tools/img_tools.hpp"
#include <sstream>

// 相机内参
    static const cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << 1286.307063384126, 0, 645.34450819155256,
         0, 1288.1400736562441, 483.6163720308021,
         0, 0, 1);
    // 畸变系数
    static const cv::Mat distort_coeffs =
        (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);

    static const float LIGHTBAR_LENGTH = 0.056; // 灯条长度    单位：米
    static const float ARMOR_WIDTH = 0.135;     // 装甲板宽度  单位：米

    // object_points 是 物体局部坐标系下 n个点 的坐标。
    static const std::vector<cv::Point3f> object_points{
        {-ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0}, // 点 1
        {ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0},  // 点 2
        {ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0},   // 点 3
        {-ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0},  // 点 4
    };

    // 假设的 extractEulerAngles 函数，你需要实现它
 std::vector<double> extractEulerAngles(const cv::Mat &rmat)
{
    // 从旋转矩阵提取欧拉角的逻辑
    cv::Mat eulerAngles;
    cv::Rodrigues(rmat, eulerAngles);

    // 将欧拉角转换为度
    double yaw = eulerAngles.at<double>(0) * 180 / CV_PI;
    double pitch = eulerAngles.at<double>(1) * 180 / CV_PI;
    double roll = eulerAngles.at<double>(2) * 180 / CV_PI;

    return {yaw, pitch, roll};
}

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode() : Node("armor_detector_node")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "color/image_raw", 10, std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

        // // 创建图像发布者
        // image_pub_ = it.advertise("camera/image_processed", 10);

        RCLCPP_INFO(this->get_logger(), "armor_detector_node, Starting...");
    }

private:
    
    // 图像回调函数
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (inHandle)
            return;

        inHandle = true;
        RCLCPP_INFO(this->get_logger(), "armor_detector_node image Callback");

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        if (img.empty()) // 读取失败 或 视频结尾
            return;

        auto armors = detector.detect(img);

        if (!armors.empty())
        {
            auto_aim::Armor &armor = armors.front(); // 如果识别到了大于等于一个装甲板，则取出第一个装甲板来处理

            std::vector<cv::Point2f> img_points{
                armor.left.top,
                armor.right.top,
                armor.right.bottom,
                armor.left.bottom,
            };

            tools::draw_points(img, img_points);

            cv::Mat rvec, tvec;
            cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);

            std::stringstream ss;
            ss << "tvec:  x:" << tvec.at<double>(0) << " y:"  << tvec.at<double>(1) << " z:" << tvec.at<double>(2);
            tools::draw_text(img, ss.str(), cv::Point2f(10, 60), 0.7, cv::Scalar(0, 255, 255), 0.3);
            
            std::stringstream sss;
            sss << "rvec:  x:" << rvec.at<double>(0) << " y:" << rvec.at<double>(1) << " z:" << rvec.at<double>(2);
            tools::draw_text(img, sss.str(), cv::Point2f(10, 120), 0.7, cv::Scalar(0, 255, 255), 0.3);

            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);

            std::vector<double> euler_angles = extractEulerAngles(rmat);

            std::stringstream sssss;
            sssss << "euler angles:  yaw:" << euler_angles[0] << " pitch:" << euler_angles[1] << " roll:" << euler_angles[2];
            tools::draw_text(img, sssss.str(), cv::Point2f(10, 180), 0.7, cv::Scalar(0, 255, 255), 0.3);
        }

        cv::namedWindow("result", cv::WINDOW_NORMAL); // 创建一个可调整大小的窗口
        cv::resizeWindow("result", 200, 100); 
        cv::imshow("result", img);
        cv::waitKey(1);  // 允许 OpenCV 窗口更新

        inHandle = false;
    }

    bool inHandle;
    auto_aim::Detector detector;
    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
