#ifndef CVTFRECORDER_HPP_
#define CVTFRECORDER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace CVTF {

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace std::chrono_literals;

class CVTFRecorder : public rclcpp::Node {
   public:
    // 创建一个构造函数
    CVTFRecorder();

    // 创建一个spin函数,用于循环监控
    void spin();

   private:
    // 建立一个图像回调函数
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 建立一个深度回调函数
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 建立一个camera info 回调函数
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    //  创建一个数据保存函数
    void saveData();

    //  检测键盘是否有按键按下
    bool isKeyPressed();

    //  指定成员变量
    std::vector<double> k_;
    int height_;
    int width_;
    bool camera_info_sub_done_ = false;
    json camera_data_;

    std::string camera_topic_;
    std::string image_topic_;
    std::string depth_topic_;
    std::string tf_frame_;
    std::string save_directory_;
    std::string save_image_directory_;
    std::string save_depth_directory_;
    std::string annotation_file_path_;
    json json_data_;
    cv::Mat current_image_;
    cv::Mat current_depth_;
    bool recording_ = false;

    // ROS 2 组件
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;  // 相机参数订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;             // RGB 图像订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;             // 深度图像订阅
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace CVTF
#endif  // CVTFRECORDER_HPP_