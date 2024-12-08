#ifndef __GS_SENSOR_HPP
#define __GS_SENSOR_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class gs_sensor : public rclcpp::Node {
   private:
    // 声明订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;  // 相机参数订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;             // RGB 图像订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;             // 深度数据订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr gsl_image_subscription_;         // 左传感器图像订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr gsr_image_subscription_;         // 右传感器图像订阅

    // 声明缓存对象
    std::unique_ptr<tf2_ros::Buffer> j1_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> j2_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> j3_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> j4_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> j5_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> j6_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> gsl_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> gsr_tf_buffer_;

    // 声明监听器对象
    std::shared_ptr<tf2_ros::TransformListener> j1_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> j2_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> j3_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> j4_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> j5_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> j6_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> gsl_tf_listenser_;
    std::shared_ptr<tf2_ros::TransformListener> gsr_tf_listenser_;

    // 声明参数
    // 相机的参数
    std::string camera_topic_;  // 相机状态的话题
    std::string image_topic_;   // 相机的图片的话题
    std::string depth_topic_;   // 相机深度的话题
    // 传感器的参数
    std::string gsl_image_topic_;  // 左传感器图片的话题
    std::string gsr_image_topic_;  // 右传感器图片的话题

    // 坐标系的参数
    std::string j1_tf_frame_;
    std::string j2_tf_frame_;
    std::string j3_tf_frame_;
    std::string j4_tf_frame_;
    std::string j5_tf_frame_;
    std::string j6_tf_frame_;
    std::string gsl_tf_frame_;
    std::string gsr_tf_frame_;

    // 存储文件的参数
    std::string save_directory_;            // 数据集存储总目录
    std::string save_image_directory_;      // 相机图像存储子目录
    std::string save_depth_directory_;      // 相机深度存储子目录
    std::string save_gsl_image_directory_;  // 左传感器图像存储子目录
    std::string save_gsr_image_directory_;  // 右传感器图像存储子目录
    std::string annotation_file_path_;      // 注释文件存储子目录

    // 其他变量
    bool recording_ = false;             // record标志位，用作记录动作逻辑
    nlohmann::json json_data_;           // 写入json文件的数据
    nlohmann::json camera_data_;         // 写入json文件的相机参数合集
    std::vector<double> k_;              // 相机内参参数
    int height_;                         // 图像高
    int width_;                          // 图像宽
    bool camera_info_sub_done_ = false;  //
    cv::Mat current_image_;              // 相机图像cv存储变量，通过cv_bridge传给ROS
    cv::Mat current_depth_;              // 相机深度cv存储变量
    cv::Mat current_gsl_image_;          // 传感器图像cv存储变量
    cv::Mat current_gsr_image_;          // 传感器图像cv存储变量

   public:
    // 构造函数
    gs_sensor();
    // 重写spin函数
    void spin();
    // 相机INFO回调函数
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    // 相机图像回调函数
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // // 相机深度回调函数
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 左传感器图像回调函数
    void gslimageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 右传感器图像回调函数
    void gsrimageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 按键检测函数
    bool ifKeyPressed();
    // 数据保存函数
    void saveData();
};

#endif