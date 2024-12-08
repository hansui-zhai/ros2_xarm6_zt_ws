#include <cstdio>
/**
 * @file gs_sensor.cpp
 * @brief
 * @author hansui (hansui_dsb@outlook.com)
 * @date 2024-12-07
 *
 * @copyright Copyright (C) 2024, HANSUI_VISION, all rights reserved.
 *
 * @par 修改日志:
 * 获取数据，保存成dataset
 * 1、监听坐标:
 *    1）两个传感器的坐标
 *    2）六个关节的坐标
 * 2、订阅图像数据
 *    1）两个传感器的图像
 *    2）相机的图像
 *    3）相机的深度
 *
 */
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <termios.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <fstream>
#include <gs_sensor/gs_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

// 构造函数
gs_sensor::gs_sensor() : rclcpp::Node("gs_data_recorder") {
    // 参数
    // 相机参数
    camera_topic_ = this->declare_parameter<std::string>("camera_topic", "/camera/camera/color/camera_info");
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_rect_raw");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    // 传感器参数
    gsl_image_topic_ = this->declare_parameter<std::string>("gsl_image_topic", "/camera/camera/color/camera_info1");
    gsr_image_topic_ = this->declare_parameter<std::string>("gsr_image_topic", "/camera/camera/color/camera_info2");
    // 坐标系参数
    j1_tf_frame_ = this->declare_parameter<std::string>("j1_tf_frame", "/camera/camera/color/camera_info3");
    j2_tf_frame_ = this->declare_parameter<std::string>("j2_tf_frame", "/camera/camera/color/camera_info4");
    j3_tf_frame_ = this->declare_parameter<std::string>("j3_tf_frame", "/camera/camera/color/camera_info5");
    j4_tf_frame_ = this->declare_parameter<std::string>("j4_tf_frame", "/camera/camera/color/camera_info6");
    j5_tf_frame_ = this->declare_parameter<std::string>("j5_tf_frame", "/camera/camera/color/camera_info7");
    j6_tf_frame_ = this->declare_parameter<std::string>("j6_tf_frame", "/camera/camera/color/camera_info8");
    gsl_tf_frame_ = this->declare_parameter<std::string>("gsl_tf_frame", "/camera/camera/color/camera_info9");
    gsr_tf_frame_ = this->declare_parameter<std::string>("gsr_tf_frame", "/camera/camera/color/camera_info10");
    // 数据存储参数
    save_directory_ = this->declare_parameter<std::string>("save_directory", "save");
    save_image_directory_ = save_directory_ + "/image";
    save_depth_directory_ = save_directory_ + "/depth";
    annotation_file_path_ = save_directory_ + "/annotation.json";
    save_gsl_image_directory_ = save_directory_ + "/gsl_image";
    save_gsr_image_directory_ = save_directory_ + "/gsr_image";

    // 创建缓存对象
    j1_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    j2_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    j3_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    j4_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    j5_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    j6_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    gsl_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    gsr_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // 创建监听者
    j1_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j1_tf_buffer_, this);
    j2_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j2_tf_buffer_, this);
    j3_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j3_tf_buffer_, this);
    j4_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j4_tf_buffer_, this);
    j5_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j5_tf_buffer_, this);
    j5_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*j5_tf_buffer_, this);
    gsl_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*gsl_tf_buffer_, this);
    gsr_tf_listenser_ = std::make_shared<tf2_ros::TransformListener>(*gsr_tf_buffer_, this);

    // 创建订阅者
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic_, 5, std::bind(&gs_sensor::cameraInfoCallback, this, std::placeholders::_1));
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic_, 5, std::bind(&gs_sensor::imageCallback, this, std::placeholders::_1));
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(depth_topic_, 5, std::bind(&gs_sensor::depthCallback, this, std::placeholders::_1));
    gsl_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(gsl_image_topic_, 5, std::bind(&gs_sensor::imageCallback, this, std::placeholders::_1));
    gsr_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(gsr_image_topic_, 5, std::bind(&gs_sensor::imageCallback, this, std::placeholders::_1));

    // 创建保存文件的目录
    if (!std::filesystem::exists(save_directory_)) {
        std::filesystem::create_directory(save_directory_);
    }
    if (!std::filesystem::exists(save_image_directory_)) {
        std::filesystem::create_directory(save_image_directory_);
    }
    if (!std::filesystem::exists(save_depth_directory_)) {
        std::filesystem::create_directory(save_depth_directory_);
    }
    if (!std::filesystem::exists(save_gsl_image_directory_)) {
        std::filesystem::create_directory(save_gsl_image_directory_);
    }
    if (!std::filesystem::exists(save_gsr_image_directory_)) {
        std::filesystem::create_directory(save_gsr_image_directory_);
    }
    if (std::filesystem::exists(annotation_file_path_)) {
        std::ifstream file_in(annotation_file_path_);
        if (file_in.is_open()) {
            file_in >> json_data_;
            file_in.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open existing JSON file. ");
        }
    } else {
        json_data_ = nlohmann::json::object();
    }

    // 日志
    RCLCPP_INFO(this->get_logger(), "创建gs_data_recorder成功!");
}
// 重写spin函数
void gs_sensor::spin() {
    while (rclcpp::ok()) {
        if (ifKeyPressed() && !recording_) {
            char c = getchar();
            if (c == 'r' || c == 'R') {
                recording_ = !recording_;
                RCLCPP_INFO(this->get_logger(), "Recording started.");
            }
        }

        // 如果正在录制，保存数据
        if (recording_) {
            saveData();
            recording_ = false;
            RCLCPP_INFO(this->get_logger(), "Recording finished.");
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }
}
// 相机INFO回调函数（原则上只用一次？）
void gs_sensor::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_sub_done_) {
        std::vector<double> k_values(msg->k.begin(), msg->k.end());
        k_ = k_values;
        height_ = msg->height;
        width_ = msg->width;
        RCLCPP_INFO(this->get_logger(), "Received camera info!");
        camera_info_sub_done_ = true;
        if (!k_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Writting  camera info!");
            camera_data_ = {{"fx", k_[0]}, {"fy", k_[4]}, {"cx", k_[2]}, {"cy", k_[5]}, {"height", height_}, {"width", width_}};
            RCLCPP_INFO(this->get_logger(), "Finish write K.");
        }
    }
}
// 相机图像回调函数
void gs_sensor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
    }
}
// 相机深度回调函数
void gs_sensor::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;
        depth_image.convertTo(current_depth_, CV_16UC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert depth: %s", e.what());
    }
}
// 左传感器图像回调函数
void gs_sensor::gslimageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        current_gsl_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert gsl_image: %s", e.what());
    }
}
// 右传感器图像回调函数
void gs_sensor::gsrimageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        current_gsr_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert gsr_image: %s", e.what());
    }
}
// 按键检测函数
bool gs_sensor::ifKeyPressed() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}
// 数据保存函数
void gs_sensor::saveData() {
    // 数据存在检测
    if (current_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No image data available.");
        return;
    }
    if (current_depth_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No depth data available.");
        return;
    }
    if (current_gsl_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No gsl_image data available.");
        return;
    }
    if (current_gsr_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No gsr_image data available.");
        return;
    }
    try {
        // 获取 TF 数据
        auto j1_tf_data = j1_tf_buffer_->lookupTransform("world", j1_tf_frame_, tf2::TimePointZero);
        auto j2_tf_data = j2_tf_buffer_->lookupTransform("world", j2_tf_frame_, tf2::TimePointZero);
        auto j3_tf_data = j3_tf_buffer_->lookupTransform("world", j3_tf_frame_, tf2::TimePointZero);
        auto j4_tf_data = j4_tf_buffer_->lookupTransform("world", j4_tf_frame_, tf2::TimePointZero);
        auto j5_tf_data = j5_tf_buffer_->lookupTransform("world", j5_tf_frame_, tf2::TimePointZero);
        auto j6_tf_data = j6_tf_buffer_->lookupTransform("world", j6_tf_frame_, tf2::TimePointZero);
        auto gsl_tf_data = gsl_tf_buffer_->lookupTransform("world", gsl_tf_frame_, tf2::TimePointZero);
        auto gsr_tf_data = gsr_tf_buffer_->lookupTransform("world", gsr_tf_frame_, tf2::TimePointZero);

        // 格式化时间戳，作为名字标识？
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

        // 存储路径
        std::string image_path = save_image_directory_ + "/image_" + std::to_string(timestamp) + ".png";
        std::string depth_path = save_depth_directory_ + "/depth_" + std::to_string(timestamp) + ".png";
        std::string gsl_image_path = save_image_directory_ + "/gsl_image_" + std::to_string(timestamp) + ".png";
        std::string gsr_image_path = save_image_directory_ + "/gsr_image_" + std::to_string(timestamp) + ".png";
        std::string tf_path = save_directory_ + std::to_string(timestamp) + ".json";

        // 保存 RGB 图像
        cv::imwrite(image_path, current_image_);
        cv::imwrite(depth_path, current_depth_);
        cv::imwrite(gsl_image_path, current_gsl_image_);
        cv::imwrite(gsl_image_path, current_gsr_image_);

        // 添加数据到 JSON 文件
        nlohmann::json data_ = {
            {"id", "ID_" + std::to_string(timestamp)},
            {"image_path", "image/image_" + std::to_string(timestamp) + ".png"},
            {"depth_path", "depth/depth_" + std::to_string(timestamp) + ".png"},
            {"gsl_image_path", "gsl_image/gsl_image_" + std::to_string(timestamp) + ".png"},
            {"gsr_image_path", "gsr_image/gsr_image_" + std::to_string(timestamp) + ".png"},
            {"j1_XYZ", {j1_tf_data.transform.translation.x, j1_tf_data.transform.translation.y, j1_tf_data.transform.translation.z}},
            {"j1_xyzw", {j1_tf_data.transform.rotation.x, j1_tf_data.transform.rotation.y, j1_tf_data.transform.rotation.z, j1_tf_data.transform.rotation.w}},
            {"j2_XYZ", {j2_tf_data.transform.translation.x, j2_tf_data.transform.translation.y, j2_tf_data.transform.translation.z}},
            {"j2_xyzw", {j2_tf_data.transform.rotation.x, j2_tf_data.transform.rotation.y, j2_tf_data.transform.rotation.z, j2_tf_data.transform.rotation.w}},
            {"j3_XYZ", {j3_tf_data.transform.translation.x, j3_tf_data.transform.translation.y, j3_tf_data.transform.translation.z}},
            {"j3_xyzw", {j3_tf_data.transform.rotation.x, j3_tf_data.transform.rotation.y, j3_tf_data.transform.rotation.z, j3_tf_data.transform.rotation.w}},
            {"j4_XYZ", {j4_tf_data.transform.translation.x, j4_tf_data.transform.translation.y, j4_tf_data.transform.translation.z}},
            {"j4_xyzw", {j4_tf_data.transform.rotation.x, j4_tf_data.transform.rotation.y, j4_tf_data.transform.rotation.z, j4_tf_data.transform.rotation.w}},
            {"j5_XYZ", {j5_tf_data.transform.translation.x, j5_tf_data.transform.translation.y, j5_tf_data.transform.translation.z}},
            {"j5_xyzw", {j5_tf_data.transform.rotation.x, j5_tf_data.transform.rotation.y, j5_tf_data.transform.rotation.z, j5_tf_data.transform.rotation.w}},
            {"j6_XYZ", {j6_tf_data.transform.translation.x, j6_tf_data.transform.translation.y, j6_tf_data.transform.translation.z}},
            {"j6_xyzw", {j6_tf_data.transform.rotation.x, j6_tf_data.transform.rotation.y, j6_tf_data.transform.rotation.z, j6_tf_data.transform.rotation.w}},
            {"gsl_XYZ", {gsl_tf_data.transform.translation.x, gsl_tf_data.transform.translation.y, gsl_tf_data.transform.translation.z}},
            {"gsl_xyzw", {gsl_tf_data.transform.rotation.x, gsl_tf_data.transform.rotation.y, gsl_tf_data.transform.rotation.z, gsl_tf_data.transform.rotation.w}},
            {"gsr_XYZ", {gsr_tf_data.transform.translation.x, gsr_tf_data.transform.translation.y, gsr_tf_data.transform.translation.z}},
            {"gsr_xyzw", {gsr_tf_data.transform.rotation.x, gsr_tf_data.transform.rotation.y, gsr_tf_data.transform.rotation.z, gsr_tf_data.transform.rotation.w}},
        };
        data_["camera_info"] = camera_data_;

        // 获取当前时间戳
        json_data_[std::to_string(timestamp)] = data_;

        // 写入json文件
        std::ofstream file_out(annotation_file_path_);
        if (file_out.is_open()) {
            file_out << json_data_.dump(4);
            file_out.close();
        }

        // 日志
        RCLCPP_INFO(this->get_logger(), "Saved image to %s , depth to %s and TF to %s.", image_path.c_str(), depth_path.c_str(), annotation_file_path_.c_str());

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to fetch TF data: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gs_sensor>();
    node->spin();
    rclcpp::shutdown();
}
