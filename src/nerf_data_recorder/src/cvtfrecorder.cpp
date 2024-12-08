#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <vector>
#include <cvtfrecorder/cvtfrecorder.hpp>
#include <thread>

CVTF::CVTFRecorder::CVTFRecorder(): Node("image_and_tf_recorder"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
{
    
    // 相机参数 、RGB 和 深度图的 topic 名称
    camera_topic_ = this->declare_parameter<std::string>("camera_topic", "/camera/camera/color/camera_info");
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_rect_raw");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    
    // 监控对象的 frame 名称
    tf_frame_ = this->declare_parameter<std::string>("tf_frame", "camera_color_optical_frame");
    
    // 数据集存储的目录
    save_directory_ = this->declare_parameter<std::string>("save_directory", "save");
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_topic_, 5,
        std::bind(&CVTFRecorder::cameraInfoCallback, this, std::placeholders::_1));

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 5,
        std::bind(&CVTFRecorder::imageCallback, this, std::placeholders::_1));
    
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, 5,
        std::bind(&CVTFRecorder::depthCallback, this, std::placeholders::_1));
    
    // 如果保存目录不存在，则创建
    if (!fs::exists(save_directory_)) 
    {
        fs::create_directory(save_directory_);
    }

    save_image_directory_ = save_directory_ + "/image";
    save_depth_directory_ = save_directory_ + "/depth";
    
    if (!fs::exists(save_image_directory_)) 
    {
        fs::create_directory(save_image_directory_);
    }
    if (!fs::exists(save_depth_directory_)) 
    {
        fs::create_directory(save_depth_directory_);
    }
    annotation_file_path_  = save_directory_ + "/annotation.json";
    if (fs::exists(annotation_file_path_))
    {
        std::ifstream file_in(annotation_file_path_);
        if (file_in.is_open()) {
            file_in >> json_data_;
            file_in.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open existing JSON file. ");
        }
    }
    else
    {
        json_data_ = json::object();
    }
    
    
    RCLCPP_INFO(this->get_logger(), "Image and TF Recorder Node initialized.");
}

void CVTF::CVTFRecorder::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_info_sub_done_){
        std::vector<double> k_values(msg->k.begin(), msg->k.end());
        k_=k_values;
        height_=msg->height;
        width_=msg->width;
        RCLCPP_INFO(this->get_logger(), "Received camera info!");
        camera_info_sub_done_ = true;
        if (!k_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Writting  camera info!");
        camera_data_ = {
            {"fx", k_[0]},
            {"fy", k_[4]},
            {"cx", k_[2]},
            {"cy", k_[5]},
            {"height", height_},
            {"width", width_}
        };
        RCLCPP_INFO(this->get_logger(), "Finish write K.");
        }
    }
}
  

void CVTF::CVTFRecorder::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try 
    {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
    }
}

void CVTF::CVTFRecorder::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    
    try 
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;
        depth_image.convertTo(current_depth_, CV_16UC1);
    }
    catch (cv_bridge::Exception &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert depth: %s", e.what());
    }
}

void CVTF::CVTFRecorder::saveData()
{
    if (current_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No image data available.");
        return;
    }

    if (current_depth_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No depth data available.");
        return;
    }

    try {
        // 获取 TF 数据
        auto tf_data = tf_buffer_.lookupTransform("world", tf_frame_, tf2::TimePointZero);

        // 格式化时间戳
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        
        std::string image_path = save_image_directory_ + "/image_" + std::to_string(timestamp) + ".png";
        std::string depth_path = save_depth_directory_ + "/depth_" + std::to_string(timestamp) + ".png";
        std::string tf_path = save_directory_  + std::to_string(timestamp) + ".json";

        // 保存 RGB h
        cv::imwrite(image_path, current_image_);
        cv::imwrite(depth_path, current_depth_);

        // 添加数据到 JSON 文件
        json data_ = 
        {   
            {"id","ID_"+std::to_string(timestamp)},
            {"image_path", "image/image_" + std::to_string(timestamp) + ".png"}, 
            {"depth_path", "depth/depth_" + std::to_string(timestamp) + ".png"},
            {"XYZ",{tf_data.transform.translation.x,tf_data.transform.translation.y,tf_data.transform.translation.z}},
            {"xyzw",{tf_data.transform.rotation.x,tf_data.transform.rotation.y,tf_data.transform.rotation.z,tf_data.transform.rotation.w}}
        };
        data_["camera_info"]=camera_data_;

        // 获取当前时间戳
        json_data_[std::to_string(timestamp)] = data_;

        std::ofstream file_out(annotation_file_path_);
        if (file_out.is_open()) {
            file_out << json_data_.dump(4);
            file_out.close();
        }

        RCLCPP_INFO(this->get_logger(), "Saved image to %s , depth to %s and TF to %s.", image_path.c_str(),depth_path.c_str(), annotation_file_path_.c_str());
    } 
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to fetch TF data: %s", ex.what());
    }
}


bool CVTF::CVTFRecorder::isKeyPressed()
{
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


void CVTF::CVTFRecorder::spin()
{   
    while (rclcpp::ok()) {
        if (isKeyPressed() && !recording_) {
            char c = getchar();
            if (c == 'r' || c == 'R') {
                recording_ = !recording_;
                RCLCPP_INFO(this->get_logger(), "Recording started.");
            }
        }

        // 如果正在录制，保存数据
        if (recording_) {
            saveData();
            recording_=false;
            RCLCPP_INFO(this->get_logger(), "Recording finished.");
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CVTF::CVTFRecorder>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
