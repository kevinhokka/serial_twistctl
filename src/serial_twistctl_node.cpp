#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

using namespace std::chrono_literals;

class SerialTwistCtlNode : public rclcpp::Node
{
public:
    SerialTwistCtlNode()
    : Node("serial_twistctl_node")
    {
        // 获取当前时间并格式化为字符串
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time);
        
        // 格式化时间为文件名
        std::ostringstream filename_stream;
        filename_stream << "/home/jetson/ros2_ws/src/serial_twistctl/twist_record/log_" 
                        << (now_tm->tm_year + 1900)  // 年
                        << std::setw(2) << std::setfill('0') << (now_tm->tm_mon + 1) // 月
                        << std::setw(2) << std::setfill('0') << now_tm->tm_mday // 日
                        << "_" 
                        << std::setw(2) << std::setfill('0') << now_tm->tm_hour // 时
                        << std::setw(2) << std::setfill('0') << now_tm->tm_min // 分
                        << std::setw(2) << std::setfill('0') << now_tm->tm_sec // 秒
                        << ".txt"; // 完整的文件名
        
        // 打开日志文件，文件名基于当前时间
        log_file_.open(filename_stream.str(), std::ios::app);

        // 检查文件是否成功打开
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件");
            throw std::runtime_error("无法打开日志文件");
        }

        // 声明并获取参数（可以通过launch文件或命令行进行配置）
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("send_attempts", 1);
        this->declare_parameter<int>("delay_between_attempts_ms", 0);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        send_attempts_ = this->get_parameter("send_attempts").as_int();
        delay_between_attempts_ms_ = this->get_parameter("delay_between_attempts_ms").as_int();

        // 初始化串口
        try {
            serial_port_.setPort(port_);
            serial_port_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s: %s", port_.c_str(), e.what());
            throw std::runtime_error("串口初始化失败");
        }

        if (!serial_port_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "串口未成功打开！");
            throw std::runtime_error("串口未成功打开");
        } else {
            RCLCPP_INFO(this->get_logger(), "串口 %s 已成功打开.", port_.c_str());
        }

        // 设置订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "traj",
            10,
            std::bind(&SerialTwistCtlNode::twist_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "serial_twistctl_node 节点已启动，正在订阅话题...");
    }

    ~SerialTwistCtlNode()
    {
        if (serial_port_.isOpen()) {
            serial_port_.close();
            RCLCPP_INFO(this->get_logger(), "串口已关闭.");
        }

        // 关闭文件流
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;

        // 将Twist消息转换为串口命令
        char command[50];
        snprintf(command, sizeof(command), "vcx=%.3f,wc=%.3f\n", linear_x, angular_z);

        // 发送命令多次以确保接收
        for (int i = 0; i < send_attempts_; ++i) {
            serial_port_.write(command);

            // 记录到文件
            if (log_file_.is_open()) {
                log_file_ << "发送命令 (" << i+1 << "/" << send_attempts_ << "): " << command << std::endl;
            }

            RCLCPP_INFO(this->get_logger(), "发送命令 (%d/%d): %s", i+1, send_attempts_, command);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_between_attempts_ms_));
        }
    }

    serial::Serial serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::ofstream log_file_;  // 文件流

    // 参数
    std::string port_;
    int baudrate_;
    int send_attempts_;
    int delay_between_attempts_ms_;
};

int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    try {
        // 创建节点
        auto node = std::make_shared<SerialTwistCtlNode>();
        // 运行节点
        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        std::cerr << "节点启动失败: " << e.what() << std::endl;
    }

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
