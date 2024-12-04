#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/process.hpp>  // Boost.Processライブラリをインクルード

using namespace std::chrono_literals;

class CameraDetected : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Pythonノードの実行
        execute_python_node();
    }

    void execute_python_node() {
        // Pythonスクリプトのパスを指定して実行
        std::string python_script = "/path/to/your/python_node.py";  // Pythonスクリプトのパス
        try {
            boost::process::child c("python3", python_script);
            c.wait();  // Pythonスクリプトが終了するまで待機
            RCLCPP_INFO(this->get_logger(), "Python script executed successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Python script: %s", e.what());
        }
    }

public:
    CameraDetected() : Node("camera_node") {
        // Subscriptionの作成
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&CameraDetected::imageCallback, this, std::placeholders::_1)
        );

        // Publisherの作成
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDetected>());
    rclcpp::shutdown();
    return 0;
}
