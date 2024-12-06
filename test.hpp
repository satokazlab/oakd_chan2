//------------------------------------------------------------------------------------
//----------------------------------- Include ----------------------------------------
//------------------------------------------------------------------------------------
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>  // std::system
#include <boost/process.hpp>  // Boost process library

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;

using namespace BT;

//-------------------------------------------------------------------------------------
//--------------------------Class MOVE_ROBOT-------------------------------------------
//-------------------------------------------------------------------------------------
class MoveRobot : public BT::SyncActionNode, public rclcpp::Node {
  private://オドメトリ情報を処理するコールバック関数
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr _msg) {

      tf2::Quaternion quat_tf;
      geometry_msgs::msg::Quaternion quat_msg = _msg->pose.pose.orientation;
      tf2::fromMsg(quat_msg, quat_tf);
      double roll{}, pitch{}, yaw{};
      tf2::Matrix3x3 m(quat_tf);
      m.getRPY(roll, pitch, yaw);

      // Left ＋,  Right −
      float yaw_check = yaw * 180 / M_PI;

      float robotAngle = yaw_check;
      RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
      RCLCPP_INFO(this->get_logger(), "position: '%f' '%f'",
                  _msg->pose.pose.position.x, _msg->pose.pose.position.y);
    }

    rclcpp::TimerBase::SharedPtr timer_; // タイマーを使って定期的に処理を実行する
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_; // 指定したトピックからメッセージを購読してコールバックを呼び出す
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;// 指定したトピックにメッセージを送信する

  public: // BTのアクションノードMoveRobotの定義のためのクラス
    MoveRobot(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), Node("move_node") {

      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS()); // QoSの設定
      // /odomを受け取ったらコールバックを実行
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", sensor_qos, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            odometry_callback(msg);
          });
    }
    
    // ノードがアクティブになったら実行
    NodeStatus tick() override {
      //  ポートからの入力取得
      auto res_time = getInput<int>("sleep_mtime");
      if (!res_time) {
        throw RuntimeError("error reading port [sleep_mtime]:", res_time.error());
      }
      int sleep_mtime = res_time.value();
      // パブリッシャー生成
      publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      auto message = geometry_msgs::msg::Twist();

      message.linear.x = 0.2;
      setOutput("linear_x", float(message.linear.x)); // ポートに出力。他のノードから参照可能に
      
      // Pythonノードを実行する
      runPythonNode();

      // 任意の条件を設定（例：ロボットの位置や角度が特定の範囲内かどうか）
        bool condition_met = check_condition();

        // 条件に基づいてSUCCESSまたはFAILUREを返す
        if (condition_met) {
            message.linear.x = 0.2;
            setOutput("linear_x", float(message.linear.x));
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_mtime));
            // Pythonノードを停止
            stop_python_node();

            return NodeStatus::SUCCESS; // 条件達成時はSUCCESSを返す
        } else {
            return NodeStatus::FAILURE; // 条件未達成時はFAILUREを返す
        }
    }

    // 任意の条件チェック関数 T or F
    bool check_condition() {
        // 条件の例：ロボットが特定の位置にいるか、角度が一定範囲にあるか
        // ここでは仮にロボットの位置がx = 1.0, y = 2.0に近いかどうかを確認する
        // 実際の条件に合わせて変更してください。
        bool condition_met = false;
        float x = 1.0;  // 仮の位置
        float y = 2.0;  
        // ここで仮の位置をチェックする例
        if (std::abs(x - 1.0) < 0.1 && std::abs(y - 2.0) < 0.1) {
            condition_met = true;
        }
        return condition_met;
    }

    // Pythonノードを停止する関数
    void stop_python_node() {
        if (python_pid != -1) {
            // PythonノードのプロセスIDが存在する場合、それを終了させる
            std::string kill_command = "kill " + std::to_string(python_pid);
            std::system(kill_command.c_str());
            python_pid = -1;  // プロセスIDをリセット
            RCLCPP_INFO(this->get_logger(), "Python node stopped.");
        }
    }

    // Pythonノードを起動する関数
    void start_python_node() {
        // Pythonノードをバックグラウンドで起動する
        std::string command = "ros2 run your_python_package your_python_node &";
        std::system(command.c_str());

        // PythonノードのプロセスIDを取得
        python_pid = std::getpid();
    }

    // ノードが提供するポートをリストとして返す
    static PortsList providedPorts() {
      const char *description = "Simply print the target on console...";
      return {
        InputPort<int>("sleep_mtime", description),
        OutputPort<float>("linear_x", description)
      };
    }
};