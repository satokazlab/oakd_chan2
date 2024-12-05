//------------------------------------------------------------------------------------
//----------------------------------- Include ----------------------------------------メインで実行するノード！！
//------------------------------------------------------------------------------------
#include <filesystem>

#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

//------------------------------------------------------------------------------------
//---------------------------- Include Thrid Party -----------------------------------
//------------------------------------------------------------------------------------
#include "oakd_chan2/read_laser_condition.hpp"
#include "oakd_chan2/detect_camera_action.hpp"
#include "oakd_chan2/move_robot_action.hpp"
#include "oakd_chan2/rotating_robot_action.hpp"
#include "oakd_chan2/stop_robot_action.hpp"

using namespace std::chrono_literals;// 時間のリテラルが使える
using std::chrono::milliseconds;
using std::placeholders::_1;//  関数の引数プレースホルダー
std::atomic_bool switchActive{true};// スレッドセーフなブール型変数

using namespace BT;// BTのライブラリのクラスや関数を簡単に利用できる


int main(int argc, char **argv) {

  // first.xml の絶対パスを生成
  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/oakd_chan2/config/first.xml";// xmlの指定
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv);// ROS2アプリケーションを初期化

 // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  //Node registration process
  factory.registerNodeType<MoveRobot>("MoveRobot");
  factory.registerNodeType<SearchForPaper>("SearchForPaper");
  factory.registerNodeType<ReadingLaser>("ReadingLaser");
  factory.registerNodeType<Rotating>("Rotating");
  factory.registerNodeType<Stop>("Stop");

  // XMLファイルからビヘイビアツリー（Behavior Tree）を構築し、それを操作するためのオブジェクトを作成
  auto tree = factory.createTreeFromFile(xml_path);

  // PublisherZMQ(const BT::Tree& tree, unsigned max_msg_per_second = 25,
  //               unsigned publisher_port = 1666, unsigned server_port = 1667);
  unsigned publisher_port = 1666;
  unsigned server_port = 1667;
  // ビヘイビアツリーの状態を外部へ配信するPublisherZMQのインスタンス化
  BT::PublisherZMQ publisher_zmq(tree, 25, publisher_port, server_port);
  // BTの各ノードの現在の状態を表す status を初期化
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  // ノードの入出力データを設定するために使われる構造体 の初期化
  BT::NodeConfiguration con = {};

  //特定の動作やロボット制御のタスクのクラスをインスタンス化
  auto lc_listener = std::make_shared<ReadingLaser>("lc_listener", con);
  auto lc_odom = std::make_shared<Rotating>("lc_odom", con);
  auto lc_camera = std::make_shared<SearchForPaper>("lc_camera", con);
  auto lc_stop = std::make_shared<Stop>("lc_stop", con);

  // treeの実行情報をログ表示
  BT::StdCoutLogger logger_cout(tree);
  // ログファイルに保存
  FileLogger logger_file(tree, "src/oakd_chan2s/log/tb3_bts_trace.fbl");
  // we spin ROS nodes
  // rootノードが失敗する限り、くり返しrootノードを実行
  while (rclcpp::ok() && status == BT::NodeStatus::FAILURE) {
    rclcpp::spin_some(lc_odom);
    rclcpp::spin_some(lc_listener);
    rclcpp::spin_some(lc_camera);
    rclcpp::spin_some(lc_stop);
    //we check the status of node
    status = tree.tickRoot();

    // Groot 4.X
    // status = tree.tickOnce();
    // status = tree.tickWhileRunning();

    tree.sleep(std::chrono::milliseconds(200));
  }

  return 0;
}
