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

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;
std::atomic_bool switchActive{true};

using namespace BT;


int main(int argc, char **argv) {

  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/oakd_chan2/config/first.xml";// xml指定？
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv);

 // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  //Node registration process
  factory.registerNodeType<MoveRobot>("MoveRobot");
  factory.registerNodeType<SearchForPaper>("SearchForPaper");
  factory.registerNodeType<ReadingLaser>("ReadingLaser");
  factory.registerNodeType<Rotating>("Rotating");
  factory.registerNodeType<Stop>("Stop");

  // we incorporated the BT (XML format)
  auto tree = factory.createTreeFromFile(xml_path);

  // PublisherZMQ(const BT::Tree& tree, unsigned max_msg_per_second = 25,
  //               unsigned publisher_port = 1666, unsigned server_port = 1667);
  unsigned publisher_port = 1666;
  unsigned server_port = 1667;
  BT::PublisherZMQ publisher_zmq(tree, 25, publisher_port, server_port);

  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  BT::NodeConfiguration con = {};

  //definiion of smart pointers to
  auto lc_listener = std::make_shared<ReadingLaser>("lc_listener", con);
  auto lc_odom = std::make_shared<Rotating>("lc_odom", con);
  auto lc_camera = std::make_shared<SearchForPaper>("lc_camera", con);
  auto lc_stop = std::make_shared<Stop>("lc_stop", con);

  // console log
  BT::StdCoutLogger logger_cout(tree);
  // for logging purposes. Details later
  FileLogger logger_file(tree, "src/oakd_chan2s/log/tb3_bts_trace.fbl");
  // we spin ROS nodes
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
