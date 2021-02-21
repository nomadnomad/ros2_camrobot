#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../include/move_instr_subscriber/move_instr_subscriber.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  /* 実行コンテキストであるExecutorの生成、この場合はSingleスレッドのExecutor */
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<MoveInstrSubscriber> node = 
    std::make_shared<MoveInstrSubscriber>();

  RCLCPP_INFO(node->get_logger(), "インスタンス生成");

  /* ノードのインスタンスを executorに登録 */
  exe.add_node(node->get_node_base_interface());
  RCLCPP_INFO(node->get_logger(), "ノードのインスタンスをexecutorに登録");
  /******* ここまでがノードの初期化です　*****/

  /* executorの実行ループを開始します。開始後は、Unconfigured状態です　*/
  RCLCPP_INFO(node->get_logger(), "executorの実行ループを開始する");
  exe.spin(); 
  //rclcpp::spin(std::make_shared<TestImageSubscriber>());
  RCLCPP_INFO(node->get_logger(), "shutdown()呼び出し前");
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "shutdown()呼び出し後");
  return 0;
}