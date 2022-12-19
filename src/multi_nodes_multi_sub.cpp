#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/subscription_base.hpp>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

uint32_t target_number = 0;

class NodeSub : public rclcpp::Node
{
public:
  explicit NodeSub(
    const std::string & node_name,
    const std::string & topic_name,
    uint32_t sub_num)
  :rclcpp::Node(node_name)
  {
    auto dummy = [](std_msgs::msg::String::ConstSharedPtr msg) {(void) msg;};

    RCLCPP_INFO(this->get_logger(), "Connect to topic %s", topic_name.c_str());
    for (uint32_t i = 1; i <= sub_num; i++) {
      auto sub = create_subscription<std_msgs::msg::String>(topic_name, 10, dummy);
      std::lock_guard<std::mutex> lock(subs_mutex_);
      subs_.emplace_back(sub);
    }

    RCLCPP_INFO(this->get_logger(), "Create %u subscriptions --- Done !", sub_num);

  }

  const std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> & get_subs() {
    return subs_;
  }

  uint32_t get_subs_num() {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    return subs_.size();
  }

private:
  std::mutex subs_mutex_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
};

void usage(std::string prog_name){
  std::cout << "Usage: " << prog_name << " -t Topic_Num -s Sub_Num -n1s1" << std::endl;
  std::cout << "  -t How many topic to be connected." << std::endl;
  std::cout << "  -s How many subscriptions connected to one topic." << std::endl;
  std::cout << "  -n1s1 1 subscription in 1 node. If no this option, Sub_Num subscription is in one Node." << std::endl;
}

class NodeCheck : public rclcpp::Node
{
public:
  NodeCheck(std::vector<std::shared_ptr<NodeSub>> & nodes, uint32_t sub_num)
  : rclcpp::Node("NodeCheck")
  {
    auto callback = [this, & nodes, sub_num](){
      for (auto & node: nodes) {
        if (node->get_subs_num() != sub_num) {
          return;
        }
      }

      for (auto & node: nodes) {
        for (auto & sub: node->get_subs()) {
          if (sub->get_publisher_count() !=1) {
            RCLCPP_INFO(this->get_logger(), "%s isn't ready", node->get_name());
            return;
          }
        }
      }

      this->timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "+++ All subscriptions connect publishers ! +++");
    };

    timer_ = create_wall_timer(std::chrono::milliseconds(500), callback);
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  char * cli_option = nullptr;
  uint32_t topic_num = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic_num = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  uint32_t sub_num = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    sub_num = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  bool one_node_one_sub = rcutils_cli_option_exist(argv, argv + argc, "-n1s1");

  rclcpp::executors::SingleThreadedExecutor exe;
  std::vector<std::shared_ptr<NodeSub>> nodes;
  uint32_t sub_num_in_node = 0;

  // Set target
  target_number = topic_num * sub_num;
  if (one_node_one_sub) {
    uint32_t node_num = topic_num * sub_num;
    for (uint32_t i = 1; i <= node_num; i++) {
      std::string topic_name = "topic_"+ std::to_string(int((i-1)/sub_num) + 1);
      std::string node_name = "node_sub_" + std::to_string(i) + "_" + topic_name;
      auto node = std::make_shared<NodeSub>(node_name, topic_name, 1);
      nodes.emplace_back(node);
      exe.add_node(node);
      sub_num_in_node = 1;
    }
  } else {
    uint32_t node_num = topic_num;
    for (uint32_t i = 1; i <= node_num; i++) {
      std::string topic_name = "topic_" + std::to_string(i);
      std::string node_name = "node_sub_" + std::to_string(i) + "_" + topic_name;
      auto node = std::make_shared<NodeSub>(node_name, topic_name, sub_num);
      nodes.emplace_back(node);
      exe.add_node(node);
      sub_num_in_node = sub_num;
    }
  }

  auto node_check = std::make_shared<NodeCheck>(nodes, sub_num_in_node);
  exe.add_node(node_check);

  exe.spin();
  nodes.clear();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
