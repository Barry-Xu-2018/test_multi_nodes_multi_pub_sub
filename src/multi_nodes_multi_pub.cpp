#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

class NodePub : public rclcpp::Node
{
public:
  explicit NodePub(const std::string & node_name, std::string topic_name, uint32_t pub_num)
  :rclcpp::Node(node_name)
  {
    for (uint32_t i = 1; i <= pub_num; i++) {
      auto pub = create_publisher<std_msgs::msg::String>(topic_name, 10);
      pubs_.emplace_back(pub);
    }

    RCLCPP_INFO(this->get_logger(), "Create %u publishers --- Done !", pub_num);
  }

private:
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

void usage(std::string prog_name){
  std::cout << "Usage: " << prog_name << " -t Topic_Num -p Pub_Num_In_One_Node" << std::endl;
}


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

  uint32_t pub_num = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-p");
  if (nullptr != cli_option) {
    pub_num = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  rclcpp::executors::SingleThreadedExecutor exe;
  std::vector<std::shared_ptr<NodePub>> nodes;
  // One topic for subscribers in one node
  for (uint32_t i = 1; i <= topic_num; i++) {
    std::string node_name = "node_pub_" + std::to_string(i);
    std::string topic_name = "topic_" + std::to_string(i);
    auto node = std::make_shared<NodePub>(node_name, topic_name, pub_num);
    nodes.emplace_back(node);
    exe.add_node(node);
  }
  exe.spin();

  nodes.clear();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
