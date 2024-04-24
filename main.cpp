#include "planner.hpp"
#include <thread>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor ex;
    auto node = std::make_shared<Planner>();
    node->test();
    /*ex.add_node(node);
    auto jth = std::jthread([&](){ node->plan(); ex.cancel(); });
    ex.spin();
    rclcpp::shutdown();
    */
}