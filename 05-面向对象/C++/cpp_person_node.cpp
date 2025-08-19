#include "rclcpp/rclcpp.hpp"
#include "cpp_person_node.h"

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PersonNode>("cpp_person_node","法外狂徒张三",18);
    node->eat("鱼香肉丝");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};