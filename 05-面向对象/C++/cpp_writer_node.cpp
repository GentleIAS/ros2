#include "rclcpp/rclcpp.hpp"
#include "cpp_person_node.h"

class WriterNode : public PersonNode // 创建类PersonNode继承rclcpp的Node类
{
private: // 私有的
    std::string write_;

public: // 公有的
    // 创建PersonNode的构造函数（与类名相同）并调用父级的构造函数
    WriterNode(const std::string &node_name, const std::string &name,const int &age,const std::string &write = "没有作品")
                : PersonNode(node_name, name.empty()?"匿名作者":name, age==0?18:age)
    {
        this->write_ = write;
    };

    void eat(const std::string &food_name)
    {
        // 调用父级的日志打印，.c_str()是将string类型转换成c语言的char *类型
        RCLCPP_INFO(this->get_logger(), "%s，%d岁，爱吃：%s，书籍：%s",this->get_name().c_str(), this->get_age(),
                    food_name.c_str(), this->write_.c_str());
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<WriterNode>("cpp_writer_node1","村民李四",35);
    auto node2 = std::make_shared<WriterNode>("cpp_writer_node2","",0,"在ros2速成C++");
    node1->eat("香蕉");
    node2->eat("苹果");
    rclcpp::spin(node1);
    rclcpp::spin(node2);
    rclcpp::shutdown();
    return 0;
};