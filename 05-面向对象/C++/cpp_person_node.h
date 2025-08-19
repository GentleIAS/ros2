#ifndef CPP_PERSON_NODE_H
#define CPP_PERSON_NODE_H

#include "rclcpp/rclcpp.hpp"
class PersonNode:public rclcpp::Node   //创建类PersonNode继承rclcpp的Node类
{
private:    //私有的
    std::string name_;
    int age_;

public:    //公有的
    //创建PersonNode的构造函数（与类名相同）并调用父级的构造函数
    PersonNode(const std::string &node_name,const std::string &name,const int &age):Node(node_name)
    {
        this->name_ = name;
        this->age_ = age;
    };

    void eat(const std::string &food_name)
    {
        //调用父级的日志打印，.c_str()是将string类型转换成c语言的char *类型
        RCLCPP_INFO(this->get_logger(),"%s，%d岁，爱吃：%s",this->name_.c_str(),this->age_,food_name.c_str());
    };

    std::string get_name()
    {
        return this->name_;
    };

    int get_age()
    {
        return this->age_;
    };
};

#endif