# C++
## C++编译（cmake）
新建c++文件与cmake文件
`hello_world.cpp：`
```cpp
#include "iostream"

int main()
{
    std::cout << "hello world!" << std::endl;
    return 0;
}
```
`CMakeLists.txt`
```clike
# 指定CMake的最低版本要求
# VERSION 3.8 是构建此项目所需的最低CMake版本
# 如果系统中的CMake版本低于3.8，构建将失败
cmake_minimum_required(VERSION 3.8)

# 定义项目名称
# project() 命令设置项目名称为 "HelloWorld"
# 这个名称会被CMake用作默认的目标名称和其他配置
project(HelloWorld)

# 创建可执行文件目标
# add_executable() 命令创建一个名为 "hello_cmake" 的可执行文件
# 参数说明：
#   - hello_cmake: 生成的可执行文件名称
#   - hello_world.cpp: 源代码文件，包含main()函数
# 构建后会生成 hello_cmake.exe (Windows) 或 hello_cmake (Linux/Mac)
add_executable(hello_cmake hello_world.cpp)
```
### 生成Makefile文件
在`CMakeLists.txt`文件所在目录下运行`cmake .`
![生成Makefile文件](https://i-blog.csdnimg.cn/direct/c10aa8acb3b74b7eb0e5afabfc9f61d6.png)
### 构建C++可执行文件
`make`
![构建C++可执行文件](https://i-blog.csdnimg.cn/direct/47f8838e775e4db78f84958986d10e6c.png)
# C++节点
新建c++文件与cmake文件
`ros2_cpp_node.cpp：`
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(),"你好，C++节点");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
`CMakeLists.txt`
```clike
# 指定CMake的最低版本要求
# VERSION 3.8 是构建此项目所需的最低CMake版本
# 如果系统中的CMake版本低于3.8，构建将失败
cmake_minimum_required(VERSION 3.8)

# 定义项目名称
# project() 命令设置项目名称为 "ros2_cpp_node"
# 这个名称会被CMake用作默认的目标名称和其他配置
# 项目根目录: ${CMAKE_CURRENT_SOURCE_DIR}
# 构建目录: ${CMAKE_CURRENT_BINARY_DIR}
project(ros2_cpp_node)

# 创建可执行文件目标
# add_executable() 命令创建一个名为 "ros2_cpp_node" 的可执行文件
# 参数说明：
#   - ros2_cpp_node: 生成的可执行文件名称
#   - ros2_cpp_node.cpp: 源代码文件，包含main()函数
# 构建后会生成:
#   - Windows: ros2_cpp_node.exe
#   - Linux/Mac: ros2_cpp_node
# 输出路径: ${CMAKE_CURRENT_BINARY_DIR}/ros2_cpp_node
add_executable(ros2_cpp_node ros2_cpp_node.cpp)

# ============================================================================
# ROS2依赖包配置
# ============================================================================
# 查找ROS2 rclcpp包
# rclcpp包路径通常位于: /opt/ros/humble/share/rclcpp
# 包含文件路径: /opt/ros/humble/include/rclcpp
# 库文件路径: /opt/ros/humble/lib
find_package(rclcpp REQUIRED)

# 调试信息：显示rclcpp包含目录路径
# 输出示例: /opt/ros/humble/include/rclcpp
message(STATUS "rclcpp包含目录: ${rclcpp_INCLUDE_DIRS}")

# 调试信息：显示rclcpp库文件路径（可选）
# message(STATUS "rclcpp库文件: ${rclcpp_LIBRARIES}")

# ============================================================================
# 编译配置
# ============================================================================
# 设置包含目录
# 添加rclcpp头文件搜索路径到编译器
# 路径示例: /opt/ros/humble/include/rclcpp
target_include_directories(ros2_cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})

# 链接库文件
# 将rclcpp库文件链接到可执行文件
# 库文件路径示例: /opt/ros/humble/lib/librclcpp.so
target_link_libraries(ros2_cpp_node ${rclcpp_LIBRARIES})

# ============================================================================
# 构建输出路径说明
# ============================================================================
# 构建文件生成位置:
# - Makefile: ${CMAKE_CURRENT_BINARY_DIR}/Makefile
# - CMakeCache.txt: ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt
# - CMakeFiles/: ${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/
# - 可执行文件: ${CMAKE_CURRENT_BINARY_DIR}/ros2_cpp_node
# ============================================================================
```
## rclcpp头文件报错
在vscode中按`ctrl + shift + p`在`C/C++配置中`添加`/opt/ros/${ROS_DISTRO}/include/**`
![头文件报错](https://i-blog.csdnimg.cn/direct/5989d1dce837486aad7015c5231e3fea.png)
## 生成Makefile文件
在`CMakeLists.txt`文件所在目录下运行`cmake .`
![生成Makefile文件](https://i-blog.csdnimg.cn/direct/7011681404c24f77adec9a865913eca5.png)
## 构建C++可执行文件
`make`
![构建C++可执行文件](https://i-blog.csdnimg.cn/direct/a2fe9f5fac1d46a5bba7650f346c8e9c.png)
## 运行C++节点
![运行C++节点](https://i-blog.csdnimg.cn/direct/c5cd5226ad0f486ab3e02d6883512189.png)
## 查看节点
`ros2 node list`查看节点
`ros2 node info /cpp_node`查看节点详细信息
![查看节点](https://i-blog.csdnimg.cn/direct/381853b913f841e2ac5271fd44ff7b88.png)
# 功能包C++节点
***注：前面的CMakeLists.txt文件会被构建工具搜索到，所以要删除或者重命名***
## 创建C++功能包
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_package
```
创建后的文件目录
```bash
cpp_package/
├── include/                # 头文件目录
│   └── cpp_package/        # 包含功能包名的子目录
├── src/                    # 源代码目录
├── CMakeLists.txt          # CMake构建配置文件
├── LICENSE                 # Apache-2.0许可证文件
└── package.xml             # 功能包元数据文件
```
在`src`文件夹下新建文件（eg:`cpp_node.py`）并编写
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(),"你好，C++节点");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
在`CMakeLists.txt `文件中添加：
```clike
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/cpp_node.cpp)

ament_target_dependencies(cpp_node rclcpp)
#等同于下面两条
#target_include_directories(cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})
#target_link_libraries(cpp_node ${rclcpp_LIBRARIES})

install(TARGETS cpp_node DESTINATION lib/${PROJECT_NAME})
#这条命令在ament_package()前添加
```
## 生成Makefile文件
新建`build`文件夹，在`build`文件夹中生成Makefile文件
```bash
mkdir build && cd build && cmake ../
```
![生成Makefile文件](https://i-blog.csdnimg.cn/direct/4b2b1276b7594e268039204e281f2aa8.png)
## 构建C++可执行文件
`make`
![构建C++可执行文件](https://i-blog.csdnimg.cn/direct/02aeddd0e734414daa2bd0c64f923b4e.png)
## 构建功能包运行节点
***注：功能包要在同级或者上一级目录构建***
![构建功能包运行节点](https://i-blog.csdnimg.cn/direct/cc01b85c9e9f445da0bcd109b15109eb.png)
`ldd cpp_node`查看节点依赖库的链接
![查看节点依赖库的链接](https://i-blog.csdnimg.cn/direct/edb74393762f48d29878267db589451e.png)
`ros2 run`命令运行：![ros2运行](https://i-blog.csdnimg.cn/direct/6b6dd93a49544da88e2a618b5a0c1990.png)
# WorkSpace（工作空间）
## WorkSpace
WorkSpace（工作空间） 是 ROS2 中用于组织和管理多个功能包的顶层目录结构。它是一个包含源代码、构建文件和安装文件的完整开发环境。
### WorkSpace 的核心作用
- 1.项目组织 ：将相关的功能包组织在一起
- 2.依赖管理 ：统一管理包之间的依赖关系
- 3.构建管理 ：提供统一的构建和编译环境
- 4.环境隔离 ：不同项目使用独立的工作空间，避免冲突
### WorkSpace 标准目录结构
```bash
my_workspace/
├── src/                    # 源代码目录
│   ├── package1/          # 功能包1
│   ├── package2/          # 功能包2
│   └── package3/          # 功能包3
├── build/                 # 构建临时文件
├── install/               # 安装文件
│   ├── setup.bash        # 环境设置脚本
│   ├── local_setup.bash  # 本地环境脚本
│   └── share/            # 共享资源
└── log/                  # 构建日志
```
## 创建WorkSpace
新建`WorkSpace`及`src`文件夹：
```bash
mkdir -p workspace/src
```
移动功能包Python节点文件：
```bash
mv python_package/ workspace/src/
```
移动功能包C++节点文件：
```bash
mv cpp_package/ workspace/src/
```
删除多余文件：
```bash
rm -rf build/ install/ log/
```
![创建WorkSpace](https://i-blog.csdnimg.cn/direct/682131a2d8a443e79de3267b5741a8a4.png)
在`WorkSpace`目录下构建功能包
![workspace](https://i-blog.csdnimg.cn/direct/1b1c0b1832d246b8b82c8c4eb6649279.png)