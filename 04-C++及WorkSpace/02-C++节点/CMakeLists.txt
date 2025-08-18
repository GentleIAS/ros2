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