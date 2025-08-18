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