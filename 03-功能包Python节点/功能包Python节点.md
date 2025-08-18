# Python功能包
## 功能包
ROS2功能包（Package）是ROS2系统中的 基本组织单元 ，类似于其他编程语言中的库或模块。它将相关的代码、配置文件、资源文件等组织在一起，形成一个可重用、可分发的软件单元。

### 功能包的核心作用
- 1.代码组织 ：将相关功能的代码组织在一起
- 2.依赖管理 ：明确声明所需的依赖关系
- 3.构建管理 ：定义如何编译和安装代码
- 4.资源管理 ：管理配置文件、启动文件、参数文件等
- 5.分发共享 ：便于在不同项目间共享和重用
## 创建Python功能包

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 python_package
```
`ros2 pkg create`构建功能包命令
`--build-type ament_python`指定构建类型（`ament_python`Python类型）
`--license Apache-2.0`指定协议
`python_package`包文件名
![Python功能包](https://i-blog.csdnimg.cn/direct/4d1d17268478486d92e080cf2304c706.png)
创建后的文件目录
```bash
python_package/
├── python_package/          # Python模块目录
│   └── __init__.py
├── resource/                # 资源文件
│   └── python_package
└── test/                    # 测试文件
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── LICENSE                  # 许可证文件
├── package.xml              # 功能包描述文件
├── setup.cfg                # 安装配置
├── setup.py                 # Python安装配置
```
在`python_package`文件夹下新建文件（eg:`python_node.py`）并编写

```python
# 导入ROS2的Python客户端库
# rclpy是ROS2的Python客户端库，提供了与ROS2系统交互的基本功能
import rclpy
# 从rclpy.node模块导入Node类，这是创建ROS2节点的基础类
from rclpy.node import Node

def main():
    # 初始化ROS2的Python客户端库
    # 这是使用ROS2功能前必须调用的函数，用于设置ROS2的运行环境
    rclpy.init()
    
    # 创建一个名为"python_node"的ROS2节点
    # 节点是ROS2中的基本计算单元，可以发布消息、订阅话题、提供服务等
    node = Node("python_node")
    
    # 使用节点的日志记录器输出信息
    # get_logger()返回节点的日志记录器，info()用于输出信息级别的日志
    node.get_logger().info("你好 Python 节点!")
    
    try:
        # 启动节点的事件循环
        # spin()函数会让节点保持运行状态，处理回调函数、消息等
        # 这是一个阻塞调用，会一直运行直到节点被关闭
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断异常（Ctrl+C）
        # 当用户按下Ctrl+C时，程序会优雅地退出
        # warn()用于输出错误级别的日志
        node.get_logger().warn("用户中断了 spin（Ctrl+C）")

    # 销毁节点，释放相关资源
    # 这是清理工作的一部分，确保节点正确关闭
    node.destroy_node()
    
    # 关闭ROS2的Python客户端库
    # 这是程序结束前的清理工作，释放ROS2相关的系统资源
    rclpy.shutdown()
```
在`setup.py`中的

```python
entry_points={
        'console_scripts': [
            
        ],
    },
```
添加`'ros2_python_node = python_package.python_node:main'`

```python
entry_points={
        'console_scripts': [
            'ros2_python_node = python_package.python_node:main'
            #"ros2_python_node"    创建后的可执行文件名字
            #"python_package.python_node:main"    包名.文件名:函数名
        ],
    },
```
在`package.xml`中添加运行依赖`<depend>rclpy<depend>`
## 构建功能包

```bash
colcon build
```
![构建功能包](https://i-blog.csdnimg.cn/direct/13a5535cfcdc451f88938e961900a36d.png)
构建所有功能包`colcon build`
或者只构建特定功能包`colcon build --packages-select my_python_package`
构建时显示详细输出`colcon build --packages-select my_python_package --event-handlers console_direct+`

添加环境变量：

```bash
export PYTHONPATH=/home/sun/ros2/install/python_package/lib/python3.10/site-packages:$PYTHONPATH
```
查看环境变量：

```bash
printenv | grep PYTHON
```
运行节点：
![功能包运行](https://i-blog.csdnimg.cn/direct/8993dfd3496e4069badc7847feebf512.png)
## 加载全部环境变量

```bash
source install/setup.bash
```
运行节点：

```bash
ros2 run python_package ros2_python_node
```
## 遇到的未解决的问题，请指教！！！
我使用`source install/setup.bash`后运行`ros2 run python_package ros2_python_node`会报错，看了看ros环境变量没加载（我是在wsl2-Ubuntu2204中，sh脚本我也试了）
![遇到的问题](https://i-blog.csdnimg.cn/direct/83ce61034ac541a4a8c30de534d0cb8d.png)
最后手动加载的环境变量
![手动加载环境变量](https://i-blog.csdnimg.cn/direct/fa91a449f5f84bfc8881cdf4c0c5a8f2.png)