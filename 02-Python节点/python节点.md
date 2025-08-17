# Python节点
## ros2节点
ros2节点（Node）是ros2系统中的 基本计算单元 ，可以理解为一个独立运行的程序或进程。每个节点都有自己的功能和职责，多个节点通过通信机制协同工作，共同完成复杂的机器人任务。
### 节点的核心特点
- 1.独立性 ：每个节点都是独立的进程，可以单独启动和关闭
- 2.通信能力 ：节点之间可以通过话题、服务、动作等方式进行通信
- 3.模块化 ：将复杂系统分解为多个简单的功能模块
- 4.分布式 ：节点可以运行在不同的计算机上
### 节点的主要功能 
- 1.消息发布（Publisher）
  - 向特定话题发送数据
- 2.消息订阅（Subscriber）
  - 从特定话题接收数据
-  3.服务提供（Service Server）
   - 提供请求-响应式的服务
- 4.服务调用（Service Client）
  - 调用其他节点提供的服务
## Python节点代码

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

if __name__ == "__main__":
    main()
```
运行Python节点代码后可以看到
![Python节点代码输出](https://i-blog.csdnimg.cn/direct/4ff276c9e52242349bb2f11d7843e94f.png)
信息级别的日志'`[INFO]`'为白色打印
错误级别的日志'`[WARN]`'为黄色打印
日志级别后跟的数字为时间戳

在新终端中输入`ros2 node list`可以查看节点列表
![查看节点](https://i-blog.csdnimg.cn/direct/b04251fd563c475283e9af5f69a814a2.png)
## 通过环境变量修改日志格式
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT=[{function_name}:{line_number}:{message}]
```
`{function_name}`函数名字
`{line_number}`行号
`{message}`打印消息
![日志打印修改](https://i-blog.csdnimg.cn/direct/cec79111b3e541c0a5867ebbd1e0c7b5.png)
## 日志格式化占位符
- 时间相关
  - {time} - 完整时间戳
  - {time_as_nanoseconds} - 纳秒级时间戳
  - {time_as_seconds} - 秒级时间戳 
 - 日志级别
   - {severity} - 日志级别（INFO, WARN, ERROR, DEBUG等） 
- 位置信息
  - {function_name} - 函数名称
  - {file_name} - 文件名
  - {line_number} - 行号 
- 节点信息
  - {name} - 日志记录器名称（通常是节点名） 
- 消息内容
  - {message} - 实际的日志消息内容