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