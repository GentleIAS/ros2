# 没有标题，工作原因超前做了个网络红外摄像头读取与rosbag储存（当然是与AI“合作”的！）
## 硬件
- 1.开发板：NVIDIA Jetson Orin NX 16+128
- 2.摄像头：网络红外摄像头
## 软件
- 1.Ubuntu2204--ros2
- 2.python环境
### ros工具：rosbag
rosbag 是ROS中的一个重要工具，用于 录制和回放ROS消息数据 。
- 1.数据采集：
  - 传感器数据 ：摄像头图像、激光雷达点云、IMU数据等
  - 机器人状态 ：位置、速度、关节角度等
  - 环境信息 ：温度、湿度、GPS坐标等 
- 2.调试和测试
  - 离线分析 ：录制数据后可以反复分析
  - 算法测试 ：用相同数据测试不同算法
  - 问题复现 ：保存出现问题时的数据
- 3.数据共享
  - 团队协作 ：共享实验数据
  - 基准测试 ：提供标准测试数据集
  - 演示展示 ：重现机器人运行场景
### 代码
#### ROS2相关库
```python
import rclpy                       #ROS2 Python客户端库
from rclpy.node import Node        #ROS2节点基类
from sensor_msgs.msg import Image  #ROS2图像消息类型
from cv_bridge import CvBridge     #OpenCV与ROS图像消息转换桥接库
```
#### ython标准库和第三方库
```python
import cv2                      #OpenCV计算机视觉库 (第三方库)
import os                       #操作系统接口库 (Python标准库)
import threading                #多线程库 (Python标准库)
from datetime import datetime   #日期时间处理库 (Python标准库)
import time                     #时间相关函数库 (Python标准库)
import subprocess               #子进程管理库 (Python标准库)
import signal                   #信号处理库 (Python标准库)
```
#### OS库
##### `os.path.expanduser(path)`路径处理
```python
os.path.expanduser() 用于展开用户目录路径
主要功能展开波浪号（~）
将路径中的 ~ 符号替换为当前用户的主目录路径

跨平台兼容性
1.自动适配不同操作系统的用户目录结构
2.Linux/macOS ： ~ → /home/username
3.Windows ： ~ → C:\Users\username
```
##### `os.makedirs()`创建文件夹
```python
os.makedirs('新文件夹')

# 会自动创建所有不存在的父目录
os.makedirs('父目录/子目录/孙目录')

# exist_ok=False（默认）：如果目录已存在会报错
os.makedirs('test_dir')  # 第二次运行会报 FileExistsError

# exist_ok=True：如果目录已存在不会报错
os.makedirs('test_dir', exist_ok=True)  # 安全，不会报错

# 设置目录权限（Windows上无效）
os.makedirs('test_dir', mode=0o755)
```
##### `os.path.join()` 用于智能拼接文件路径
```python
基本功能
os.path.join() 用于将多个路径组件智能地连接成一个完整的路径，它会根据操作系统自动选择正确的路径分隔符。

path = os.path.join(path1, path2, path3, ...)

参数说明
- 可变参数 ：接受任意数量的路径组件（字符串）
- 自动处理分隔符 ：在Windows上使用反斜杠 \ ，在Unix/Linux上使用正斜杠 /
- 智能拼接 ：自动处理路径组件之间的分隔符，避免重复或缺失
```
##### `os.walk()` 用于递归遍历目录树
```python
基本功能
os.walk() 用于生成目录树中所有文件和子目录的路径，它会递归地遍历指定目录及其所有子目录

for dirpath, dirnames, filenames in os.walk(top):
    # 处理当前目录
    pass

返回值说明
os.walk() 返回一个生成器，每次迭代产生一个三元组 (dirpath, dirnames, filenames) ：
1. dirpath ：字符串，当前正在遍历的目录路径
2.dirnames ：列表，当前目录中所有子目录的名称
3.filenames ：列表，当前目录中所有文件的名称
```
#### ROS2库
##### `create_publisher()`Node类，用于创建消息发布者
```python
create_publisher(msg_type, topic, qos_profile)

参数说明
1.msg_type （消息类型）：要发布的消息类型
   - 例如： Image （图像消息）、 String （字符串消息）等
   - 必须是ROS2标准消息类型或自定义消息类型
2.topic （话题名）：发布消息的话题名称
   - 字符串格式，例如： '/camera/image' 、 '/status' 等
   - 其他节点可以通过订阅这个话题来接收消息
3.qos_profile （队列大小/QoS配置）：服务质量配置
   - 简单用法：直接传入整数表示队列大小，如 10
   - 高级用法：传入QoS配置对象

特点
1.异步发布 ：不会阻塞程序执行
2.自动重连 ：网络断开时自动重新连接
3.类型安全 ：确保发布的消息类型正确
4.资源管理 ：节点销毁时自动清理发布者
```
##### `publish()`用于将消息发布到指定的话题上
```python
publisher.publish(message)

message : 要发布的消息对象，必须与发布器创建时指定的消息类型匹配
```

##### `cv2_to_imgmsg()` CvBridge 类，用于将 OpenCV 图像格式转换为 ROS 图像消息格式
```python
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding)

参数说明
1.cv_image : OpenCV 图像对象
2.encoding : 图像编码格式字符串，如 'bgr8', 'rgb8', 'mono8' 等

常见编码格式
1.'bgr8' : 8位BGR彩色图像（OpenCV默认格式）
2.'rgb8' : 8位RGB彩色图像
3.'mono8' : 8位灰度图像
4.'bgra8' : 8位BGRA图像（带透明通道）
5.'rgba8' : 8位RGBA图像（带透明通道）
6.'16UC1' : 16位单通道图像
7.'32FC1' : 32位浮点单通道图像

反向转换
如果需要将ROS图像消息转换回OpenCV格式，使用相反的方法：
cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
```
##### `create_timer()` Node类，用于创建定时器，实现周期性任务执行
```python
create_timer(timer_period_sec, callback)

### 参数说明
1.timer_period_sec （时间间隔）：定时器触发的时间间隔
   - 单位：秒（浮点数）
   - 例如： 0.1 （100毫秒）、 1.0 （1秒）、 0.033 （约30FPS）
2.callback （回调函数）：定时器触发时要执行的函数
   - 通常是类的方法，如 self.timer_callback
   - 函数不需要参数，返回值会被忽略

工作原理
1.周期性执行 ：ROS2在后台管理定时器，按时触发回调
2.非阻塞 ：定时器在独立线程中运行，不会阻塞主程序
3.自动管理 ：节点销毁时自动停止定时器
4. 精确计时 ：使用系统时钟确保时间精度

特点
1.线程安全 ：ROS2确保回调函数的线程安全执行
2.资源高效 ：使用系统级定时器，资源消耗低
3.可控制 ：可以通过保存返回的定时器对象来控制启停
4.异常处理 ：回调函数中的异常不会影响定时器继续运行
```
#### 标准库
##### `threading.Thread()` 用于创建和管理线程的核心类
```python
thread = threading.Thread(target=function, args=(), kwargs={}, daemon=False)

主要参数说明
1.target : 线程要执行的目标函数
2.args : 传递给目标函数的位置参数（元组形式）
3.kwargs : 传递给目标函数的关键字参数（字典形式）
4.daemon : 是否为守护线程（布尔值）
5.name : 线程名称（可选）

守护线程 (daemon) 详解
1.daemon=True : 守护线程，主程序结束时会自动终止
2.daemon=False : 非守护线程，主程序会等待该线程完成才退出

常用方法
1.thread.start() : 启动线程
2.thread.join() : 等待线程完成
3.thread.is_alive() : 检查线程是否还在运行
4.thread.getName() : 获取线程名称
```
##### `threading.Lock()` 用于创建线程锁对象，实现多线程环境下的资源同步
```python
主要功能
创建一个互斥锁（Mutex Lock），确保在多线程环境中同一时间只有一个线程能够访问共享资源

基本用法
# 创建锁对象
lock = threading.Lock()
# 获取锁
lock.acquire()
try:
    # 执行需要同步的代码
    print("正在访问共享资源")
finally:
    # 释放锁
    lock.release()

重要特性
1.互斥性 ：同一时间只有一个线程能获得锁
2.可重入性 ：同一线程可以多次获取同一个锁（需要相应次数的释放）
3.阻塞性 ：如果锁被占用，其他线程会等待直到锁被释放
4.异常安全 ：使用with语句确保即使发生异常也能正确释放锁

性能考虑
1.锁的粒度 ：锁保护的代码块应该尽可能小，减少等待时间
2.避免长时间持有锁 ：不要在锁内执行耗时操作
3.合理使用锁 ：只在真正需要同步的地方使用锁
```
##### `time.time()`用于获取当前时间的时间戳
```python
基本功能
1.返回值 : 浮点数，表示从1970年1月1日00:00:00 UTC到当前时间的秒数
2.精度 : 通常精确到微秒级别（小数点后6位）
3.时区 : 返回的是UTC时间戳，与本地时区无
```
##### `time.sleep()`用于暂停程序执行指定的时间
```python
time.sleep(seconds)

参数说明
1.seconds : 浮点数或整数，表示暂停的秒数
2.可以是小数，如 0.1 表示暂停100毫秒
3.最小精度取决于系统，通常可以精确到毫秒级
```
##### `hasattr()`用于检查对象是否具有指定的属性或方法
```python
hasattr(object, name)

参数说明
1.object : 要检查的对象
2.name : 属性或方法名称（字符串格式）
3.返回值 : 布尔值， True 表示存在， False 表示不存在
```
##### `datetime.now()` 和 `strftime()` 用于处理日期和时间
```python
datetime.now() 是 datetime 模块中的方法，用于获取当前的本地日期和时间

current_time = datetime.now()

返回一个 datetime 对象，包含当前的年、月、日、时、分、秒和微秒信息

strftime() 是 "string format time" 的缩写，用于将 datetime 对象格式化为指定格式的字符串

常用格式代码
- %Y ：四位数年份（如：2024）
- %m ：月份（01-12）
- %d ：日期（01-31）
- %H ：小时（00-23）
- %M ：分钟（00-59）
- %S ：秒（00-59）
- %A ：完整星期名称
- %B ：完整月份名称
```
##### `subprocess.Popen()` 用于创建和管理子进程
```python
基本功能
subprocess.Popen() 用于启动新的进程，执行外部命令或程序，并提供与子进程交互的接口。它是Python中执行系统命令的强大工具。

process = subprocess.Popen(args, **kwargs)

必需参数
- args ：要执行的命令，可以是字符串或列表
常用可选参数
1.stdout ：标准输出重定向
  - subprocess.PIPE ：捕获输出
  - subprocess.DEVNULL ：丢弃输出
  - 文件对象：重定向到文件
2.stderr ：标准错误重定向
3.stdin ：标准输入重定向
4.shell ：是否通过shell执行命令
5.cwd ：设置工作目录
6.env ：设置环境变量
```
#### OpenCV库
##### `cv2.VideoCapture()`用于视频捕获的核心函数，可以从多种视频源读取视频流
```python
cap = cv2.VideoCapture(source, apiPreference)

参数说明
1.source : 视频源，可以是：
  - 整数：摄像头设备ID（如 0, 1, 2...）
  - 字符串：视频文件路径或网络流URL
  - GStreamer管道字符串
2.apiPreference (可选): 后端API偏好，常用值：
  - cv2.CAP_GSTREAMER : GStreamer后端
  - cv2.CAP_FFMPEG : FFmpeg后端
  - cv2.CAP_DSHOW : DirectShow后端（Windows）

常用方法
1.cap.read() : 读取一帧图像
2.cap.isOpened() : 检查是否成功打开
3.cap.release() : 释放资源
4.cap.get(propId) : 获取属性值
5.cap.set(propId, value) : 设置属性值
```
##### `cv2.imshow()` 用于显示图像
```python
cv2.imshow(window_name, image)

参数说明
1.window_name ：字符串类型，窗口的名称标识符。如果窗口不存在，会自动创建一个新窗口；如果已存在同名窗口，则在该窗口中显示新图像
2.image ：要显示的图像，通常是NumPy数组格式，可以是灰度图像或彩色图像

常用配套函数
通常与以下函数配合使用：
1.cv2.waitKey() ：等待键盘输入，控制窗口刷新
2.cv2.destroyAllWindows() ：关闭所有OpenCV创建的窗口
3.cv2.destroyWindow(window_name) ：关闭指定名称的窗口
```