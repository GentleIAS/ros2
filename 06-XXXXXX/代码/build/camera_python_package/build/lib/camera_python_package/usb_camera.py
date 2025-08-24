import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import os 
import threading 
from datetime import datetime   
import time 
import subprocess
import signal

class NetworkCamera(Node):
    def __init__(self):
        super().__init__('camera_network')

        #CvBridge对象，转换ROS图像消息和OpenCV图像
        self.bridge = CvBridge()
        #设置视频保存路径
        self.save_dir = os.path.expanduser('/home/mepus/network_camera')
        #如果路径不存在则创建
        os.makedirs(self.save_dir, exist_ok=True)
        
        #RTSP摄像头地址配置
        self.rtsp_url = 'rtsp://admin:js12345@192.168.1.254:554/live'   #网络摄像头RTSP地址
        self.cap = None                    #OpenCV视频捕获对象
        self.capture_thread = None         #图像捕获线程
        self.capture_flag = False          #捕获线程运行标志

        #初始化视频录制相关变量
        self.rosbag_process = None         #录制进程
        self.recording_flag = False        #录制标志
        self.message_count = 0             #消息计数
        self.bag_filename = None           #bag文件名
        self.recording_start_time = None   #录制开始时间
        self.video_duration = 5 * 60       #视频时长

        """#订阅/image_raw话题，当收到图像消息时调用image_receive函数
        self.subscription = self.create_subscription(
            Image,                         #消息类型
            '/image_raw',                  #话题名称
            self.image_receive,            #回调函数
            10                             #队列大小
        )"""
        #创建图像发布者，发布到/image_raw话题
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)

        #创建定时器，检查录制是否需要切换视频文件
        self.timer = self.create_timer(1.0, self.status_callback)

        self.lock = threading.Lock()

        self.get_logger().info('网络摄像头节点启动成功')
        
        #初始化摄像头连接
        self.init_camera()
        

        #开始rosbag录制
        self.start_recording()

    #初始化摄像头连接
    def init_camera(self):
        try:
            self.get_logger().info(f'正在连接网络摄像头: {self.rtsp_url}')
            
            #创建VideoCapture对象
            self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            
            #设置缓冲区大小
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            #设置硬件解码参数
            try:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
                self.get_logger().info('已启用H.264硬件解码')
            except:
                self.get_logger().warn('硬件解码不可用，使用软件解码')
            
            if not self.cap.isOpened():
                self.get_logger().error('无法连接到网络摄像头')
                return False
                
            #获取摄像头参数
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            self.get_logger().info(f'摄像头连接成功 - 分辨率: {width}x{height}, 帧率: {fps}')
            
            # 启动图像捕获线程
            self.capture_running = True
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'初始化摄像头时出错: {str(e)}')
            return False

    #图像捕获线程
    def capture_loop(self):
        while self.capture_running and self.cap is not None:
            try:
                #从摄像头读取一帧图像
                ret, frame = self.cap.read()
                
                if not ret:
                    self.get_logger().warn('无法读取摄像头画面，尝试重连...')
                    time.sleep(1.0)
                    continue
                
                #获取当前时间戳
                current_time = time.time()
                
                with self.lock:
                    #将OpenCV图像转换为ROS消息
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                        #设置时间戳
                        ros_image.header.stamp = self.get_clock().now().to_msg()
                        ros_image.header.frame_id = 'camera_frame'
                        
                        #发布图像消息到/image_raw话题
                        self.image_publisher.publish(ros_image)
                        
                        #增加消息计数
                        if self.recording_flag:
                            self.message_count += 1
                        
                        #检查是否需要切换录制文件）
                        if (self.recording_flag and 
                            self.recording_start_time is not None and 
                            current_time - self.recording_start_time >= self.video_duration):
                            self.switch_recording(frame)
                            
                    except Exception as e:
                        self.get_logger().error(f'ROS消息转换失败: {str(e)}')
                        continue
                
                #显示实时画面
                try:
                    cv2.imshow('网络摄像头实时画面', frame)
                    cv2.waitKey(1)
                except:
                    pass
                    
            except Exception as e:
                self.get_logger().error(f'图像捕获出错: {str(e)}')
                time.sleep(0.1)
        
        self.get_logger().info('图像捕获线程已停止')

    #开始录制视频
    def start_recording(self):
        try:
            #生成带时间戳的视频文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.bag_filename = os.path.join(self.save_dir, f'{timestamp}')

            #构建rosbag命令
            cmd = [
                'ros2', 'bag', 'record',
                '/image_raw',                       #录制的话题
                '-o', self.bag_filename,            #输出文件名
                '--compression-mode', 'file',       #压缩模式
                '--compression-format', 'zstd'      #压缩格式
            ]
            
            #启动rosbag录制进程
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self.save_dir
            )
            
            #检查rosbag录制进程是否成功
            if self.rosbag_process:
                self.recording_flag = True
                self.message_count = 0
                self.recording_start_time = time.time()       #记录录制开始时间
                self.get_logger().info(f'开始录制视频: {self.bag_filename}')
                self.get_logger().info(f'录制话题: /image_raw')
            else:
                self.get_logger().error('无法创建视频录制器')
                self.rosbag_process = None
                self.recording_flag = False
                
        except Exception as e:
            self.get_logger().error(f'开始录制时出错: {str(e)}')
            self.recording_flag = False
            if self.rosbag_process is not None:
                try:
                    self.rosbag_process.terminate()
                    self.rosbag_process.wait()
                except:
                    pass
                self.rosbag_process = None
    
    #录制视频停止
    def stop_recording(self):
        try:
            if self.recording_flag and self.rosbag_process is not None:
                #释放视频录制器资源
                self.rosbag_process.send_signal(signal.SIGINT)        #发送SIGINT信号停止rosbag进程
                self.rosbag_process.wait()                            #等待进程结束
                self.rosbag_process = None
                self.recording_flag = False
                    
                self.get_logger().info(f'录制完成！共录制 {self.message_count} 条消息')
                self.get_logger().info(f'视频已保存至: {self.bag_filename}')
                    
                #显示文件信息
                if os.path.exists(self.bag_filename):
                    total_size = 0
                    for dirpath, _, filenames in os.walk(self.bag_filename):
                        for filename in filenames:
                            filepath = os.path.join(dirpath, filename)
                            total_size += os.path.getsize(filepath)
                    file_size = total_size / (1024 * 1024)
                    self.get_logger().info(f'文件大小: {file_size:.2f} MB')
                        
        except Exception as e:
            self.get_logger().error(f'停止录制时出错: {str(e)}')
            self.recording_flag = False
            self.rosbag_process = None    
    
    #文件切换时调用，先停止当前录制，再开始新的录制
    def switch_recording(self, first_frame):
        try:
            #停止当前录制
            if self.recording_flag:
                self.stop_recording()
                time.sleep(0.1)  
            
            #开始新的录制
            self.start_recording()
            
        except Exception as e:
            self.get_logger().error(f'切换录制文件时出错: {str(e)}')
    
    #检查录制状态
    def status_callback(self):
        if self.recording_flag and self.recording_start_time is not None:
            #计算实际录制时长
            current_time = time.time()
            elapsed_time = current_time - self.recording_start_time
            remaining_time = max(0, self.video_duration - elapsed_time)
            
            if remaining_time > 0:
                self.get_logger().info(f'已录制: {elapsed_time:.0f}秒, 剩余: {remaining_time:.0f}秒, 消息数: {self.message_count}')
            elif elapsed_time >= self.video_duration:
                self.get_logger().warn(f'录制时间已超过设定时长({self.video_duration}秒)，当前已录制{elapsed_time:.1f}秒')
    
    def __del__(self):
        self.cleanup()
    
    #清理资源
    def cleanup(self):
        try:
            #停止录制
            if self.recording_flag:
                self.stop_recording()
            
            # 停止图像捕获线程
            if hasattr(self, 'capture_running'):
                self.capture_running = False

            #关闭OpenCV窗口
            cv2.destroyAllWindows()
            
            # 释放摄像头资源
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
                self.cap = None
                self.get_logger().info('摄像头资源已释放')

            self.get_logger().info('资源清理完成')
            
        except Exception as e:
            self.get_logger().error(f'清理资源时出错: {str(e)}')

def main(args=None):
    try:
        rclpy.init(args=args)
        
        camera_recorder = NetworkCamera()  # 创建网络摄像头节点
        
        try:
            rclpy.spin(camera_recorder)
        except KeyboardInterrupt:
            print("\n正在停止录制...")
            if camera_recorder:
                camera_recorder.stop_recording()
            
    except Exception as e:
        print(f"程序运行出错: {str(e)}")
        
    finally:
        try:
            # 清理资源
            if camera_recorder:
                camera_recorder.cleanup()
                camera_recorder.destroy_node()
            
            # 检查RCL是否已经关闭，避免重复关闭
            if rclpy.ok():
                rclpy.shutdown()
            
        except Exception as e:
            print(f"清理时出错: {str(e)}")

if __name__ == '__main__':
    main()