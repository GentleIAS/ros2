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
        self.save_dir = os.path.expanduser('~/network_camera')
        #如果路径不存在则创建
        os.makedirs(self.save_dir, exist_ok=True)
        
        #RTSP摄像头地址配置
        self.rtsp_url = 'rtsp://xxxxx'     #网络摄像头RTSP地址
        self.cap = None                    #OpenCV视频捕获对象
        self.capture_thread = None         #图像捕获线程
        self.capture_flag = False          #捕获线程运行标志

        #初始化视频录制相关变量
        self.rosbag_process = None         #当前录制进程
        self.next_rosbag_process = None    #下一个录制进程
        self.recording_flag = False        #录制标志
        self.message_count = 0             #消息计数
        self.bag_filename = None           #bag文件名
        self.recording_start_time = None   #录制开始时间
        self.video_duration = 60           #视频时长(s)
        self.file_counter = 1              #文件计数器
        
        #关闭标志
        self._shutdown_requested = False
        #切换标志，防止重复切换
        self._switching_recording = False

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
            
            #尝试使用GPU硬件解码
            success = False
            decode_method = "未知"
            
            #使用GStreamer管道进行GPU硬件解码
            if not success:
                try:
                    gst_pipeline = f"rtspsrc location={self.rtsp_url} latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1 max-buffers=1"
                    self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                    if self.cap.isOpened():
                        decode_method = "GStreamer GPU硬件解码"
                        success = True
                except Exception as e:
                    self.get_logger().warn(f'GStreamer初始化失败: {str(e)}')
            
            #获取摄像头参数
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            self.get_logger().info(f'摄像头连接成功 - 解码方式: {decode_method}, 分辨率: {width}x{height}, 帧率: {fps:.1f}')
            
            #启动图像捕获线程
            self.capture_running = True
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'初始化摄像头时出错: {str(e)}')
            return False

    #图像捕获线程
    def capture_loop(self):
        while self.capture_running and self.cap is not None and not self._shutdown_requested:
            try:
                #从摄像头读取一帧图像
                ret, frame = self.cap.read()
                
                if not ret:
                    if not self._shutdown_requested:
                        self.get_logger().warn('无法读取摄像头画面，尝试重连...')
                    time.sleep(1.0)
                    continue
                
                #获取当前时间戳
                current_time = time.time()
                
                with self.lock:
                    #检查节点是否仍然有效
                    if self._shutdown_requested:
                        break
                        
                    #将OpenCV图像转换为ROS消息
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                        #设置时间戳
                        ros_image.header.stamp = self.get_clock().now().to_msg()
                        ros_image.header.frame_id = 'camera_frame'
                        
                        #检查发布者是否仍然有效
                        if not self._shutdown_requested and hasattr(self, 'image_publisher'):
                            try:
                                self.image_publisher.publish(ros_image)
                                
                                #增加消息计数
                                if self.recording_flag:
                                    self.message_count += 1
                            except Exception as pub_e:
                                if not self._shutdown_requested:
                                    self.get_logger().error(f'发布消息失败: {str(pub_e)}')
                                break
                        
                        #检查是否需要切换录制文件（移除此处的切换逻辑，统一在status_callback中处理）
                        # 切换逻辑已移至status_callback中统一处理，避免重复触发
                            
                    except Exception as e:
                        if hasattr(self, 'get_logger') and not self._shutdown_requested:
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
    def start_recording(self, is_next=False):
        try:
            #生成带时间戳的视频文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_filename = os.path.join(self.save_dir, f'{self.file_counter:03d}_{timestamp}')

            #构建rosbag命令
            cmd = [
                'ros2', 'bag', 'record',
                '/image_raw',                       #录制的话题
                '-o', bag_filename,                 #输出文件名
                '--compression-mode', 'file',       #压缩模式
                '--compression-format', 'zstd'      #压缩格式
            ]
            
            #启动rosbag录制进程
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self.save_dir
            )
            
            if is_next:
                self.next_rosbag_process = process
            else:
                self.rosbag_process = process
                self.bag_filename = bag_filename
                self.recording_flag = True
                self.message_count = 0
                self.recording_start_time = time.time()
                self.get_logger().info(f'开始录制: {bag_filename}')
                
        except Exception as e:
            self.get_logger().error(f'开始录制时出错: {str(e)}')
            if not is_next:
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
    
    #停止旧的录制进程
    def _stop_old_process(self, process, message_count, bag_filename):
        if not process:
            return
            
        try:
            #尝试优雅停止
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except:
                        pass
            
            #显示录制完成信息
            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self._show_file_info(message_count, bag_filename)
                
        except Exception as e:
            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self.get_logger().error(f'停止旧录制进程时出错: {str(e)}')
    
    #异步显示文件信息
    def _show_file_info(self, message_count=0, bag_filename=None):
        try:
            if bag_filename is None:
                bag_filename = self.bag_filename
                
            if bag_filename and os.path.exists(bag_filename):
                #获取文件大小
                total_size = 0
                for dirpath, _, filenames in os.walk(bag_filename):
                    for filename in filenames:
                        filepath = os.path.join(dirpath, filename)
                        total_size += os.path.getsize(filepath)
                
                size_mb = total_size / (1024 * 1024)
                if hasattr(self, 'get_logger') and not self._shutdown_requested:
                    self.get_logger().info(f'录制完成: {message_count} 条消息, {size_mb:.2f} MB')
                    
        except Exception as e:
            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self.get_logger().error(f'显示文件信息时出错: {str(e)}')
    
    #无缝切换录制文件
    def _seamless_switch_recording(self):
        if self._shutdown_requested or self._switching_recording:
            return
            
        self._switching_recording = True
        
        try:
            #停止当前录制进程
            old_process = self.rosbag_process
            old_message_count = self.message_count
            old_bag_filename = self.bag_filename
            
            #切换到下一个录制进程
            if self.next_rosbag_process:
                self.rosbag_process = self.next_rosbag_process
                self.next_rosbag_process = None
                
                #生成新的文件名
                self.file_counter += 1
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.bag_filename = os.path.join(self.save_dir, f'{self.file_counter:03d}_{timestamp}')
                
                self.recording_start_time = time.time()
                self.message_count = 0
                
                self.get_logger().info(f'切换完成，共录制 {old_message_count} 条消息')
            else:
                #如果没有预启动的进程，立即启动新的
                self.recording_flag = False
                self.file_counter += 1
                self.start_recording()
            
            #异步停止旧的录制进程
            if old_process:
                threading.Thread(target=self._stop_old_process, args=(old_process, old_message_count, old_bag_filename), daemon=True).start()
            
        except Exception as e:
            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self.get_logger().error(f'切换录制时出错: {str(e)}')
        finally:
            self._switching_recording = False    
    

    
    #检查录制状态
    def status_callback(self):
        if (self.recording_flag and 
            self.recording_start_time is not None and 
            not self._shutdown_requested and 
            not self._switching_recording):  # 避免在切换过程中重复触发
            
            #计算实际录制时长
            current_time = time.time()
            elapsed_time = current_time - self.recording_start_time
            remaining_time = max(0, self.video_duration - elapsed_time)
            
            #显示录制进度
            if remaining_time > 0:
                self.get_logger().info(f'已录制: {elapsed_time:.0f}秒, 剩余: {remaining_time:.0f}秒, 消息数: {self.message_count}')
            
            #在55秒时预启动下一个录制
            if elapsed_time >= (self.video_duration - 5) and not self.next_rosbag_process:
                self.start_recording(is_next=True)
            
            #在60秒时切换录制文件
            if elapsed_time >= self.video_duration:
                threading.Thread(target=self._seamless_switch_recording, daemon=True).start()
    
    def __del__(self):
        self.cleanup()
    
    #清理资源
    def cleanup(self):
        try:
            #设置关闭标志
            self._shutdown_requested = True
            
            #停止录制
            if self.recording_flag:
                self.stop_recording()  #使用同步版本确保完全停止
            
            #停止图像捕获线程
            if hasattr(self, 'capture_running'):
                self.capture_running = False
            if hasattr(self, 'capture_thread') and self.capture_thread and self.capture_thread.is_alive():
                self.capture_thread.join(timeout=3)  #增加超时时间

            #关闭OpenCV窗口
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            #释放摄像头资源
            if hasattr(self, 'cap') and self.cap:
                try:
                    self.cap.release()
                    self.cap = None
                    if hasattr(self, 'get_logger') and not self._shutdown_requested:
                        self.get_logger().info('摄像头资源已释放')
                except:
                    pass

            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self.get_logger().info('资源清理完成')
            
        except Exception as e:
            if hasattr(self, 'get_logger') and not self._shutdown_requested:
                self.get_logger().error(f'清理资源时出错: {str(e)}')

def main(args=None):
    camera_recorder = None
    try:
        rclpy.init(args=args)
        
        camera_recorder = NetworkCamera()  #创建网络摄像头节点
        
        try:
            rclpy.spin(camera_recorder)
        except KeyboardInterrupt:
            print("\n正在停止录制...")
            
    except Exception as e:
        print(f"程序运行出错: {str(e)}")
        
    finally:
        try:
            # 清理资源
            if camera_recorder:
                camera_recorder.cleanup()
                try:
                    camera_recorder.destroy_node()
                except:
                    pass
            
            # 检查RCL是否已经关闭，避免重复关闭
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass
            
        except Exception as e:
            print(f"清理时出错: {str(e)}")

if __name__ == '__main__':
    main()