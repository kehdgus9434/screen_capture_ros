#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import threading
import time
import os
import tempfile
import base64
from io import BytesIO

class StableScreenCapturePublisher(Node):
    def __init__(self):
        super().__init__('stable_screen_capture_publisher')
        
        # 파라미터 선언
        self.declare_parameter('window_names', ['Camera', 'Map'])  # 캡처할 창 이름
        self.declare_parameter('publish_rate', 30.0)  # 발행 주기 (Hz)
        self.declare_parameter('debug', True)  # 디버그 메시지
        self.declare_parameter('resize_factor', 1.0)  # 리사이징 비율
        
        # 파라미터 가져오기
        self.window_names = self.get_parameter('window_names').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.resize_factor = self.get_parameter('resize_factor').value
        
        # 창 정보와 쓰레드 저장
        self.window_info = {}
        self.window_threads = {}
        self.thread_locks = {}
        self.running = True
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 각 창에 대한 퍼블리셔 생성
        for window_name in self.window_names:
            self.window_info[window_name] = {
                'window_id': None,
                'title': window_name,
                'x': 0, 'y': 0, 'width': 0, 'height': 0,
                'fps': 0.0, 'frame_count': 0,
                'last_time': time.time(),
                'last_frame_time': time.time(),
                'resize_width': 0, 'resize_height': 0,
                'pipe': None,
                'capture_count': 0,
                'error_count': 0
            }
            
            # 토픽 이름
            safe_name = self.get_safe_name(window_name)
            
            # 이미지 퍼블리셔
            self.window_info[window_name]['publisher'] = self.create_publisher(
                Image, f'screen_capture/{safe_name}/image', 1
            )
            
            # 정보 퍼블리셔
            self.window_info[window_name]['info_publisher'] = self.create_publisher(
                String, f'screen_capture/{safe_name}/info', 10
            )
            
            # 쓰레드 락
            self.thread_locks[window_name] = threading.Lock()
        
        # 타이머 생성
        self.find_window_timer = self.create_timer(5.0, self.find_all_windows)
        
        # 창 찾기
        self.find_all_windows()
        
        # 캡처 쓰레드 시작
        self.start_capture_threads()
        
        self.get_logger().info(
            f'StableScreenCapturePublisher 초기화 완료: '
            f'창 이름 목록={self.window_names} | '
            f'발행 주기={self.publish_rate}Hz | '
            f'리사이징 비율={self.resize_factor}'
        )
    
    def get_safe_name(self, window_name):
        """창 이름을 안전한 토픽 이름으로 변환"""
        safe_name = window_name.lower()
        safe_name = safe_name.replace(' ', '_')
        safe_name = safe_name.replace('-', '_')
        safe_name = ''.join(c for c in safe_name if c.isalnum() or c == '_')
        return safe_name
    
    def find_all_windows(self):
        """wmctrl로 모든 창 찾기"""
        try:
            # 창 목록 가져오기
            result = subprocess.run(['wmctrl', '-l', '-G', '-p'], capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error("wmctrl 명령 실패. wmctrl을 설치하세요: sudo apt install wmctrl")
                return
            
            window_list = result.stdout.strip().split('\n')
            found_windows = []
            
            for line in window_list:
                if not line.strip():
                    continue
                
                # wmctrl 출력 파싱
                parts = line.split(None, 8)
                if len(parts) >= 9:
                    window_id = parts[0]
                    x, y, width, height = map(int, parts[3:7])
                    title = parts[8]
                    
                    # 지정한 창 이름 검색
                    for window_name in self.window_names:
                        if window_name.lower() in title.lower():
                            info = self.window_info[window_name]
                            
                            # 창 정보 변경 시 로그 출력
                            if info['window_id'] != window_id or \
                               info['width'] != width or info['height'] != height:
                                self.get_logger().info(
                                    f"'{window_name}' 창 찾음: ID={window_id}, "
                                    f"위치=({x}, {y}), 크기={width}x{height}"
                                )
                                
                                info['window_id'] = window_id
                                info['title'] = title
                                info['x'] = x
                                info['y'] = y
                                info['width'] = width
                                info['height'] = height
                                
                                # 리사이징 설정
                                if self.resize_factor != 1.0:
                                    info['resize_width'] = int(width * self.resize_factor)
                                    info['resize_height'] = int(height * self.resize_factor)
                                else:
                                    info['resize_width'] = width
                                    info['resize_height'] = height
                            
                            found_windows.append(window_name)
            
            # 찾지 못한 창 출력
            for window_name in self.window_names:
                if window_name not in found_windows and self.debug:
                    self.get_logger().warn(f"'{window_name}' 창을 찾을 수 없습니다.")
                    
        except Exception as e:
            self.get_logger().error(f"창 찾기 오류: {str(e)}")
    
    def start_capture_threads(self):
        """각 창에 대한 캡처 쓰레드 시작"""
        for window_name in self.window_names:
            # 기존 쓰레드가 있으면 건너뛰기
            if window_name in self.window_threads and self.window_threads[window_name].is_alive():
                continue
                
            # 쓰레드 생성 및 시작
            thread = threading.Thread(
                target=self.capture_and_publish_thread,
                args=(window_name,),
                daemon=True
            )
            self.window_threads[window_name] = thread
            thread.start()
    
    def capture_and_publish_thread(self, window_name):
        """창 캡처 및 발행 쓰레드"""
        info = self.window_info[window_name]
        lock = self.thread_locks[window_name]
        wait_time = 1.0 / self.publish_rate
        
        while rclpy.ok() and self.running:
            start_time = time.time()
            
            # 창 ID가 없으면 다음 반복으로
            if info['window_id'] is None:
                time.sleep(wait_time)
                continue
            
            try:
                # 파이프로 캡처
                with lock:
                    image = self.capture_window_with_pipe(info)
                    
                if image is None:
                    time.sleep(wait_time)
                    continue
                
                # 리사이징이 필요하면 적용
                if self.resize_factor != 1.0:
                    image = cv2.resize(
                        image, 
                        (info['resize_width'], info['resize_height']),
                        interpolation=cv2.INTER_AREA
                    )
                
                # 이미지를 ROS 메시지로 변환
                img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = f"{self.get_safe_name(window_name)}_frame"
                
                # 이미지 발행
                info['publisher'].publish(img_msg)
                
                # 성공 카운터 증가
                info['capture_count'] += 1
                
                # FPS 계산 및 정보 업데이트
                current_time = time.time()
                info['frame_count'] += 1
                frame_time = current_time - info['last_frame_time']
                info['last_frame_time'] = current_time
                
                # 1초마다 FPS 계산 및 정보 발행
                if current_time - info['last_time'] >= 1.0:
                    elapsed = current_time - info['last_time']
                    info['fps'] = info['frame_count'] / elapsed
                    info['frame_count'] = 0
                    info['last_time'] = current_time
                    
                    # 정보 메시지 생성
                    info_msg = String()
                    
                    # 리사이징 적용 여부에 따른 메시지
                    if self.resize_factor != 1.0:
                        resize_info = f", 리사이징: 원본 {info['width']}x{info['height']} → {image.shape[1]}x{image.shape[0]}"
                    else:
                        resize_info = ""
                    
                    info_msg.data = (
                        f"창: '{info['title']}', ID: {info['window_id']}, "
                        f"크기: {image.shape[1]}x{image.shape[0]}{resize_info}, "
                        f"FPS: {info['fps']:.1f}, "
                        f"성공: {info['capture_count']}, 오류: {info['error_count']}"
                    )
                    
                    # 정보 발행
                    info['info_publisher'].publish(info_msg)
                
                # 캡처 주기에 맞게 대기
                elapsed = time.time() - start_time
                if elapsed < wait_time:
                    time.sleep(wait_time - elapsed)
            
            except Exception as e:
                info['error_count'] += 1
                self.get_logger().error(f"'{window_name}' 창 캡처/발행 오류: {str(e)}")
                time.sleep(wait_time)
    
    def capture_window_with_pipe(self, info):
        """파이프를 통해 안정적으로 창 캡처"""
        window_id = info['window_id']
        
        # 메모리 기반 임시 파일 경로 (메모리 파일시스템 사용)
        try:
            # 메모리 파일시스템에 임시 파일 생성
            mem_file = '/dev/shm/tmp_capture.png'
        except:
            # 메모리 파일시스템 사용 불가시 일반 임시 파일
            mem_file = os.path.join(tempfile.gettempdir(), f'tmp_capture_{window_id}.png')
        
        try:
            # 안정적인 방법 1: gnome-screenshot
            result = subprocess.run([
                'gnome-screenshot', '-w', '-f', mem_file
            ], timeout=0.5, capture_output=True)
            
            if result.returncode == 0 and os.path.exists(mem_file) and os.path.getsize(mem_file) > 0:
                image = cv2.imread(mem_file)
                # 임시 파일 삭제
                os.remove(mem_file)
                return image
        except:
            pass
            
        try:
            # 안정적인 방법 2: import (ImageMagick)
            result = subprocess.run([
                'import', '-silent', '-window', window_id, 'png:-'
            ], timeout=0.5, capture_output=True)
            
            if result.returncode == 0 and len(result.stdout) > 0:
                # 이미지 데이터를 메모리에서 바로 디코딩
                nparr = np.frombuffer(result.stdout, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                return image
        except:
            pass
            
        try:
            # 안정적인 방법 3: 명시적으로 파일에 저장 후 로드 (속도는 느리지만 가장 안정적)
            result = subprocess.run([
                'import', '-silent', '-window', window_id, mem_file
            ], timeout=0.5, capture_output=True)
            
            if result.returncode == 0 and os.path.exists(mem_file) and os.path.getsize(mem_file) > 0:
                image = cv2.imread(mem_file)
                # 임시 파일 삭제
                os.remove(mem_file)
                return image
        except:
            pass
        
        # 모든 방법이 실패한 경우 None 반환
        return None
    
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        self.running = False
        
        # 쓰레드 종료 대기
        for name, thread in self.window_threads.items():
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        super().destroy_node()

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        node = StableScreenCapturePublisher()
        
        # 멀티스레드 실행기
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            # 실행
            executor.spin()
        finally:
            # 종료
            executor.shutdown()
            node.destroy_node()
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"예상치 못한 오류 발생: {str(e)}")
    finally:
        # ROS2 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()
