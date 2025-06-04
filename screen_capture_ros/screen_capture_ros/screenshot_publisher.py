#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import tempfile
import os
import time

class ScreenshotPublisher(Node):
    def __init__(self):
        super().__init__('screenshot_publisher')
        
        # 파라미터 선언
        self.declare_parameter('window_id', '')  # 창 ID (wmctrl -l로 찾을 수 있음)
        self.declare_parameter('window_name', 'SM-S711N')  # 창 이름 (자동으로 ID를 찾을 때 사용)
        self.declare_parameter('publish_rate', 15.0)  # 발행 주기 (Hz)
        self.declare_parameter('debug', True)  # 디버그 모드
        
        # 파라미터 가져오기
        self.window_id = self.get_parameter('window_id').value
        self.window_name = self.get_parameter('window_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        
        # Image 토픽 발행자
        self.publisher = self.create_publisher(
            Image, 
            'screen_capture/image', 
            10
        )
        
        # 정보 토픽 발행자
        self.info_publisher = self.create_publisher(
            String, 
            'screen_capture/info', 
            10
        )
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 임시 파일 경로
        self.temp_dir = tempfile.gettempdir()
        self.temp_file = os.path.join(self.temp_dir, "scrcpy_capture.png")
        
        # 창 ID 없으면 찾기
        if not self.window_id:
            self.find_window_id()
            
        # 타이머 설정
        self.timer = self.create_timer(1.0/self.publish_rate, self.capture_and_publish)
        self.window_check_timer = self.create_timer(5.0, self.find_window_id)  # 주기적으로 창 ID 확인
        
        # 초기화 로그
        self.get_logger().info(
            f'ScreenshotPublisher 초기화 완료: '
            f'창 이름="{self.window_name}" | 발행 주기={self.publish_rate}Hz'
        )
    
    def find_window_id(self):
        """창 이름으로 창 ID 찾기"""
        try:
            # wmctrl로 창 목록 가져오기
            result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error("wmctrl 명령 실패. 설치되어 있는지 확인하세요: sudo apt install wmctrl")
                return
            
            window_list = result.stdout.strip().split('\n')
            found_windows = []
            
            # 모든 창의 ID와 이름 출력 (디버그 모드)
            if self.debug:
                message = "=== 사용 가능한 창 목록 ===\n"
                
                for line in window_list:
                    if line.strip():
                        parts = line.split(None, 3)
                        if len(parts) >= 4:
                            window_id, desktop, host, title = parts
                            message += f"ID: {window_id} | 제목: '{title}'\n"
                            
                            # scrcpy 또는 SM-S711N 관련 창 체크
                            if 'scrcpy' in title.lower() or 'sm-' in title.lower():
                                found_windows.append((window_id, title))
                
                self.get_logger().info(message)
            
            # 이름으로 창 ID 찾기
            for line in window_list:
                if line.strip():
                    parts = line.split(None, 3)
                    if len(parts) >= 4:
                        window_id, desktop, host, title = parts
                        if self.window_name.lower() in title.lower():
                            self.window_id = window_id
                            self.get_logger().info(f"창 ID 찾음: {window_id} | '{title}'")
                            return
            
            # 정확한 이름 못 찾았지만 scrcpy 관련 창 발견
            if found_windows:
                self.window_id = found_windows[0][0]
                self.window_name = found_windows[0][1]
                self.get_logger().info(f"관련 창 ID 찾음: {self.window_id} | '{self.window_name}'")
            else:
                self.get_logger().warn(f"'{self.window_name}' 창을 찾을 수 없습니다.")
                self.window_id = ""
        
        except Exception as e:
            self.get_logger().error(f"창 ID 찾기 오류: {str(e)}")
            self.window_id = ""
    
    def capture_and_publish(self):
        """스크린샷 캡처 및 발행"""
        if not self.window_id:
            return  # 창 ID가 없으면 캡처 불가
        
        try:
            # 방법 1: import 사용 (Wayland에서도 작동)
            result = subprocess.run([
                'import', '-window', self.window_id, self.temp_file
            ], capture_output=True, text=True)
            
            if result.returncode != 0:
                # 방법 2: xwd 시도
                result = subprocess.run([
                    'xwd', '-id', self.window_id, '-out', self.temp_file + '.xwd'
                ], capture_output=True, text=True)
                
                if result.returncode == 0:
                    # xwd 파일을 PNG로 변환
                    subprocess.run([
                        'convert', self.temp_file + '.xwd', self.temp_file
                    ])
                else:
                    # 방법 3: gnome-screenshot 시도
                    result = subprocess.run([
                        'gnome-screenshot', '-w', '-f', self.temp_file
                    ], capture_output=True, text=True)
                    
                    if result.returncode != 0:
                        self.get_logger().warn("스크린샷 캡처 실패. 다른 도구 설치 필요: sudo apt install imagemagick scrot")
                        return
            
            # 이미지 읽기
            if not os.path.exists(self.temp_file):
                self.get_logger().warn(f"캡처된 이미지 파일이 존재하지 않음: {self.temp_file}")
                return
                
            image = cv2.imread(self.temp_file)
            
            if image is None:
                self.get_logger().warn("이미지 파일을 읽을 수 없습니다.")
                return
            
            # OpenCV 이미지를 ROS2 이미지 메시지로 변환
            img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            
            # 타임스탬프와 프레임 ID 설정
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            
            # 이미지 토픽 발행
            self.publisher.publish(img_msg)
            
            # 정보 발행
            h, w =  image.shape[:2]
            info_msg = String()
            info_msg.data = f"Window: {self.window_name}, ID: {self.window_id}, Size: {w}x{h}, FPS: {self.publish_rate:.1f}"
            self.info_publisher.publish(info_msg)
            
            # 임시 파일 삭제
            try:
                os.remove(self.temp_file)
                if os.path.exists(self.temp_file + '.xwd'):
                    os.remove(self.temp_file + '.xwd')
            except:
                pass
            
        except Exception as e:
            self.get_logger().error(f"스크린샷 캡처 중 오류 발생: {str(e)}")

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        publisher = ScreenshotPublisher()
        
        # 노드 실행
        rclpy.spin(publisher)
    
    except KeyboardInterrupt:
        pass
    
    except Exception as e:
        print(f"예상치 못한 오류 발생: {str(e)}")
    
    finally:
        # 노드 종료 및 ROS2 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()
