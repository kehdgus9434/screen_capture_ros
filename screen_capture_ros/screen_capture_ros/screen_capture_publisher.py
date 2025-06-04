#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import gi
import subprocess
import re

# GTK 라이브러리 설정
gi.require_version('Gdk', '3.0')
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GdkPixbuf

class ScreenCapturePublisher(Node):
    def __init__(self):
        super().__init__('screen_capture_publisher')
        
        # 파라미터 선언 - 찾을 창 이름의 일부분으로 설정
        self.declare_parameter('window_title_part', 'S711N')  # SM-S711N의 일부
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('debug', True)  # 디버그 모드 활성화
        self.declare_parameter('list_windows', True)  # 시작 시 창 목록 출력
        
        # 파라미터 가져오기
        self.window_title_part = self.get_parameter('window_title_part').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.list_windows = self.get_parameter('list_windows').value
        
        # Image 토픽 발행자
        self.publisher = self.create_publisher(
            Image, 
            'screen_capture/image', 
            10
        )
        
        # 디버그용 정보 토픽
        self.info_publisher = self.create_publisher(
            String, 
            'screen_capture/info', 
            10
        )
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 디스플레이 초기화
        self.display = Gdk.Display.get_default()
        
        # 창 검색 관련 변수
        self.window = None
        self.window_title = None
        self.search_attempts = 0
        self.max_attempts = 10
        self.window_found = False
        
        # 시스템 환경 파악
        is_wayland = 'wayland' in subprocess.getoutput('echo $XDG_SESSION_TYPE').lower()
        if is_wayland:
            self.get_logger().warn('Wayland 세션 감지됨. X11에서 더 잘 동작합니다.')
        
        # 창 목록 표시 (초기에만)
        if self.list_windows:
            self.list_all_windows()
        
        # 타이머 설정
        self.timer = self.create_timer(1.0/self.publish_rate, self.capture_and_publish)
        self.window_check_timer = self.create_timer(5.0, self.list_all_windows)  # 주기적으로 창 목록 업데이트
        
        # 초기화 로그
        self.get_logger().info(
            f'ScreenCapturePublisher 초기화 완료: '
            f'창 제목 포함="{self.window_title_part}" | 발행 주기={self.publish_rate}Hz'
        )
    
    def list_all_windows(self):
        """시스템에서 사용 가능한 모든 창 목록을 출력"""
        try:
            # wmctrl로 창 목록 가져오기 - 가장 신뢰성 높은 방법
            try:
                result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
                if result.returncode == 0:
                    window_list = result.stdout.strip().split('\n')
                    
                    message = "=== 사용 가능한 창 목록 (wmctrl) ===\n"
                    scrcpy_windows = []
                    
                    for line in window_list:
                        if line.strip():
                            parts = line.split(None, 3)
                            if len(parts) >= 4:
                                window_id, desktop, host, title = parts
                                message += f"- '{title}'\n"
                                
                                # scrcpy 관련 창 강조
                                if 'scrcpy' in title.lower() or 'sm-' in title.lower():
                                    scrcpy_windows.append(title)
                    
                    if scrcpy_windows:
                        message += "\n=== scrcpy 관련 창 (이 중 하나로 시도) ===\n"
                        for i, title in enumerate(scrcpy_windows):
                            message += f"{i+1}. '{title}'\n"
                            # 첫 번째 scrcpy 창 선택
                            if not self.window_found and i == 0:
                                self.window_title_part = title
                                self.get_logger().info(f"자동으로 창 선택: '{title}'")
                    
                    # 정보 발행
                    msg = String()
                    msg.data = message
                    self.info_publisher.publish(msg)
                    
                    if self.debug:
                        self.get_logger().info(message)
                    
                    return
            except FileNotFoundError:
                self.get_logger().warn("wmctrl이 설치되어 있지 않습니다. 'sudo apt install wmctrl'로 설치하세요.")
            
            # xwininfo 사용 - 대안
            try:
                result = subprocess.run(['xwininfo', '-tree', '-root'], capture_output=True, text=True)
                if result.returncode == 0:
                    # scrcpy 관련 창 찾기
                    scrcpy_windows = []
                    for line in result.stdout.split('\n'):
                        if 'scrcpy' in line.lower() or 'sm-' in line.lower():
                            # 창 제목 추출 시도
                            match = re.search(r'"([^"]+)"', line)
                            if match:
                                title = match.group(1)
                                scrcpy_windows.append(title)
                    
                    if scrcpy_windows:
                        message = "=== xwininfo로 찾은 scrcpy 관련 창 ===\n"
                        for i, title in enumerate(scrcpy_windows):
                            message += f"{i+1}. '{title}'\n"
                            # 첫 번째 scrcpy 창 선택
                            if not self.window_found and i == 0:
                                self.window_title_part = title
                                self.get_logger().info(f"자동으로 창 선택: '{title}'")
                        
                        # 정보 발행
                        msg = String()
                        msg.data = message
                        self.info_publisher.publish(msg)
                        
                        if self.debug:
                            self.get_logger().info(message)
            except Exception as e:
                self.get_logger().error(f"xwininfo 오류: {str(e)}")
            
        except Exception as e:
            self.get_logger().error(f'창 목록 가져오기 중 오류 발생: {str(e)}')
    
    def find_window_by_title(self):
        """이름의 일부분으로 창 찾기"""
        try:
            # 모든 화면 순회
            screen = self.display.get_default_screen()
            root_window = screen.get_root_window()
            
            # 열려있는 모든 창 목록
            window_list = []
            
            # 모든 자식 창 열거 (재귀 함수)
            def get_all_windows(parent, window_list):
                children = parent.get_children()
                for child in children:
                    window_list.append(child)
                    get_all_windows(child, window_list)
            
            # 모든 창 조회
            get_all_windows(root_window, window_list)
            
            # 제목으로 scrcpy 창 찾기 - 부분 일치 검색
            for window in window_list:
                try:
                    win_name = window.get_property("WM_NAME")
                    
                    # 디버그 모드일 경우 모든 창 이름 출력
                    if self.debug and win_name:
                        self.get_logger().debug(f'검사 중: "{win_name}"')
                    
                    # 부분 일치하는 창 찾기
                    if win_name and self.window_title_part.lower() in win_name.lower():
                        self.window_found = True
                        self.search_attempts = 0
                        self.window_title = win_name
                        if self.debug:
                            self.get_logger().info(f'창을 찾았습니다: "{win_name}" (부분 일치: {self.window_title_part})')
                        return window
                except Exception as e:
                    if self.debug:
                        self.get_logger().debug(f'창 속성 읽기 오류: {str(e)}')
                    continue
            
            # 정확히 scrcpy 프로세스인 창 찾기 (WM_CLASS 속성 검사)
            for window in window_list:
                try:
                    wm_class = window.get_property("WM_CLASS")
                    win_name = window.get_property("WM_NAME")
                    
                    if wm_class and 'scrcpy' in str(wm_class).lower() and win_name:
                        self.window_found = True
                        self.search_attempts = 0
                        self.window_title = win_name
                        if self.debug:
                            self.get_logger().info(f'창을 찾았습니다: "{win_name}" (scrcpy 클래스)')
                        return window
                except:
                    continue
        
        except Exception as e:
            self.get_logger().error(f'창 검색 중 오류 발생: {str(e)}')
        
        # 창을 찾지 못한 경우
        self.search_attempts += 1
        if self.search_attempts >= self.max_attempts:
            if not self.window_found:  # 한 번도 창을 찾지 못한 경우만 경고
                self.get_logger().warn(
                    f'"{self.window_title_part}" 제목을 포함하는 창을 {self.max_attempts}회 시도했으나 찾지 못했습니다. '
                    f'scrcpy가 실행 중인지 확인하거나 window_title_part 파라미터를 확인하세요.'
                )
                # 검색 시도 횟수 리셋
                self.search_attempts = 0
        
        return None
    
    def capture_window_screenshot(self, window):
        """지정된 창의 스크린샷 캡처"""
        try:
            # 창의 위치와 크기 얻기
            x, y, width, height = window.get_geometry()
            
            # 화면 전체를 덮는 루트 창 얻기
            root_window = Gdk.get_default_root_window()
            
            # 지정된 영역의 스크린샷 캡처
            pixbuf = Gdk.pixbuf_get_from_window(root_window, x, y, width, height)
            
            if pixbuf is None:
                self.get_logger().warn('스크린샷 캡처 실패: pixbuf가 None입니다')
                return None
            
            # Pixbuf -> OpenCV 이미지(numpy 배열) 변환
            width = pixbuf.get_width()
            height = pixbuf.get_height()
            channels = pixbuf.get_n_channels()
            
            # Pixbuf 데이터를 직접 numpy 배열로 변환
            pixel_bytes = pixbuf.get_pixels()
            rowstride = pixbuf.get_rowstride()
            
            # numpy 배열로 변환 (기본적으로 RGB 형식)
            image = np.frombuffer(pixel_bytes, dtype=np.uint8)
            image = image.reshape((height, rowstride // channels, channels))
            image = image[:, :width, :]  # 올바른 너비로 자르기
            
            # RGB -> BGR 변환 (OpenCV 형식)
            if channels == 3:  # RGB인 경우만
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            return image
            
        except Exception as e:
            self.get_logger().error(f'스크린샷 캡처 중 오류 발생: {str(e)}')
            return None
    
    def capture_and_publish(self):
        """창을 찾고, 캡처하고, 발행하는 메인 함수"""
        # scrcpy 창 찾기
        window = self.find_window_by_title()
        
        if window is None:
            return  # 창을 찾지 못함
        
        # 스크린샷 캡처
        screenshot = self.capture_window_screenshot(window)
        
        if screenshot is None:
            return  # 스크린샷 캡처 실패
        
        try:
            # OpenCV 이미지를 ROS2 이미지 메시지로 변환
            img_msg = self.bridge.cv2_to_imgmsg(screenshot, "bgr8")
            
            # 타임스탬프와 프레임 ID 설정
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            
            # 이미지 토픽 발행
            self.publisher.publish(img_msg)
            
            # 정보 토픽 발행
            h, w = screenshot.shape[:2]
            info_msg = String()
            info_msg.data = f"Window: {self.window_title}, Size: {w}x{h}, FPS: {self.publish_rate:.1f}"
            self.info_publisher.publish(info_msg)
            
        except Exception as e:
            self.get_logger().error(f'이미지 발행 중 오류 발생: {str(e)}')

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        publisher = ScreenCapturePublisher()
        
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
