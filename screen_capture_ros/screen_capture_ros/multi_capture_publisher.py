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
import gi

# GTK 라이브러리 설정 (옵션)
try:
    gi.require_version('Gdk', '3.0')
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gdk, Gtk, GdkPixbuf
    has_gtk = True
except:
    has_gtk = False

class MultiCapturePublisher(Node):
    def __init__(self):
        super().__init__('multi_capture_publisher')
        
        # 파라미터 선언
        self.declare_parameter('window_names', ['Map', 'Camera'])  # 기본 창 이름 목록
        self.declare_parameter('publish_rate', 15.0)  # 발행 주기 (Hz)
        self.declare_parameter('debug', True)  # 디버그 모드
        self.declare_parameter('capture_method', 'import')  # 캡처 방법: 'import', 'gdk', 'xwd'
        self.declare_parameter('list_windows', True)  # 시작 시 사용 가능한 창 목록 출력
        
        # 파라미터 가져오기
        self.window_names = self.get_parameter('window_names').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.capture_method = self.get_parameter('capture_method').value
        self.list_windows = self.get_parameter('list_windows').value
        
        # 퍼블리셔 및 창 정보 저장용 딕셔너리
        self.image_publishers = {}  # 이미지 퍼블리셔
        self.info_publishers = {}   # 정보 퍼블리셔
        self.processed_publishers = {}  # 처리된 이미지 퍼블리셔 (옵션)
        self.window_ids = {}        # 창 ID
        self.window_info = {}       # 창 정보
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 사용 가능한 창 목록 가져오기
        if self.list_windows:
            self.list_all_windows()
        
        # 각 창에 대한 퍼블리셔 생성
        for window_name in self.window_names:
            # 토픽 이름에 사용할 안전한 이름 생성
            safe_name = self.get_safe_name(window_name)
            
            # 이미지 퍼블리셔 생성
            self.image_publishers[window_name] = self.create_publisher(
                Image, 
                f'screen_capture/{safe_name}/image',
                10
            )
            
            # 처리된 이미지 퍼블리셔 생성 (그레이스케일 변환)
            self.processed_publishers[window_name] = self.create_publisher(
                Image, 
                f'screen_capture/{safe_name}/processed_image',
                10
            )
            
            # 정보 퍼블리셔 생성
            self.info_publishers[window_name] = self.create_publisher(
                String, 
                f'screen_capture/{safe_name}/info', 
                10
            )
            
            # 창 ID 초기화
            self.window_ids[window_name] = ""
            
            # 창 정보 초기화
            self.window_info[window_name] = {
                'fps': 0.0,
                'frame_count': 0,
                'last_time': time.time(),
                'last_frame_time': time.time(),
                'width': 0,
                'height': 0
            }
        
        # GTK 디스플레이 초기화 (옵션)
        if has_gtk and self.capture_method == 'gdk':
            self.display = Gdk.Display.get_default()
        
        # 타이머 설정
        self.timer = self.create_timer(1.0/self.publish_rate, self.capture_and_publish_all)
        self.window_check_timer = self.create_timer(5.0, self.find_all_window_ids)  # 주기적으로 창 ID 확인
        
        # 초기화 시 모든 창 ID 찾기
        self.find_all_window_ids()
        
        # 초기화 로그
        self.get_logger().info(
            f'MultiCapturePublisher 초기화 완료: '
            f'창 이름 목록={self.window_names} | 발행 주기={self.publish_rate}Hz | '
            f'캡처 방법={self.capture_method}'
        )
    
    def get_safe_name(self, window_name):
        """창 이름을 토픽 이름에 적합한 형식으로 변환"""
        safe_name = window_name.lower()
        safe_name = safe_name.replace(' ', '_')
        safe_name = safe_name.replace('-', '_')
        safe_name = ''.join(c for c in safe_name if c.isalnum() or c == '_')
        return safe_name
    
    def list_all_windows(self):
        """사용 가능한 모든 창 목록 출력"""
        try:
            # wmctrl로 창 목록 가져오기
            result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
            
            if result.returncode == 0:
                window_list = result.stdout.strip().split('\n')
                
                message = "=== 사용 가능한 창 목록 ===\n"
                for i, line in enumerate(window_list):
                    if line.strip():
                        parts = line.split(None, 3)
                        if len(parts) >= 4:
                            window_id, desktop, host, title = parts
                            message += f"{i+1}. ID: {window_id} | 제목: '{title}'\n"
                
                self.get_logger().info(message)
            else:
                self.get_logger().warn("wmctrl 명령 실패. 설치되어 있는지 확인하세요: sudo apt install wmctrl")
        except Exception as e:
            self.get_logger().error(f"창 목록 가져오기 오류: {str(e)}")
    
    def find_all_window_ids(self):
        """모든 창 ID 찾기"""
        try:
            # wmctrl로 창 목록 가져오기
            result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
            
            if result.returncode == 0:
                window_list = result.stdout.strip().split('\n')
                
                # 모든 창 이름에 대해 ID 찾기
                for window_name in self.window_names:
                    found = False
                    
                    for line in window_list:
                        if line.strip():
                            parts = line.split(None, 3)
                            if len(parts) >= 4:
                                window_id, desktop, host, title = parts
                                
                                # 창 이름이 제목에 포함되어 있는지 확인
                                if window_name.lower() in title.lower():
                                    self.window_ids[window_name] = window_id
                                    
                                    # 새 창 ID를 찾았을 때만 로그 출력
                                    if self.debug:
                                        self.get_logger().info(f"창 ID 찾음: {window_id} | '{title}'")
                                    
                                    found = True
                                    break
                    
                    if not found and self.debug:
                        self.get_logger().warn(f"'{window_name}' 창을 찾을 수 없습니다.")
            else:
                self.get_logger().error("wmctrl 명령 실패. 설치가 필요합니다: sudo apt install wmctrl")
        
        except Exception as e:
            self.get_logger().error(f"창 ID 찾기 오류: {str(e)}")
    
    def capture_screenshot_import(self, window_id):
        """import 명령으로 창 스크린샷 캡처"""
        try:
            if not window_id:
                return None
            
            # 임시 파일 생성
            temp_file = tempfile.mktemp(suffix='.png')
            
            # import 명령으로 스크린샷 캡처
            result = subprocess.run([
                'import', '-window', window_id, temp_file
            ], capture_output=True)
            
            if result.returncode != 0:
                if self.debug:
                    self.get_logger().warn(f"창 ID {window_id}의 스크린샷 캡처 실패 (import)")
                
                if os.path.exists(temp_file):
                    os.remove(temp_file)
                
                return None
            
            # 이미지 읽기
            image = cv2.imread(temp_file)
            
            # 임시 파일 삭제
            if os.path.exists(temp_file):
                os.remove(temp_file)
            
            return image
        
        except Exception as e:
            self.get_logger().error(f"import 스크린샷 캡처 오류: {str(e)}")
            return None
    
    def capture_screenshot_xwd(self, window_id):
        """xwd 명령으로 창 스크린샷 캡처"""
        try:
            if not window_id:
                return None
            
            # 임시 파일 경로
            temp_xwd = tempfile.mktemp(suffix='.xwd')
            temp_png = tempfile.mktemp(suffix='.png')
            
            # xwd 명령으로 스크린샷 캡처
            result = subprocess.run([
                'xwd', '-id', window_id, '-out', temp_xwd
            ], capture_output=True)
            
            if result.returncode != 0:
                if self.debug:
                    self.get_logger().warn(f"창 ID {window_id}의 스크린샷 캡처 실패 (xwd)")
                
                # 임시 파일 삭제
                for f in [temp_xwd, temp_png]:
                    if os.path.exists(f):
                        os.remove(f)
                
                return None
            
            # xwd를 png로 변환
            result = subprocess.run([
                'convert', temp_xwd, temp_png
            ], capture_output=True)
            
            if result.returncode != 0:
                if self.debug:
                    self.get_logger().warn(f"xwd -> png 변환 실패")
                
                # 임시 파일 삭제
                for f in [temp_xwd, temp_png]:
                    if os.path.exists(f):
                        os.remove(f)
                
                return None
            
            # 이미지 읽기
            image = cv2.imread(temp_png)
            
            # 임시 파일 삭제
            for f in [temp_xwd, temp_png]:
                if os.path.exists(f):
                    os.remove(f)
            
            return image
        
        except Exception as e:
            self.get_logger().error(f"xwd 스크린샷 캡처 오류: {str(e)}")
            return None
    
    def capture_screenshot_gdk(self, window_id, window_name):
        """GDK로 창 스크린샷 캡처"""
        if not has_gtk:
            return None
        
        try:
            # 창 ID가 없으면 찾을 수 없음
            if not window_id:
                return None
            
            # 모든 화면 순회
            screen = self.display.get_default_screen()
            root_window = screen.get_root_window()
            window_list = []
            
            # 모든 자식 창 열거 (재귀 함수)
            def get_all_windows(parent, window_list):
                children = parent.get_children()
                for child in children:
                    window_list.append(child)
                    get_all_windows(child, window_list)
            
            # 모든 창 조회
            get_all_windows(root_window, window_list)
            
            # 창 ID로 창 찾기
            window = None
            for w in window_list:
                try:
                    w_id = w.get_xid()
                    if str(w_id) == window_id:
                        window = w
                        break
                except:
                    continue
            
            if window is None:
                if self.debug:
                    self.get_logger().warn(f"ID가 {window_id}인 창을 GDK에서 찾을 수 없습니다.")
                return None
            
            # 창의 위치와 크기 얻기
            x, y, width, height = window.get_geometry()
            
            # 지정된 영역의 스크린샷 캡처
            pixbuf = Gdk.pixbuf_get_from_window(root_window, x, y, width, height)
            
            if pixbuf is None:
                if self.debug:
                    self.get_logger().warn(f"GDK pixbuf 생성 실패")
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
            self.get_logger().error(f"GDK 스크린샷 캡처 오류: {str(e)}")
            return None
    
    def capture_screenshot(self, window_id, window_name):
        """선택한 방법으로 스크린샷 캡처"""
        # 창 ID가 없으면 캡처 불가
        if not window_id:
            return None
        
        # 캡처 방법에 따라 적절한 함수 호출
        if self.capture_method == 'import':
            return self.capture_screenshot_import(window_id)
        elif self.capture_method == 'xwd':
            return self.capture_screenshot_xwd(window_id)
        elif self.capture_method == 'gdk' and has_gtk:
            return self.capture_screenshot_gdk(window_id, window_name)
        else:
            # 기본값은 import 사용
            return self.capture_screenshot_import(window_id)
    
    def process_image(self, image):
        """이미지 처리 예시 (그레이스케일 변환)"""
        if image is None:
            return None
        
        try:
            # 그레이스케일 변환
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 다시 BGR로 변환 (ROS는 BGR 이미지를 기대)
            processed = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            
            return processed
        except Exception as e:
            self.get_logger().error(f"이미지 처리 오류: {str(e)}")
            return None
    
    def update_fps(self, window_name):
        """FPS 계산 업데이트"""
        info = self.window_info[window_name]
        current_time = time.time()
        
        # 프레임 카운터 증가
        info['frame_count'] += 1
        
        # 1초마다 FPS 계산
        if current_time - info['last_time'] >= 1.0:
            elapsed = current_time - info['last_time']
            info['fps'] = info['frame_count'] / elapsed
            info['frame_count'] = 0
            info['last_time'] = current_time
        
        # 프레임 간 시간 계산
        frame_time = current_time - info['last_frame_time']
        info['last_frame_time'] = current_time
        
        return info['fps'], frame_time
    
    def capture_and_publish_all(self):
        """모든 창 캡처하고 발행"""
        # 모든 창에 대해 반복
        for window_name in self.window_names:
            window_id = self.window_ids.get(window_name, "")
            
            # ID가 없으면 다음 창으로
            if not window_id:
                continue
            
            # 스크린샷 캡처
            image = self.capture_screenshot(window_id, window_name)
            
            if image is None:
                continue
            
            try:
                # 이미지 크기 가져오기
                h, w = image.shape[:2]
                self.window_info[window_name]['width'] = w
                self.window_info[window_name]['height'] = h
                
                # FPS 업데이트
                fps, frame_time = self.update_fps(window_name)
                
                # 토픽 이름에 사용할 안전한 이름 생성
                safe_name = self.get_safe_name(window_name)
                
                # 원본 이미지를 ROS 메시지로 변환
                raw_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                raw_msg.header.frame_id = f"{safe_name}_frame"
                
                # 원본 이미지 발행
                self.image_publishers[window_name].publish(raw_msg)
                
                # 처리된 이미지 생성 및 발행
                processed_image = self.process_image(image)
                if processed_image is not None:
                    proc_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                    proc_msg.header.stamp = raw_msg.header.stamp
                    proc_msg.header.frame_id = f"{safe_name}_processed_frame"
                    self.processed_publishers[window_name].publish(proc_msg)
                
                # 정보 메시지 생성
                info_msg = String()
                info_msg.data = (
                    f"Window: {window_name}, ID: {window_id}, "
                    f"Size: {w}x{h}, FPS: {fps:.1f}, "
                    f"Frame time: {frame_time*1000:.1f}ms"
                )
                
                # 정보 메시지 발행
                self.info_publishers[window_name].publish(info_msg)
                
            except Exception as e:
                self.get_logger().error(f"{window_name} 이미지 발행 중 오류 발생: {str(e)}")

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        publisher = MultiCapturePublisher()
        
        # 노드 실행
        rclpy.spin(publisher)
    
    except KeyboardInterrupt:
        pass
    
    except Exception as e:
        print(f"예상치 못한 오류 발생: {str(e)}")
    
    finally:
        # ROS2 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()
