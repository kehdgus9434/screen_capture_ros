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
import gi
import tempfile

# GTK3 라이브러리 설정
gi.require_version('Gdk', '3.0')
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GdkPixbuf, GLib

class DirectMemoryPublisher(Node):
    def __init__(self):
        super().__init__('direct_memory_publisher')
        
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
        
        # GTK 초기화 확인
        self.display = Gdk.Display.get_default()
        if self.display is None:
            self.get_logger().error("X11 디스플레이에 접근할 수 없습니다!")
        
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
                'method': 'gtk', # 기본 캡처 방식
                'capture_success_count': 0
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
        
        # 타이머 생성 (창 찾기)
        self.find_window_timer = self.create_timer(5.0, self.find_all_windows)
        
        # 창 찾기
        self.find_all_windows()
        
        # 캡처 쓰레드 시작
        self.start_capture_threads()
        
        self.get_logger().info(
            f'DirectMemoryPublisher 초기화 완료: '
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
        
        # 캡처 방법 목록 - 하나가 실패하면 다른 방법 시도
        capture_methods = ['gtk', 'xpixmap', 'rgb_data']
        method_index = 0
        
        while rclpy.ok() and self.running:
            start_time = time.time()
            
            # 창 ID가 없으면 다음 반복으로
            if info['window_id'] is None:
                time.sleep(wait_time)
                continue
            
            try:
                # 메모리에서 직접 창 캡처
                with lock:
                    # 현재 설정된 캡처 방법 사용
                    method = info['method']
                    image = self.capture_window_direct(info, method)
                    
                    # 캡처 실패 시 다른 방법 시도
                    if image is None:
                        # 현재 방법이 3번 연속 실패하면 다음 방법으로 전환
                        method_index = (method_index + 1) % len(capture_methods)
                        info['method'] = capture_methods[method_index]
                        self.get_logger().warn(
                            f"'{window_name}' 창 캡처 방법 변경: {method} → {info['method']}"
                        )
                        time.sleep(wait_time)
                        continue
                    
                    # 캡처 성공 카운터 증가
                    info['capture_success_count'] += 1
                
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
                        f"FPS: {info['fps']:.1f}, 방식: {info['method']}, "
                        f"성공 프레임: {info['capture_success_count']}"
                    )
                    
                    # 정보 발행
                    info['info_publisher'].publish(info_msg)
                
                # 캡처 주기에 맞게 대기
                elapsed = time.time() - start_time
                if elapsed < wait_time:
                    time.sleep(wait_time - elapsed)
            
            except Exception as e:
                self.get_logger().error(f"'{window_name}' 창 캡처/발행 오류: {str(e)}")
                time.sleep(wait_time)
    
    def capture_window_direct(self, info, method='gtk'):
        """여러 방법으로 메모리에서 직접 창 캡처"""
        window_id = info['window_id']
        x = info['x']
        y = info['y']
        width = info['width']
        height = info['height']
        
        try:
            if method == 'gtk':
                # GTK/GDK 방식으로 캡처 (가장 빠름)
                display = Gdk.Display.get_default()
                root_window = Gdk.get_default_root_window()
                
                # 스크린샷 캡처
                pixbuf = GdkPixbuf.Pixbuf.get_from_window(root_window, x, y, width, height)
                
                if pixbuf is None:
                    return None
                
                # Pixbuf -> Numpy 배열로 변환
                channels = pixbuf.get_n_channels()
                rowstride = pixbuf.get_rowstride()
                pixels = pixbuf.get_pixels()
                
                # Numpy 배열로 변환
                image = np.frombuffer(pixels, dtype=np.uint8)
                image = image.reshape((height, rowstride // channels, channels))
                image = image[:, :width, :]
                
                # RGB -> BGR로 변환 (OpenCV 형식)
                if channels == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                return image
            
            elif method == 'xpixmap':
                # XPixmap을 사용한 캡처
                from Xlib import X, display as xdisplay
                
                # X 디스플레이 연결
                try:
                    xdisp = xdisplay.Display()
                    screen = xdisp.screen()
                    root = screen.root
                    
                    # 캡처할 창이 루트 창이 아니면 찾기
                    if window_id.lower() != "0x0":
                        target_window = xdisp.create_resource_object('window', int(window_id, 16))
                    else:
                        target_window = root
                    
                    # 창의 위치와 크기 얻기
                    geom = target_window.get_geometry()
                    x, y, width, height = geom.x, geom.y, geom.width, geom.height
                    
                    # 스크린 좌표로 변환
                    abs_x, abs_y = 0, 0
                    parent = target_window.query_tree().parent
                    while parent and parent.id != root.id:
                        parent_geom = parent.get_geometry()
                        abs_x += parent_geom.x
                        abs_y += parent_geom.y
                        parent = parent.query_tree().parent
                    
                    x += abs_x
                    y += abs_y
                    
                    # 이미지 캡처
                    raw = root.get_image(x, y, width, height, X.ZPixmap, 0xffffffff)
                    data = raw.data
                    
                    # PIL 이미지로 변환
                    from PIL import Image
                    pil_image = Image.frombytes("RGB", (width, height), data, "raw", "BGRX")
                    
                    # OpenCV 이미지로 변환
                    image = np.array(pil_image)
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    
                    return image
                except ImportError:
                    self.get_logger().warn("python-xlib이 설치되지 않았습니다. 다른 방식으로 시도합니다.")
                    return None
            
            elif method == 'rgb_data':
                # RGB 데이터를 직접 얻는 방식
                try:
                    # Pillow와 ImageTk로 시도
                    import PIL.ImageGrab
                    
                    # 전체 화면 캡처
                    screen = PIL.ImageGrab.grab(bbox=(x, y, x + width, y + height))
                    
                    # OpenCV 이미지로 변환
                    image = np.array(screen)
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    return image
                except ImportError:
                    self.get_logger().warn("PIL.ImageGrab이 설치되지 않았습니다.")
                    return None
            
            # 모든 방법 실패 시 마지막 수단으로 subprocess + in-memory pipe 사용
            # (임시 파일 없이 파이프를 통해 데이터 전달)
            import io
            
            # xwd 명령어 실행
            xwd_process = subprocess.Popen(
                ['xwd', '-silent', '-id', window_id],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            
            # xwd 출력을 convert에 파이프로 전달
            convert_process = subprocess.Popen(
                ['convert', '-', '-quality', '100', 'PNG:-'],
                stdin=xwd_process.stdout,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            xwd_process.stdout.close()
            
            # 변환된 이미지 데이터 가져오기
            png_data = convert_process.communicate()[0]
            
            # 이미지 데이터를 메모리에서 디코딩
            nparr = np.frombuffer(png_data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            return image
        
        except Exception as e:
            self.get_logger().error(f"직접 캡처 오류 ({method}): {str(e)}")
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
        node = DirectMemoryPublisher()
        
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
