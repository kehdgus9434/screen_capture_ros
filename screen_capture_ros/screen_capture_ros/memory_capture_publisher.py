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
import io
import sys
import traceback

class MemoryCapturePublisher(Node):
    def __init__(self):
        super().__init__('memory_capture_publisher')
        
        # 파라미터 선언
        self.declare_parameter('window_names', ['Camera', 'Map'])  # 캡처할 창 이름
        self.declare_parameter('publish_rate', 15.0)  # 발행 주기 (Hz)
        self.declare_parameter('debug', True)  # 디버그 메시지
        self.declare_parameter('resize_factor', 1.0)  # 리사이징 비율
        self.declare_parameter('capture_methods', ['ffmpeg', 'import', 'xwd', 'scrot'])  # 사용할 캡처 방법 순서
        self.declare_parameter('timeout', 0.5)  # 캡처 시간 제한 (초)
        
        # 파라미터 가져오기
        self.window_names = self.get_parameter('window_names').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.resize_factor = self.get_parameter('resize_factor').value
        self.capture_methods = self.get_parameter('capture_methods').value
        self.timeout = self.get_parameter('timeout').value
        
        # 창 정보와 쓰레드 저장
        self.window_info = {}
        self.window_threads = {}
        self.thread_locks = {}
        self.running = True
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 캡처 방법 성공률 관리
        self.method_stats = {
            'import': {'success': 0, 'fail': 0, 'last_error': None},
            'xwd': {'success': 0, 'fail': 0, 'last_error': None},
            'scrot': {'success': 0, 'fail': 0, 'last_error': None},
            'ffmpeg': {'success': 0, 'fail': 0, 'last_error': None}
        }
        
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
                'capture_count': 0,
                'error_count': 0,
                'current_method': self.capture_methods[0] if self.capture_methods else 'import',
                'method_change_time': time.time(),
                'consecutive_errors': 0,
                'consecutive_errors_method': {}
            }
            
            # 각 방법에 대한 에러 카운터 초기화
            for method in self.capture_methods:
                self.window_info[window_name]['consecutive_errors_method'][method] = 0
            
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
            
            # 진단 퍼블리셔 (상세 오류 정보)
            self.window_info[window_name]['diag_publisher'] = self.create_publisher(
                String, f'screen_capture/{safe_name}/diagnostics', 10
            )
            
            # 쓰레드 락
            self.thread_locks[window_name] = threading.Lock()
        
        # 타이머 생성
        self.find_window_timer = self.create_timer(5.0, self.find_all_windows)
        self.diagnostics_timer = self.create_timer(10.0, self.publish_global_diagnostics)
        
        # 창 찾기
        self.find_all_windows()
        
        # 필요한 툴 검사
        self.check_required_tools()
        
        # 캡처 쓰레드 시작
        self.start_capture_threads()
        
        self.get_logger().info(
            f'MemoryCapturePublisher 초기화 완료: '
            f'창 이름 목록={self.window_names} | '
            f'발행 주기={self.publish_rate}Hz | '
            f'리사이징 비율={self.resize_factor} | '
            f'캡처 방법={self.capture_methods}'
        )
    
    def check_required_tools(self):
        """필요한 외부 도구 존재 확인"""
        tools = {
            'wmctrl': False,
            'import': False,
            'xwd': False,
            'convert': False,
            'scrot': False,
            'ffmpeg': False
        }
        
        # 각 도구 확인
        for tool in tools:
            try:
                result = subprocess.run(['which', tool], capture_output=True, text=True)
                tools[tool] = result.returncode == 0
            except:
                tools[tool] = False
        
        # 가용 방법 로그 출력
        available_methods = []
        missing_tools = []
        
        # 방법별 필요 도구
        if tools['import']:
            available_methods.append('import')
        else:
            missing_tools.append('import (from ImageMagick)')
        
        if tools['xwd'] and tools['convert']:
            available_methods.append('xwd')
        else:
            missing_tools.extend([t for t in ['xwd', 'convert'] if not tools[t]])
        
        if tools['scrot']:
            available_methods.append('scrot')
        else:
            missing_tools.append('scrot')
        
        if tools['ffmpeg']:
            available_methods.append('ffmpeg')
        else:
            missing_tools.append('ffmpeg')
        
        # 결과 로그
        if available_methods:
            self.get_logger().info(f"사용 가능한 캡처 방법: {', '.join(available_methods)}")
        else:
            self.get_logger().error("사용 가능한 캡처 방법이 없습니다!")
            
        if missing_tools:
            self.get_logger().warn(
                f"일부 도구를 찾을 수 없습니다: {', '.join(missing_tools)}. "
                "모든 기능을 사용하려면 설치하세요: sudo apt install imagemagick x11-utils scrot ffmpeg")
        
        # 캡처 방법 필터링
        self.capture_methods = [m for m in self.capture_methods if m in available_methods]
        
        if not self.capture_methods:
            self.get_logger().error("사용 가능한 캡처 방법이 없습니다. 적어도 하나의 방법이 필요합니다!")
            # 기본값 설정
            if 'import' in available_methods:
                self.capture_methods = ['import']
            else:
                self.capture_methods = list(available_methods)
    
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
                error_msg = f"wmctrl 명령 실패 (코드 {result.returncode}): {result.stderr}"
                self.get_logger().error(error_msg)
                self.publish_diagnostics("system", f"창 목록 가져오기 실패: {error_msg}")
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
                                
                                # 진단 정보 발행
                                self.publish_diagnostics(
                                    window_name, 
                                    f"창 정보 업데이트: ID={window_id}, 위치=({x}, {y}), 크기={width}x{height}"
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
                if window_name not in found_windows:
                    self.get_logger().warn(f"'{window_name}' 창을 찾을 수 없습니다.")
                    self.publish_diagnostics(
                        window_name, 
                        f"창을 찾을 수 없음: '{window_name}'. "
                        f"현재 활성 창 목록: {[parts[8] for parts in [line.split(None, 8) for line in window_list] if len(parts) >= 9][:5]}"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"창 찾기 오류: {str(e)}")
            self.publish_diagnostics("system", f"창 찾기 오류: {str(e)}\n{traceback.format_exc()}")
    
    def publish_diagnostics(self, window_name, message):
        """진단 정보를 발행"""
        try:
            # 타임스탬프 추가
            timestamp = self.get_clock().now().to_msg().sec
            formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))
            
            # 진단 메시지 생성
            diag_msg = String()
            diag_msg.data = f"[{formatted_time}] {message}"
            
            # 해당 창의 진단 토픽에 발행
            if window_name in self.window_info:
                self.window_info[window_name]['diag_publisher'].publish(diag_msg)
            elif window_name == "system":
                # 시스템 진단은 모든 창에 발행
                for info in self.window_info.values():
                    info['diag_publisher'].publish(diag_msg)
        except Exception as e:
            self.get_logger().error(f"진단 정보 발행 오류: {str(e)}")
    
    def publish_global_diagnostics(self):
        """전역 진단 정보 주기적 발행"""
        try:
            # 캡처 방법 통계
            stats = []
            for method, data in self.method_stats.items():
                total = data['success'] + data['fail']
                success_rate = (data['success'] / total * 100) if total > 0 else 0
                last_error = data['last_error'] or "없음"
                if len(last_error) > 100:  # 에러 메시지 길이 제한
                    last_error = last_error[:100] + "..."
                
                stats.append(
                    f"{method}: 성공률={success_rate:.1f}% ({data['success']}/{total}), "
                    f"마지막 오류={last_error}"
                )
            
            # 진단 메시지 생성
            message = (
                f"캡처 방법 통계:\n" + "\n".join(stats) + "\n\n"
                f"활성 쓰레드: {sum(1 for t in self.window_threads.values() if t and t.is_alive())}/{len(self.window_names)}"
            )
            
            self.publish_diagnostics("system", message)
        except Exception as e:
            self.get_logger().error(f"전역 진단 발행 오류: {str(e)}")
    
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
            
            self.publish_diagnostics(window_name, f"캡처 쓰레드 시작: {thread.name}")
    
    def capture_and_publish_thread(self, window_name):
        """창 캡처 및 발행 쓰레드"""
        info = self.window_info[window_name]
        lock = self.thread_locks[window_name]
        wait_time = 1.0 / self.publish_rate
        
        # 쓰레드 시작 알림
        thread_id = threading.get_ident()
        self.get_logger().info(f"'{window_name}' 캡처 쓰레드 시작 (ID: {thread_id})")
        
        while rclpy.ok() and self.running:
            start_time = time.time()
            
            # 창 ID가 없으면 다음 반복으로
            if info['window_id'] is None:
                time.sleep(wait_time)
                continue
            
            try:
                # 현재 캡처 방법 가져오기
                current_method = info['current_method']
                
                # 메모리에서 직접 창 캡처
                with lock:
                    # 이미지 캡처 시도
                    image, method_used, error_details = self.capture_with_method(
                        info['window_id'], 
                        info['x'], info['y'], 
                        info['width'], info['height'],
                        current_method
                    )
                    
                # 캡처 결과 처리
                if image is None:
                    # 캡처 실패 카운트 증가
                    info['error_count'] += 1
                    info['consecutive_errors'] += 1
                    
                    # 특정 방법의 연속 오류 카운트 증가
                    if method_used in info['consecutive_errors_method']:
                        info['consecutive_errors_method'][method_used] += 1
                    
                    # 연속 오류가 3회 이상이면 캡처 방법 변경
                    if info['consecutive_errors_method'][method_used] >= 3:
                        self.switch_capture_method(window_name, method_used, error_details)
                    
                    time.sleep(wait_time)
                    continue
                
                # 캡처 성공 - 연속 오류 카운터 리셋
                info['consecutive_errors'] = 0
                info['consecutive_errors_method'][method_used] = 0
                
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
                        f"방법: {method_used}, FPS: {info['fps']:.1f}, "
                        f"성공: {info['capture_count']}, 오류: {info['error_count']}"
                    )
                    
                    # 정보 발행
                    info['info_publisher'].publish(info_msg)
                
                # 캡처 주기에 맞게 대기
                elapsed = time.time() - start_time
                if elapsed < wait_time:
                    time.sleep(wait_time - elapsed)
            
            except Exception as e:
                # 오류 발생 시
                info['error_count'] += 1
                error_msg = f"'{window_name}' 창 캡처/발행 오류: {str(e)}"
                self.get_logger().error(error_msg)
                
                # 상세 오류 추적 정보
                tb = traceback.format_exc()
                self.publish_diagnostics(window_name, f"캡처 오류: {str(e)}\n{tb}")
                
                # 오류 후 약간의 대기
                time.sleep(wait_time)
        
        # 쓰레드 종료 알림
        self.get_logger().info(f"'{window_name}' 캡처 쓰레드 종료 (ID: {thread_id})")
    
    def switch_capture_method(self, window_name, failed_method, error_details):
        """캡처 방법 변경"""
        info = self.window_info[window_name]
        
        # 현재 방법의 인덱스 찾기
        try:
            current_index = self.capture_methods.index(failed_method)
            next_index = (current_index + 1) % len(self.capture_methods)
            next_method = self.capture_methods[next_index]
            
            # 방법 변경
            info['current_method'] = next_method
            info['method_change_time'] = time.time()
            info['consecutive_errors_method'][failed_method] = 0  # 오류 카운터 리셋
            
            # 로그 출력
            self.get_logger().info(
                f"'{window_name}' 창 캡처 방법 변경: {failed_method} → {next_method} "
                f"(이유: {error_details})"
            )
            
            # 진단 정보 발행
            self.publish_diagnostics(
                window_name, 
                f"캡처 방법 변경: {failed_method} → {next_method}\n오류 원인: {error_details}"
            )
        except ValueError:
            # 현재 방법이 리스트에 없는 경우 첫 번째 방법으로 설정
            info['current_method'] = self.capture_methods[0] if self.capture_methods else 'import'
            self.get_logger().warn(f"알 수 없는 캡처 방법: {failed_method}, 재설정: {info['current_method']}")
    
    def capture_with_method(self, window_id, x, y, width, height, method='import'):
        """지정된 방법으로 창 캡처 시도"""
        start_time = time.time()
        
        # 메서드 통계를 위한 작업
        try:
            # 방법에 따라 다른 캡처 방식 사용
            if method == 'import':
                image = self.capture_with_import(window_id)
            elif method == 'xwd':
                image = self.capture_with_xwd(window_id)
            elif method == 'scrot':
                image = self.capture_with_scrot(window_id, x, y, width, height)
            elif method == 'ffmpeg':
                image = self.capture_with_ffmpeg(window_id, x, y, width, height)
            else:
                # 알 수 없는 방법
                error_msg = f"지원되지 않는 캡처 방법: {method}"
                self.get_logger().error(error_msg)
                if method in self.method_stats:
                    self.method_stats[method]['fail'] += 1
                    self.method_stats[method]['last_error'] = error_msg
                return None, method, error_msg
            
            # 결과 확인
            if image is None:
                # 실패
                error_msg = self.method_stats[method]['last_error'] or "알 수 없는 오류"
                self.method_stats[method]['fail'] += 1
                return None, method, error_msg
            else:
                # 성공
                self.method_stats[method]['success'] += 1
                elapsed = time.time() - start_time
                return image, method, f"성공 (소요 시간: {elapsed*1000:.1f}ms)"
        
        except Exception as e:
            # 예외 발생
            error_msg = f"{method} 캡처 중 예외 발생: {str(e)}"
            self.get_logger().error(error_msg)
            
            # 상세 오류 정보
            tb = traceback.format_exc()
            
            if method in self.method_stats:
                self.method_stats[method]['fail'] += 1
                self.method_stats[method]['last_error'] = f"{error_msg}\n{tb}"
                
            return None, method, error_msg
    
    def capture_with_import(self, window_id):
        """ImageMagick import 명령으로 디스크 I/O 없이 메모리에서 창 캡처"""
        try:
            # import를 사용하여 PNG 데이터를 표준 출력으로 전송
            process = subprocess.Popen(
                ['import', '-silent', '-window', window_id, 'png:-'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 시간 제한 설정
            try:
                stdout, stderr = process.communicate(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                self.method_stats['import']['last_error'] = f"시간 초과 ({self.timeout}초)"
                return None
            
            # 종료 코드 확인
            if process.returncode != 0:
                error_message = stderr.decode('utf-8', errors='replace').strip()
                self.method_stats['import']['last_error'] = f"import 실패 (코드 {process.returncode}): {error_message}"
                return None
            
            # 표준 오류 확인
            if stderr and len(stderr) > 0:
                error_message = stderr.decode('utf-8', errors='replace').strip()
                self.get_logger().debug(f"import 경고: {error_message}")
            
            # 이미지 데이터 확인
            if not stdout or len(stdout) < 100:  # 최소 이미지 크기 확인
                self.method_stats['import']['last_error'] = f"이미지 데이터가 너무 작음 ({len(stdout) if stdout else 0} 바이트)"
                return None
            
            # 이미지 데이터를 메모리에서 바로 디코딩
            nparr = np.frombuffer(stdout, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.method_stats['import']['last_error'] = "이미지 디코딩 실패"
                return None
            
            return image
            
        except Exception as e:
            self.method_stats['import']['last_error'] = f"예외 발생: {str(e)}"
            return None
    
    def capture_with_xwd(self, window_id):
        """xwd와 파이프를 사용하여 디스크 I/O 없이 창 캡처"""
        try:
            # xwd로 캡처 후 convert로 PNG로 변환
            xwd_process = subprocess.Popen(
                ['xwd', '-silent', '-id', window_id],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            convert_process = subprocess.Popen(
                ['convert', '-', 'png:-'],
                stdin=xwd_process.stdout,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 첫 번째 프로세스의 출력 파이프 닫기
            xwd_process.stdout.close()
            
            # 시간 제한 설정
            try:
                stdout, stderr = convert_process.communicate(timeout=self.timeout)
                xwd_process.wait(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                convert_process.kill()
                xwd_process.kill()
                self.method_stats['xwd']['last_error'] = f"시간 초과 ({self.timeout}초)"
                return None
            
            # 종료 코드 확인
            if convert_process.returncode != 0 or xwd_process.returncode != 0:
                error_message = stderr.decode('utf-8', errors='replace').strip()
                self.method_stats['xwd']['last_error'] = (
                    f"xwd/convert 실패 (xwd: {xwd_process.returncode}, convert: {convert_process.returncode}): {error_message}"
                )
                return None
            
            # 이미지 데이터 확인
            if not stdout or len(stdout) < 100:  # 최소 이미지 크기 확인
                self.method_stats['xwd']['last_error'] = f"이미지 데이터가 너무 작음 ({len(stdout) if stdout else 0} 바이트)"
                return None
            
            # 이미지 데이터를 메모리에서 바로 디코딩
            nparr = np.frombuffer(stdout, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.method_stats['xwd']['last_error'] = "이미지 디코딩 실패"
                return None
            
            return image
            
        except Exception as e:
            self.method_stats['xwd']['last_error'] = f"예외 발생: {str(e)}"
            return None
    
    def capture_with_scrot(self, window_id, x, y, width, height):
        """scrot을 사용하여 디스크 I/O 없이 영역 캡처"""
        try:
            # 윈도우 위치와 크기 확인
            if x is None or y is None or width is None or height is None:
                win_info = subprocess.check_output(['xwininfo', '-id', window_id], text=True)
                
                for line in win_info.splitlines():
                    if 'Absolute upper-left X:' in line:
                        x = int(line.split(':')[1].strip())
                    elif 'Absolute upper-left Y:' in line:
                        y = int(line.split(':')[1].strip())
                    elif 'Width:' in line:
                        width = int(line.split(':')[1].strip())
                    elif 'Height:' in line:
                        height = int(line.split(':')[1].strip())
                
                if x is None or y is None or width is None or height is None:
                    self.method_stats['scrot']['last_error'] = "xwininfo에서 창 위치/크기를 가져올 수 없음"
                    return None
            
            # scrot으로 영역 캡처
            scrot_process = subprocess.Popen(
                ['scrot', '-z', '-a', f'{x},{y},{width},{height}', '-', '-e', 'cat > /dev/stdout'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 시간 제한 설정
            try:
                stdout, stderr = scrot_process.communicate(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                scrot_process.kill()
                self.method_stats['scrot']['last_error'] = f"시간 초과 ({self.timeout}초)"
                return None
            
            # 종료 코드 확인
            if scrot_process.returncode != 0:
                error_message = stderr.decode('utf-8', errors='replace').strip()
                self.method_stats['scrot']['last_error'] = f"scrot 실패 (코드 {scrot_process.returncode}): {error_message}"
                return None
            
            # 이미지 데이터 확인
            if not stdout or len(stdout) < 100:
                self.method_stats['scrot']['last_error'] = f"이미지 데이터가 너무 작음 ({len(stdout) if stdout else 0} 바이트)"
                return None
            
            # 이미지 데이터를 메모리에서 바로 디코딩
            nparr = np.frombuffer(stdout, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.method_stats['scrot']['last_error'] = "이미지 디코딩 실패"
                return None
            
            return image
            
        except Exception as e:
            self.method_stats['scrot']['last_error'] = f"예외 발생: {str(e)}"
            return None
    
    def capture_with_ffmpeg(self, window_id, x, y, width, height):
        """ffmpeg를 사용하여 디스크 I/O 없이 화면 캡처"""
        try:
            # 윈도우 위치와 크기 확인
            if x is None or y is None or width is None or height is None:
                win_info = subprocess.check_output(['xwininfo', '-id', window_id], text=True)
                
                for line in win_info.splitlines():
                    if 'Absolute upper-left X:' in line:
                        x = int(line.split(':')[1].strip())
                    elif 'Absolute upper-left Y:' in line:
                        y = int(line.split(':')[1].strip())
                    elif 'Width:' in line:
                        width = int(line.split(':')[1].strip())
                    elif 'Height:' in line:
                        height = int(line.split(':')[1].strip())
                
                if x is None or y is None or width is None or height is None:
                    self.method_stats['ffmpeg']['last_error'] = "xwininfo에서 창 위치/크기를 가져올 수 없음"
                    return None
            
            # ffmpeg로 화면 영역 캡처
            ffmpeg_process = subprocess.Popen([
                'ffmpeg',
                '-hide_banner',
                '-loglevel', 'error',
                '-f', 'x11grab',
                '-draw_mouse', '0',
                '-video_size', f'{width}x{height}',
                '-i', f':0.0+{x},{y}',
                '-vframes', '1',
                '-f', 'image2pipe',
                '-c:v', 'png',
                '-pix_fmt', 'bgr24',
                '-'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 시간 제한 설정
            try:
                stdout, stderr = ffmpeg_process.communicate(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                ffmpeg_process.kill()
                self.method_stats['ffmpeg']['last_error'] = f"시간 초과 ({self.timeout}초)"
                return None
            
            # 종료 코드 확인
            if ffmpeg_process.returncode != 0:
                error_message = stderr.decode('utf-8', errors='replace').strip()
                self.method_stats['ffmpeg']['last_error'] = f"ffmpeg 실패 (코드 {ffmpeg_process.returncode}): {error_message}"
                return None
            
            # 이미지 데이터 확인
            if not stdout or len(stdout) < 100:
                self.method_stats['ffmpeg']['last_error'] = f"이미지 데이터가 너무 작음 ({len(stdout) if stdout else 0} 바이트)"
                return None
            
            # 이미지 데이터를 메모리에서 바로 디코딩
            nparr = np.frombuffer(stdout, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.method_stats['ffmpeg']['last_error'] = "이미지 디코딩 실패"
                return None
            
            return image
            
        except Exception as e:
            self.method_stats['ffmpeg']['last_error'] = f"예외 발생: {str(e)}"
            return None
    
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        self.running = False
        
        # 쓰레드 종료 대기
        self.get_logger().info("모든 캡처 쓰레드 정리 중...")
        for name, thread in self.window_threads.items():
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        
        # 캡처 방법 통계 출력
        self.get_logger().info("캡처 방법 성공/실패 통계:")
        for method, data in self.method_stats.items():
            total = data['success'] + data['fail']
            if total > 0:
                success_rate = data['success'] / total * 100
                self.get_logger().info(f"  {method}: 성공률 {success_rate:.1f}% ({data['success']}/{total})")
        
        # 부모 클래스 종료 처리
        super().destroy_node()

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        node = MemoryCapturePublisher()
        
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
        print(traceback.format_exc())
    finally:
        # ROS2 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()
