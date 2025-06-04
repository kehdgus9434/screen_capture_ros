### 필요한 패키지 설치 
    sudo apt install wmctrl imagemagick x11-utils scrot ffmpeg

### 파일 저장 위치
 - ros2_ws/src 안에 본 파일을 저장

### 실행 권한 부여
    chmod +x memory_capture_publisher.py

### 패키지 빌드 
    cd ~/ros2_ws
    colcon build --packages-select screen_capture_ros
    source install/setup.bash

### 노드 실행
    # 기본 실행
    ros2 run screen_capture_ros pure_memory_capture

    # 특정 창 지정
    ros2 run screen_capture_ros pure_memory_capture --ros-args -p window_names:="['SM-S711N']"

    # 캡처 방법 우선순위 지정
    ros2 run screen_capture_ros pure_memory_capture --ros-args -p capture_methods:="['import', 'ffmpeg', 'scrot', 'xwd']"

    # 성능 최적화
    ros2 run screen_capture_ros pure_memory_capture --ros-args -p resize_factor:=0.5 -p publish_rate:=20.0
