# LIMO ROS2 Line Follower

ROS 2 Humble 기반의 LIMO 로봇 라인 추종 및 장애물 회피 시스템입니다.

## 시스템 구성
* **카메라(/camera/image_raw)**: ROI 기반 영상 처리를 통한 라인 검출 및 조향 제어
* **라이다(/scan)**: 전방 장애물 감지 시 우선순위 제어 로직 적용
* **제어(/cmd_vel)**: 장애물 여부에 따른 상태 머신 기반 주행

## 실행 방법
```bash
colcon build --packages-select line_follower
source install/setup.bash
ros2 run line_follower object_detect
