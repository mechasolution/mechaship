# MechaShip

<p align="center">
  <strong>ROS 2 packages for autonomous surface vessel development.</strong>
</p>

<p align="center">
  <a href="https://docs.ros.org/en/jazzy/"><img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white"></a>
  <a href="https://ubuntu.com/download/desktop"><img alt="Ubuntu 24.04" src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white"></a>
  <a href="https://gazebosim.org/"><img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic-F58113"></a>
  <a href="https://bluexrobotics.kr/wiki"><img alt="Documentation" src="https://img.shields.io/badge/Docs-BlueX%20Wiki-0A66C2"></a>
  <a href="https://forums.bluexrobotics.kr/"><img alt="Forum" src="https://img.shields.io/badge/Forum-BlueX%20Robotics-4B5563"></a>
  <a href="#라이선스"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue"></a>
</p>

MechaShip은 ROS 2 기반 무인 수상정 개발을 위한 통합 패키지 모음입니다. 실제 하드웨어 bringup, Gazebo 시뮬레이션, 로봇 모델, SLAM, 원격 조종, RKNN 기반 YOLOv8 객체 인식까지 하나의 워크스페이스에서 사용할 수 있도록 구성되어 있습니다.

## 링크

- GitHub: https://github.com/mechasolution/mechaship
- Documentation: https://bluexrobotics.kr/wiki
- Forum: https://forums.bluexrobotics.kr/

## 주요 기능

- 실제 보트 구동을 위한 센서, 액추에이터, micro-ROS 통합 launch
- Gazebo Harmonic 기반 수상정 시뮬레이션과 ROS-Gazebo topic bridge
- SDF 모델, STL mesh, robot state publisher 구성
- LiDAR odometry, EKF, SLAM Toolbox 기반 위치 추정 및 지도 작성
- 키보드/조이스틱 기반 원격 조종 노드
- RKNN YOLOv8 객체 인식 및 detection visualization 노드

## 요구 사항

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz` 패키지
- `colcon`, `rosdep`
- 실제 하드웨어 사용 시 USB 카메라, LiDAR, IMU, GPS, micro-ROS 연결 장치
- YOLOv8 RKNN 실행 시 RKNN 런타임이 설치된 보드 환경

## 빠른 시작

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/mechasolution/mechaship.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

빌드 후에는 실제 하드웨어 또는 시뮬레이션 중 하나를 선택해서 실행합니다.

실제 MechaShip 하드웨어를 사용할 때:

```bash
ros2 launch mechaship_bringup mechaship_bringup.launch.py
```

Gazebo 시뮬레이션을 사용할 때:

```bash
ros2 launch mechaship_simulation gazebo.launch.py
```

> 실제 하드웨어 bringup과 Gazebo 시뮬레이션은 같은 로봇 시스템을 대상으로 하므로 동시에 실행하지 마세요.

## 패키지 구성

| 패키지 | 설명 |
| --- | --- |
| `mechaship_bringup` | 실제 MechaShip 실행용 통합 launch 및 센서/액추에이터 파라미터 |
| `mechaship_description` | MechaShip SDF 모델과 STL mesh |
| `mechaship_interfaces` | MechaShip 전용 message, service, action 정의 |
| `mechaship_simulation` | Gazebo 시뮬레이션 world, spawn, ROS-Gazebo bridge |
| `mechaship_slam` | RF2O laser odometry, EKF, SLAM Toolbox launch |
| `mechaship_system` | 액추에이터 enable 등 시스템 보조 노드 |
| `mechaship_teleop` | 키보드/조이스틱 원격 조종 노드 |
| `mechaship_yolov8` | RKNN YOLOv8 객체 인식 및 시각화 노드 |

## 관련 드라이버 패키지

외부 드라이버 패키지는 git submodule로 포함되어 있습니다.

- [`iahrs_ros2_driver`](https://github.com/mechasolution/iahrs_ros2_driver)
- [`wtrtk_ros2_driver`](https://github.com/mechasolution/wtrtk_ros2_driver)
- [`ydlidar_ros2_driver`](https://github.com/mechasolution/ydlidar_ros2_driver)
- [`rf2o_laser_odometry`](https://github.com/mechasolution/rf2o_laser_odometry)
- [`ntrip_client`](https://github.com/maax-cha/ntrip_client)

## 원격 조종

키보드 조종 노드를 실행합니다.

```bash
ros2 run mechaship_teleop mechaship_teleop_keyboard
```

조이스틱 조종은 `joy_node`와 함께 launch로 실행합니다.

```bash
ros2 launch mechaship_teleop mechaship_teleop_joystick.launch.py
```

## SLAM

EKF와 LiDAR odometry를 실행합니다.

```bash
ros2 launch mechaship_slam ekf.launch.py
```

SLAM Toolbox를 함께 실행합니다.

```bash
ros2 launch mechaship_slam slam_toolbox.launch.py
```

`mechaship_slam`은 `/robot_state_publisher`의 `use_sim_time` 값을 읽어 실제 환경과 시뮬레이션 환경을 자동으로 맞춥니다.

## YOLOv8 객체 인식

객체 인식 노드를 실행합니다.

```bash
ros2 launch mechaship_yolov8 detect.launch.py
```

인식 결과 시각화 노드를 실행합니다.

```bash
ros2 launch mechaship_yolov8 visualize.launch.py
```

기본 입력 영상 토픽은 `/image_raw/compressed`이며, 인식 결과는 `/detections` 토픽으로 publish됩니다. 모델과 라벨 파일은 `mechaship_yolov8/model`에 있고, 파라미터는 `mechaship_yolov8/param/yolov8_params.yaml`에서 수정합니다.

## 라이선스

이 저장소의 MechaShip 패키지는 `Apache-2.0` 라이선스를 사용합니다. 포함된 외부 submodule의 라이선스는 각 패키지의 `LICENSE` 또는 README를 확인하세요.
