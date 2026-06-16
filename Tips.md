## interface 구현 시

- CMakeLists.txt 추가
```bash
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/BatteryStatus.msg"
  "msg/DiagnosticEvent.msg"
  "msg/Heartbeat.msg"
  "msg/ObstacleInfo.msg"
  "msg/SecurityEvent.msg"
  "msg/VehicleState.msg"

  "srv/GetFaultInfo.srv"
  "srv/ClearFault.srv"
  "srv/CalibrateSensor.srv"

  "action/GoToTarget.action"
  "action/ReturnHome.action"
)
```

- package.xml 추가
```bash
  <build_depend>rosidl_default_generators</build_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
```

## Package 생성

```bash
cd ./src # src 폴더로 이동

ros2 pkg create --build-type <빌드_타입> <패키지_이름>

# 빌드 타입
# ament_cmake -> C++
# ament_python -> python

# Arguments
# --dependencies <pkg1> <pkg2> : 패키지 의존성 추가
# --node-name <node-name> : 코드 템플릿 함께 생성
# --license <license-name> : 라이선스 지정 (default : Apache-2.0)

```

## build

```bash
colcon build
# 워크스페이스 전체 패키지 빌드

colcon build --packages-select "폴더명"
# --symlink-install : python 의 경우 install 폴더와 연동 되어 별도 빌드 필요X

colcon build --packages-up-to "폴더명"
# 폴더명 의 의존 패키지까지 같이 빌드
# package.xml에 적힌 exec_depend/build_depend 기준

# 성공 후 반영
source install/setup.bash

# 확인
ros2 interface list | grep sdv
```

## run

``` bash

ros2 run <pkg_name> <node_name>

```


## launch

``` bash

ros2 pkg create sdv_bringup \
  --build-type ament_python \
  --dependencies launch launch_ros

source install/setup.bash
ros2 launch sdv_bringup sdv_system.launch.py
```