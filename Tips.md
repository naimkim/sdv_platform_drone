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

## build

```bash
colcon build --packages-select "폴더명"

# 성공 후 반영
source install/setup.bash

# 확인
ros2 interface list | grep sdv
```