# ROS2 기반 SDV 분산 ECU 시뮬레이션 플랫폼

## 1. 프로젝트 개요

### 프로젝트명

ROS2 기반 SDV(Software Defined Vehicle) 분산 ECU 시뮬레이션 플랫폼

### 프로젝트 목표

ROS2 DDS 기반 통신을 활용하여 차량 ECU 구조를 모사한 분산 시스템을 구축한다.

각 기능은 독립적인 ECU(Node) 형태로 설계하며, Vehicle Manager를 중심으로 상태 관리, Fault Handling, Cyber Security 기능을 구현한다.

실물 플랫폼(Mentor Pi M1)을 이용하여 시스템 동작을 검증한다.

---

## 2. 시스템 아키텍처

```text
                 Vehicle Manager ECU
                          │
      ┌───────────────────┼───────────────────┐
      │                   │                   │
      ▼                   ▼                   ▼

 Battery ECU        Sensor ECU         Diagnostics ECU
      │                   │                   │
      └───────────┬───────┴───────────┬───────┘
                  │                   │
                  ▼                   ▼

             Security ECU        Motor ECU
                                      │
                                      ▼

                               Mentor Pi M1
```

---

## 3. ECU 구성

### Vehicle Manager ECU

#### 역할

* 시스템 상태 관리
* ECU 상태 감시
* 미션 제어
* 최종 의사결정 수행

#### 상태(State)

```text
state
mission_active

INIT
READY
MISSION
LOW_BATTERY
FAULT
EMERGENCY
MRM
```

`mission_active`는 현재 상태에서 실제 주행 미션이 활성화되어 있는지를 나타낸다.
예를 들어 LOW_BATTERY 상태에서도 StartMission 요청이 성공하면
`state=LOW_BATTERY`, `mission_active=true`로 발행되며,
Motor ECU는 이를 limp mode 주행으로 처리한다.

`MRM`은 주행계와 센서가 정상인 경미한 이상에서 수행하는
Minimal Risk Maneuver 상태다. 현재는 Low Battery 발생 시 원점의
충전 위치로 자동 복귀하는 데 사용한다.

---

### Motor ECU

#### 역할

* Vehicle State 기반 모터 동작 정책 수행
* 시뮬레이션 모터 상태 갱신
* 현재 속도 상태 보고
* 실제 H/W Driver 교체 가능한 구조 제공

#### 제어 대상

* SimMotorDriver
* Mentor Pi M1 모터 (확장 예정)

---

### Sensor ECU

#### 역할

* 장애물 감지
* 거리 측정
* 센서 데이터 제공
* 실제 H/W Driver 교체 가능한 구조 제공

#### 입력

* SimSensorDriver
* LiDAR
* Ultrasonic Sensor
* Camera (확장 가능)

---

### Battery ECU

#### 역할

* SOC 관리
* 배터리 상태 제공

#### 제공 데이터

```text
SOC
Voltage
Current
```

---

### Diagnostics ECU

#### 역할

* ECU Health Monitoring
* Heartbeat 감시
* Fault 관리

---

### Security ECU

#### 역할

* 이상 통신 탐지
* 비정상 데이터 탐지
* 공격 감지

---

## 4. ROS2 통신 구조

### Topic

주기적인 상태 데이터 송수신

#### 사용 예

```text
Battery ECU
  └─> /ecu/battery/status

Sensor ECU
  └─> /ecu/obstacle/info

Motor ECU
  └─> /ecu/motor/status

Motor ECU
  └─> /ecu/vehicle/pose

Vehicle Manager
  └─> /ecu/vehicle/status

All ECU
  └─> /ecu/heartbeat
```

---

### Service

요청/응답 기반 통신

#### 사용 예

```text
/ecu/diagnostics/get_fault_info

/ecu/diagnostics/clear_fault

/ecu/sensor/calibrate

/ecu/vehicle/start_mission
```

---

### Action

장시간 수행 작업

#### 사용 예

```text
/go_to_target

/return_home
```

---

## 5. Topic 설계

### /ecu/battery/status

#### Publisher

Battery ECU

#### Subscriber

* Vehicle Manager ECU
* Security ECU

#### 데이터

```text
SOC
Voltage
Current
```

---

### /ecu/obstacle/info

#### Publisher

Sensor ECU

#### Subscriber

* Vehicle Manager ECU
* Security ECU
* Test GUI

#### 데이터

```text
Detected
Distance
Angle
```

---

### /ecu/motor/status

#### Publisher

Motor ECU

#### Subscriber

* Vehicle Manager ECU
* Security ECU
* Test GUI

#### 데이터

```text
Target Linear Velocity
Current Linear Velocity
Target Angular Velocity
Current Angular Velocity
Enabled
```

---

### /ecu/vehicle/status

#### Publisher

Vehicle Manager ECU

#### Subscriber

전체 ECU

#### 데이터

```text
INIT
READY
MISSION
LOW_BATTERY
FAULT
EMERGENCY
```

---

### /ecu/heartbeat

#### Publisher

모든 ECU

#### Subscriber

* Vehicle Manager ECU
* Diagnostics ECU
* Test GUI

#### 데이터

```text
ECU Name
Timestamp
```

---

### /ecu/diagnostics/event

#### Publisher

Diagnostics ECU

#### Subscriber

* Test GUI

#### 데이터

```text
ECU Name
Severity
Description
```

---

### /ecu/security/event

#### Publisher

Security ECU

#### Subscriber

Vehicle Manager ECU

#### 데이터

```text
Attack Type
Severity
Timestamp
```

---

## 6. Service 설계

### /ecu/diagnostics/get_fault_info

#### Client

Vehicle Manager ECU

#### Server

Diagnostics ECU

#### 기능

현재 Fault 정보 조회

---

### /ecu/diagnostics/clear_fault

#### Client

Vehicle Manager ECU

#### Server

Diagnostics ECU

#### 기능

Fault 초기화

---

### /ecu/sensor/calibrate

#### Client

Vehicle Manager ECU

#### Server

Sensor ECU

#### 기능

센서 캘리브레이션 수행

---

### /ecu/vehicle/start_mission

#### Client

Test GUI

#### Server

Vehicle Manager ECU

#### 기능

READY 상태에서 MISSION 상태로 전이를 요청한다.

응답으로 요청 성공 여부와 실패 사유를 반환한다.

---

### /ecu/vehicle/reset_emergency

#### Client

Test GUI

#### Server

Vehicle Manager ECU

#### 기능

EMERGENCY 상태를 INIT 상태로 초기화하고 ECU Heartbeat 및 Battery 상태를
다시 확인하는 재초기화 절차를 수행한다.

EMERGENCY가 아닌 상태에서는 요청을 거부한다.

---

## 7. Action 설계

### /go_to_target

#### Client

Vehicle Manager ECU

#### Server

Motor ECU

---

### Goal

```text
Target Position
```

---

### Feedback

```text
Remaining Distance
Progress(%)
```

---

### Result

```text
SUCCESS
FAIL
CANCELED
```

---

### /return_home

#### Client

Vehicle Manager ECU

#### Server

Motor ECU

#### 기능

홈 위치 복귀

---

## 8. 기능 요구사항

### FR-001

Vehicle Manager는 시스템 상태를 관리해야 한다.

---

### FR-002

Battery ECU는 1초 주기로 배터리 상태를 발행해야 한다.

---

### FR-003

Sensor ECU는 주기적으로 장애물 정보를 발행해야 한다.

---

### FR-004

Motor ECU는 Vehicle State와 mission_active에 따라 모터 상태를 제어하고 MotorStatus를 발행해야 한다.

```text
MISSION + mission_active=true
  -> normal speed

LOW_BATTERY + mission_active=true
  -> limp speed

LOW_BATTERY + mission_active=false
  -> stop

FAULT / EMERGENCY
  -> stop / emergency stop
```

---

### FR-005

Vehicle Manager는 SOC <= 20% 조건에서 LOW_BATTERY 상태로 진입해야 한다.
주행 또는 READY 상태에서 Low Battery가 발생하면 MRM 상태로 전이하고
ReturnHome Action을 자동 호출해야 한다.
복귀 중에는 limp mode 속도를 사용하며 원점 도착 후
`LOW_BATTERY`, `mission_active=false` 상태로 대기해야 한다.
SOC >= 25% 조건이 5초 이상 유지되면 INIT 상태로 복귀하여 재초기화 과정을 수행해야 한다.

---

### FR-006

Vehicle Manager는 장애물 감지 시 정지 명령을 수행해야 한다.

---

### FR-006-1

Sensor ECU는 시뮬레이션 센서 Driver와 실제 H/W Driver를 교체 가능한 구조로 제공해야 한다.

---

### FR-006-2

Motor ECU는 시뮬레이션 모터 Driver와 실제 H/W Driver를 교체 가능한 구조로 제공해야 한다.

---

### FR-007

Diagnostics ECU는 모든 ECU의 Heartbeat를 감시해야 한다.

---

### FR-008

Heartbeat Timeout 발생 시 Fault를 생성해야 한다.

---

### FR-009

Fault 발생 시 Vehicle Manager는 Emergency Stop을 수행해야 한다.

---

## 9. Cyber Security 요구사항

### CSR-001

Security ECU는 주요 Topic을 모니터링해야 한다.

감시 대상

```text
/ecu/battery/status
/ecu/motor/status
/ecu/vehicle/status
```

---

### CSR-002

비정상 배터리 데이터를 탐지해야 한다.

예시

```text
SOC > 100
SOC < 0
Voltage < 0
```

---

### CSR-003

비정상 속도 명령을 탐지해야 한다.

예시

```text
Speed > MaxSpeed
```

---

### CSR-004

이상 탐지 시 Security Event를 발행해야 한다.

---

### CSR-005

Vehicle Manager는 Security Event 수신 시 EMERGENCY 상태로 진입해야 한다.

---

## 10. 시연 시나리오

### Demo #1 : 정상 주행

```text
Mission Start

↓

Move

↓

Obstacle Detect

↓

Stop
```

Test GUI의 `Start Mission` 버튼을 누르면 미션 시작 약 4초 후
시뮬레이션 장애물이 발생한다. 장애물 감지 중에는 모터가 정지하고,
장애물이 사라지면 남은 미션을 재개한다.

---

### Demo #2 : Low Battery

```text
SOC = 15%

↓

LOW_BATTERY

↓

Start Mission

↓

LOW_BATTERY + mission_active=true

↓

Limp Mode

↓

Stop
```

Test GUI의 Battery Simulator에서 SOC를 15%로 설정해 발행한 뒤
`Start Mission`을 누른다. `LOW_BATTERY / ACTIVE`와 0.1 m/s의
limp mode를 확인한다.

---

### Demo #3 : ECU Failure

```text
Sensor ECU 종료

↓

Heartbeat Timeout

↓

Diagnostics Fault

↓

Emergency Stop
```

실행 중인 Sensor ECU 프로세스를 종료하면 약 3초 후
Diagnostics ECU가 ERROR 이벤트를 발행하고 Vehicle Manager가
EMERGENCY로 전이한다.

---

### Demo #4 : Cyber Attack

```text
공격 노드 실행

↓

가짜 SOC = 150% 송신

↓

Security ECU 탐지

↓

Security Event 발생

↓

Emergency Stop
```

다른 터미널에서 다음 공격 노드를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run attack_node attack_node
```

공격 노드는 SOC 150% 메시지를 발행한다. Security ECU의
`BATTERY_SOC_RANGE` 이벤트와 Vehicle Manager의 EMERGENCY 전이를
확인한다.

---

## 10-1. 빌드 및 실행

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sdv_bringup sdv_system.launch.py
```

GUI 없이 실행하려면 다음 launch argument를 사용한다.

```bash
ros2 launch sdv_bringup sdv_system.launch.py gui:=false
```

EMERGENCY 발생 후 GUI의 `Reset Emergency → INIT` 버튼을 누르면
Vehicle Manager가 INIT으로 전이한 뒤 시스템 준비 상태를 다시 확인한다.
정상 ECU가 모두 실행 중이면 READY로 자동 전이한다.

GUI의 `Action Commands` 영역에서는 목표 X/Y를 입력하고
`Go To Target` 또는 `Return Home`을 실행할 수 있다. Action Feedback은
Progress 항목에 0~100%로 표시되고 최종 성공 여부는 Result 항목에
표시된다.

GUI에서 Action을 실행하면 READY 상태에서는 Start Mission을 자동으로
요청한 뒤 Action Goal을 전송한다. LOW_BATTERY idle 상태에서도
limp mode 미션을 자동 시작한다. INIT, FAULT, EMERGENCY 상태의 요청은
거부된다.

Action 수행 중 장애물이 감지되면 Goal을 실패 처리하지 않는다.
Motor를 정지하고 Action 진행률과 미션 완료 시간을 일시정지한 뒤,
장애물이 해제되면 같은 Goal을 이어서 수행한다. Action은 명시적 취소,
FAULT 또는 EMERGENCY 발생 시에만 실패 또는 중단된다.

Low Battery가 일반 미션 또는 GoToTarget 수행 중 발생하면 기존 Goal을
중단하고 `MRM` 상태에서 ReturnHome을 자동 수행한다. Motor, Sensor,
Heartbeat 또는 Security 중대 이상은 MRM을 수행하지 않고 즉시
EMERGENCY Stop을 유지한다.

Action 수행 중 MotorStatus가 계속 발행되므로 GUI의 Motor Status,
Dashboard 및 2D Simulation이 실제 선속도·각속도에 따라 갱신된다.
Motor ECU는 `/ecu/vehicle/pose`로 실제 시뮬레이션 위치를 발행한다.
`Go To Target`은 현재 위치에서 입력한 절대좌표까지 pose 오차를
폐루프 제어하며, `Return Home`은 실제 현재 위치에서 원점 `(0, 0)`까지
주행한다. GUI 좌표를 강제로 변경하는 순간이동 처리는 사용하지 않는다.

Action 목표 도달 후 Vehicle Manager의 미션을 완료 처리하여 차량을
정지 상태로 유지한다.

Diagnostics 서비스 확인:

```bash
ros2 service call /ecu/diagnostics/get_fault_info \
  sdv_interfaces/srv/GetFaultInfo '{}'
ros2 service call /ecu/diagnostics/clear_fault \
  sdv_interfaces/srv/ClearFault "{fault_name: all}"
```

Action 서버 확인:

```bash
ros2 action send_goal /go_to_target \
  sdv_interfaces/action/GoToTarget "{x: 1.0, y: 0.0}" --feedback
ros2 action send_goal /return_home \
  sdv_interfaces/action/ReturnHome "{start: true}" --feedback
```

위 Action은 Test GUI에서도 동일하게 실행할 수 있으며 CLI 명령은
GUI가 없는 환경의 확인 용도로 사용할 수 있다.

---

## 11. 개발 단계

### Phase 1 (MVP)

* Vehicle Manager ECU
* Battery ECU
* Sensor ECU Simulation
* Motor ECU Simulation
* Diagnostics ECU
* Test GUI
* Topic 통신
* 기본 상태 머신

---

### Phase 2

* Service 추가
* Action 추가
* Return Home 기능

---

### Phase 3

* Security ECU
* Attack Node
* IDS 기능
* Cyber Security 시나리오

---

### 최종 목표

차량 SW 개발자의 관점에서 ROS2 DDS 기반 분산 ECU 구조를 구현하고, Fault Handling 및 Cyber Security 기능을 포함한 SDV 아키텍처 축소 모델을 완성한다.
