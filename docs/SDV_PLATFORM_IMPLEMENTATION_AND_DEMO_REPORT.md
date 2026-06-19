# ROS2 기반 SDV 분산 ECU 시뮬레이션 플랫폼

## 구현 및 시연 결과 보고서

| 항목 | 내용 |
|---|---|
| 프로젝트 | ROS2 기반 SDV 분산 ECU 시뮬레이션 플랫폼 |
| 검증 기준 | `README.md` 기능 요구사항, Cyber Security 요구사항, 시연 시나리오 |
| 검증 일자 | 2026-06-19 |
| 검증 환경 | ROS 2 Jazzy, Python 3.12, Linux |
| 대상 브랜치 | `master` |
| 기준 커밋 | `7d62c67` 이후 로컬 구현 변경사항 포함 |
| 최종 판정 | 전체 빌드 및 자동 테스트 통과, Demo #1~#4 시연 가능 |

---

## 1. 프로젝트 개요

본 프로젝트는 ROS 2 DDS 통신을 기반으로 차량의 분산 ECU 구조를
시뮬레이션하는 플랫폼이다.

Vehicle Manager ECU를 중심으로 Battery, Sensor, Motor, Diagnostics,
Security ECU가 독립적인 ROS 2 Node로 동작한다. 각 ECU는 Topic,
Service, Action을 이용하여 차량 상태 관리, 주행 제어, 장애물 대응,
Heartbeat 기반 고장 탐지 및 비정상 데이터 기반 보안 대응 기능을
수행한다.

현재 시뮬레이션 환경에서 다음 기능을 검증하였다.

- 정상 미션 시작과 모터 주행
- 장애물 탐지에 따른 정지 및 미션 재개
- 저전압 상태와 limp mode 주행
- ECU Heartbeat timeout과 Emergency Stop
- 비정상 SOC 공격 탐지와 Emergency Stop
- Diagnostics fault 조회 및 초기화
- 장시간 동작을 위한 ROS 2 Action 처리

---

## 2. 시스템 구성

| 구성 요소 | 주요 역할 | 구현 상태 |
|---|---|---|
| Vehicle Manager ECU | 상태 머신, 미션 제어, 최종 안전 판단 | 완료 |
| Battery ECU | SOC, 전압, 전류 및 Heartbeat 발행 | 완료 |
| Sensor ECU | 장애물 시뮬레이션 및 캘리브레이션 | 완료 |
| Motor ECU | 차량 상태 기반 모터 정책 및 Action 수행 | 완료 |
| Diagnostics ECU | Heartbeat 감시, fault 저장 및 조회 | 완료 |
| Security ECU | Battery/Motor/Vehicle Topic 이상 탐지 | 완료 |
| Attack Node | 비정상 SOC 150% 공격 메시지 발행 | 완료 |
| Test GUI | 상태, 센서, 모터, Heartbeat 및 이벤트 표시 | 완료 |
| Mentor Pi M1 연동 | 실제 센서 및 모터 하드웨어 제어 | 확장 지점 제공 |

---

## 3. ROS 2 통신 구현 결과

### 3.1 Topic

| Topic | Publisher | 주요 Subscriber | 상태 |
|---|---|---|---|
| `/ecu/battery/status` | Battery ECU, Test GUI, Attack Node | Vehicle Manager, Diagnostics, Security, GUI | 구현 |
| `/ecu/obstacle/info` | Sensor ECU | Vehicle Manager, GUI | 구현 |
| `/ecu/motor/status` | Motor ECU | Security, GUI | 구현 |
| `/ecu/vehicle/pose` | Motor ECU | Test GUI | 구현 |
| `/ecu/vehicle/status` | Vehicle Manager | Battery, Sensor, Motor, Diagnostics, Security, GUI | 구현 |
| `/ecu/heartbeat` | 각 ECU 및 GUI | Vehicle Manager, Diagnostics, GUI | 구현 |
| `/ecu/diagnostics/event` | Diagnostics ECU | Vehicle Manager, GUI | 구현 |
| `/ecu/security/event` | Security ECU | Vehicle Manager, GUI | 구현 |

### 3.2 Service

| Service | Server | 기능 | 상태 |
|---|---|---|---|
| `/ecu/vehicle/start_mission` | Vehicle Manager | 정상 및 Low Battery 미션 시작 | 구현 및 검증 |
| `/ecu/vehicle/reset_emergency` | Vehicle Manager | EMERGENCY 해제 및 INIT 재초기화 | 구현 |
| `/ecu/sensor/calibrate` | Sensor ECU | Sensor Driver 캘리브레이션 | 구현 |
| `/ecu/diagnostics/get_fault_info` | Diagnostics ECU | 현재 fault 조회 | 구현 및 검증 |
| `/ecu/diagnostics/clear_fault` | Diagnostics ECU | 특정 또는 전체 fault 초기화 | 구현 |

### 3.3 Action

| Action | Server | 기능 | 상태 |
|---|---|---|---|
| `/go_to_target` | Motor ECU | 목표 위치 이동 진행률 및 결과 제공 | 구현 및 검증 |
| `/return_home` | Motor ECU | 홈 복귀 진행률 및 결과 제공 | 구현 |

`GoToTarget` 검증에서는 10%부터 100%까지 Feedback이 발행되었으며,
최종 결과는 `success: true`, 상태는 `SUCCEEDED`로 확인되었다.
두 Action은 Test GUI의 `Action Commands` 영역에서도 실행할 수 있다.
목표 X/Y 입력, Action 실행 버튼, 진행률 및 최종 결과 표시를 제공한다.
GUI Action 요청은 필요한 경우 미션을 자동 시작한다. 실행 중 장애물이
발생하면 Goal과 미션 시간을 일시정지하고 장애물 해제 후 재개한다.
MotorStatus와 2D Simulation은 Action 동작에 연동된다.

---

## 4. 기능 요구사항 검증

| ID | 요구사항 | 구현 근거 | 검증 결과 |
|---|---|---|---|
| FR-001 | Vehicle Manager는 시스템 상태를 관리해야 한다. | INIT, READY, MISSION, LOW_BATTERY, FAULT, EMERGENCY 상태 머신 | 충족 |
| FR-002 | Battery ECU는 1초 주기로 상태를 발행해야 한다. | 1초 Timer에서 BatteryStatus 및 Heartbeat 발행 | 충족 |
| FR-003 | Sensor ECU는 주기적으로 장애물 정보를 발행해야 한다. | 100ms Timer와 Sensor Driver | 충족 |
| FR-004 | Motor ECU는 Vehicle State와 `mission_active`에 따라 동작해야 한다. | normal, limp, stop, emergency stop 정책 | 충족 |
| FR-005 | SOC 20% 이하에서 자동 안전 복귀하고 25% 이상 5초 유지 시 복구해야 한다. | MRM 전이, 자동 ReturnHome 및 recovery hold 로직 | 충족 |
| FR-006 | 장애물 감지 시 정지해야 한다. | 장애물 감지 시 `mission_active=false` 발행 | 충족 |
| FR-006-1 | Sensor Driver 교체 구조를 제공해야 한다. | `SimSensorDriver`, `HwSensorDriver` | 충족 |
| FR-006-2 | Motor Driver 교체 구조를 제공해야 한다. | `SimMotorDriver`, `HwMotorDriver` | 충족 |
| FR-007 | Diagnostics ECU는 Heartbeat를 감시해야 한다. | ECU별 마지막 수신 시각 및 alive 상태 관리 | 충족 |
| FR-008 | Heartbeat timeout 발생 시 Fault를 생성해야 한다. | 필수 ECU 3초 timeout 후 ERROR Event | 충족 |
| FR-009 | Fault 발생 시 Emergency Stop을 수행해야 한다. | Diagnostic ERROR 수신 후 EMERGENCY 전이 | 충족 |

### 4.1 Vehicle State 및 Motor 정책

| Vehicle State | `mission_active` | Motor 동작 |
|---|---:|---|
| READY | false | 정지 |
| MISSION | true | 정상 속도 0.4 m/s |
| MISSION | false | 정지 |
| LOW_BATTERY | true | limp mode 0.1 m/s |
| LOW_BATTERY | false | 정지 |
| MRM | true | 원점까지 limp mode 자동 복귀 |
| FAULT | 무관 | 정지 |
| EMERGENCY | 무관 | 즉시 Emergency Stop |

---

## 5. Cyber Security 요구사항 검증

| ID | 요구사항 | 구현 근거 | 검증 결과 |
|---|---|---|---|
| CSR-001 | 주요 Topic을 모니터링해야 한다. | Battery, Motor, Vehicle Status 구독 | 충족 |
| CSR-002 | 비정상 Battery 데이터를 탐지해야 한다. | SOC, 전압, 전류 범위 검사 | 충족 |
| CSR-003 | 비정상 속도 데이터를 탐지해야 한다. | 선속도, 각속도 및 disabled 상태 검사 | 충족 |
| CSR-004 | 이상 탐지 시 Security Event를 발행해야 한다. | `/ecu/security/event` 발행 | 충족 |
| CSR-005 | Security Event 수신 시 EMERGENCY로 전이해야 한다. | Vehicle Manager Security callback | 충족 |

Battery Security 검사 범위는 다음과 같다.

| 데이터 | 정상 범위 |
|---|---|
| SOC | 0~100% |
| Voltage | 0~80V |
| Current | 절댓값 500A 이하 |

Motor Security 검사에는 목표 및 현재 선속도·각속도 범위와
모터가 disabled 상태인데 속도가 존재하는 경우가 포함된다.

---

## 6. 시연 결과

### 6.1 Demo #1: 정상 주행 및 장애물 정지

#### 절차

1. 전체 시스템을 실행한다.
2. Vehicle State가 READY인지 확인한다.
3. Test GUI 또는 Service로 미션을 시작한다.
4. Motor가 정상 속도로 동작하는지 확인한다.
5. 미션 시작 약 4초 후 장애물이 발생하는지 확인한다.
6. 장애물 감지 중 Motor가 정지하는지 확인한다.
7. 약 3초 후 장애물이 해제되고 미션이 재개되는지 확인한다.

#### 관찰 결과

- `StartMission_Response(success=True, message='Mission started')`
- 상태 전이: `READY → MISSION`
- 약 4초 후 장애물 감지
- 장애물 감지 시 `mission_active=false`
- Motor target 및 current speed가 정지 방향으로 변경
- 약 3초 후 장애물 해제
- `mission_active=true`로 미션 재개

#### 판정

**PASS**

---

### 6.2 Demo #2: Low Battery 및 Limp Mode

#### 절차

1. GUI Battery Simulator에서 SOC를 15%로 설정한다.
2. Battery 상태를 발행한다.
3. Vehicle State가 LOW_BATTERY로 전이되는지 확인한다.
4. Start Mission을 요청한다.
5. `LOW_BATTERY / ACTIVE` 상태와 0.1 m/s 동작을 확인한다.

#### 관찰 결과

- SOC 20% 이하에서 LOW_BATTERY 진입
- Low Battery 상태에서 미션 시작 요청 성공
- 응답 메시지:
  `Low battery mission started in limp mode`
- Vehicle State:
  `state: 3`, `mission_active: true`
- Motor limp mode 목표 속도: 0.1 m/s
- 기존 Battery SOC 0→100 순환으로 발생하던 Security 오탐 제거

#### 판정

**PASS**

---

### 6.3 Demo #3: ECU Failure

#### 절차

1. 전체 시스템이 READY 상태인지 확인한다.
2. Sensor ECU 프로세스를 종료한다.
3. Diagnostics Event와 Vehicle State를 관찰한다.
4. Motor 상태가 정지인지 확인한다.

#### 관찰 결과

- Sensor ECU 마지막 Heartbeat 이후 약 3초에 timeout 발생
- Diagnostics Event:
  `Required ECU timeout: sensor_ecu, elapsed=3.0s`
- Event severity: ERROR
- Vehicle State 전이: `READY → EMERGENCY`
- Diagnostics fault 조회 결과:
  `sensor_ecu: Required ECU timeout`
- Motor Emergency Stop 수행

#### 판정

**PASS**

---

### 6.4 Demo #4: Cyber Attack

#### 절차

1. 전체 시스템이 READY 상태인지 확인한다.
2. 별도 터미널에서 Attack Node를 실행한다.
3. Security Event를 확인한다.
4. Vehicle State와 Motor 상태를 확인한다.

#### 실행 명령

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run attack_node attack_node
```

#### 관찰 결과

- Attack Node가 SOC 150% 메시지를 발행
- Security Event:

```text
attack_type: BATTERY_SOC_RANGE
severity: 2
description: Invalid battery SOC: 150.0%
```

- Vehicle State:

```text
state: 5
mission_active: false
```

- Motor Status:

```text
target_linear: 0.0
current_linear: 0.0
target_angular: 0.0
current_angular: 0.0
enabled: false
```

- Diagnostics fault 저장:
  `vehicle_manager: Vehicle state changed to EMERGENCY`

#### 판정

**PASS**

---

## 7. 빌드 및 자동 테스트 결과

### 7.1 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

결과:

```text
Summary: 10 packages finished
```

전체 10개 패키지가 정상적으로 빌드되었다.

### 7.2 테스트

```bash
colcon test
colcon test-result --verbose
```

결과:

```text
Summary: 25 tests, 0 errors, 0 failures, 7 skipped
```

`sdv_test_gui` 테스트 실행 중 multi-threaded process의 `fork()` 사용과
관련된 경고가 출력되었으나 테스트 실패는 발생하지 않았다.

---

## 8. 시스템 실행 및 재현 방법

### 8.1 전체 시스템 실행

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sdv_bringup sdv_system.launch.py
```

GUI가 없는 환경에서는 다음과 같이 실행한다.

```bash
ros2 launch sdv_bringup sdv_system.launch.py gui:=false
```

### 8.2 Mission 시작

```bash
ros2 service call /ecu/vehicle/start_mission \
  sdv_interfaces/srv/StartMission '{}'
```

### 8.3 Diagnostics fault 조회 및 초기화

```bash
ros2 service call /ecu/diagnostics/get_fault_info \
  sdv_interfaces/srv/GetFaultInfo '{}'

ros2 service call /ecu/diagnostics/clear_fault \
  sdv_interfaces/srv/ClearFault "{fault_name: all}"
```

### 8.4 Action 실행

```bash
ros2 action send_goal /go_to_target \
  sdv_interfaces/action/GoToTarget \
  "{x: 1.0, y: 0.0}" --feedback

ros2 action send_goal /return_home \
  sdv_interfaces/action/ReturnHome \
  "{start: true}" --feedback
```

---

## 9. 구현 개선 사항

통합 시연 준비 과정에서 다음 문제를 확인하고 수정하였다.

| 기존 문제 | 개선 결과 |
|---|---|
| Battery SOC가 0%에서 100%로 순환하며 Security 오탐 발생 | SOC가 0%에서 유지되도록 변경 |
| Battery 감소 속도가 빨라 시연 중 자동 Low Battery 진입 | 기본 감소량을 초당 0.1%로 조정 |
| SOC 변화율 검사와 GUI 수동 입력이 충돌 | 명확한 데이터 범위 중심 탐지로 정리 |
| Attack Node 실행 코드가 없음 | SOC 150% 공격 노드 구현 |
| Attack 메시지가 DDS discovery 전에 유실될 수 있음 | 구독자 연결을 확인한 뒤 공격 발행 |
| 정상 미션보다 장애물 발생 시점이 늦음 | 미션 시작 기준 4초 후 장애물 발생 |
| Diagnostics Service가 인터페이스만 존재 | 조회 및 초기화 Server 구현 |
| Action이 인터페이스만 존재 | Motor ECU Action Server 구현 |
| Timeout Event가 매초 반복됨 | 최초 timeout 시 한 번만 fault 발행 |
| ROS 종료 시 불필요한 traceback 출력 | 정상 종료 예외 처리 추가 |

---

## 10. 제한 사항 및 향후 과제

### 10.1 실제 하드웨어

`HwSensorDriver`와 `HwMotorDriver`는 교체 가능한 구조를 제공하지만,
Mentor Pi M1의 실제 센서 및 모터 API와 직접 연결된 상태는 아니다.

실물 시연을 위해 다음 작업이 추가로 필요하다.

- Mentor Pi Motor Driver API 연동
- LiDAR 또는 Ultrasonic Sensor 연동
- 실제 하드웨어 Emergency Stop 검증
- 시뮬레이션과 실물 모드의 설정 파일 분리

### 10.2 자동화 테스트

현재 자동 테스트는 주로 정적 검사로 구성되어 있다. 기능 회귀 방지를
위해 다음 통합 테스트를 추가하는 것이 적절하다.

- Launch Testing 기반 상태 전이 테스트
- Heartbeat timeout 시간 검증
- 장애물 정지 및 재개 테스트
- Security Event 발생 테스트
- Action cancel 및 Emergency abort 테스트

### 10.3 상태 복구

EMERGENCY 상태는 안전을 위해 자동 복구되지 않는다. 실제 제품 수준의
운영을 위해서는 인증된 Reset Service, fault 원인 제거 확인 및
운영자 승인 절차가 필요하다.

---

## 11. 최종 결론

README에 정의된 FR-001~FR-009 및 CSR-001~CSR-005의 소프트웨어
시뮬레이션 요구사항을 구현하였다.

정상 주행, Low Battery, ECU Failure, Cyber Attack의 네 가지 시연
시나리오를 실제 ROS 2 실행 환경에서 검증했으며 모두 PASS로
판정하였다.

전체 10개 패키지가 빌드되었고 자동 테스트 결과는
`25 tests, 0 errors, 0 failures`이다.

따라서 현재 플랫폼은 ROS 2 기반 분산 ECU 구조, Fault Handling 및
Cyber Security 동작을 소프트웨어 시뮬레이션으로 시연할 수 있는
상태이다. Mentor Pi M1 실물 연동은 후속 하드웨어 통합 과제로 남는다.
