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
INIT
READY
MISSION
LOW_BATTERY
FAULT
EMERGENCY
```

---

### Motor ECU

#### 역할

* 속도 명령 수신
* 이동 수행
* 현재 속도 상태 보고

#### 제어 대상

* Mentor Pi M1 모터

---

### Sensor ECU

#### 역할

* 장애물 감지
* 거리 측정
* 센서 데이터 제공

#### 입력

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
  └─> /battery/status

Sensor ECU
  └─> /obstacle/info

Motor ECU
  └─> /motor/speed

Vehicle Manager
  └─> /vehicle/state
```

---

### Service

요청/응답 기반 통신

#### 사용 예

```text
/get_fault_info

/clear_fault

/calibrate_sensor
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

### /obstacle/info

#### Publisher

Sensor ECU

#### Subscriber

* Vehicle Manager ECU
* Security ECU

#### 데이터

```text
Distance
Direction
```

---

### /cmd_vel

#### Publisher

Vehicle Manager ECU

#### Subscriber

Motor ECU

#### 데이터

```text
Linear Velocity
Angular Velocity
```

---

### /vehicle/state

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

### /heartbeat

#### Publisher

모든 ECU

#### Subscriber

Diagnostics ECU

#### 데이터

```text
ECU Name
Timestamp
```

---

### /security/event

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

### /get_fault_info

#### Client

Vehicle Manager ECU

#### Server

Diagnostics ECU

#### 기능

현재 Fault 정보 조회

---

### /clear_fault

#### Client

Vehicle Manager ECU

#### Server

Diagnostics ECU

#### 기능

Fault 초기화

---

### /calibrate_sensor

#### Client

Vehicle Manager ECU

#### Server

Sensor ECU

#### 기능

센서 캘리브레이션 수행

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

Motor ECU는 속도 명령을 수신해야 한다.

---

### FR-005

Vehicle Manager는 SOC ≤ 20% 조건에서 LOW_BATTERY 상태로 진입해야 한다.

---

### FR-006

Vehicle Manager는 장애물 감지 시 정지 명령을 수행해야 한다.

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
/battery/status
/cmd_vel
/vehicle/state
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

---

### Demo #2 : Low Battery

```text
SOC = 15%

↓

LOW_BATTERY

↓

Return Home

↓

Stop
```

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

---

## 11. 개발 단계

### Phase 1 (MVP)

* Vehicle Manager ECU
* Battery ECU
* Sensor ECU
* Motor ECU
* Diagnostics ECU
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
