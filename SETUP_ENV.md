# Git Setting

```bash
# Git 설치
sudo apt install git -y

# 사용자 정보 설정
git config --global user.name "username"
git config --global user.email "emailId@gmail.com"

# 확인
git config --list

# Repository Clone
cd ~/dev
git clone <repo_url> sdv_platform_ws

# Repository 생성 후 연결
git init
git remote add origin <repo_url>

# Push
git add .
git commit -m "Initial Commit"
git push -u origin main
```

---

# Directory

```text
/home/username/

└── dev/
    └── sdv_platform_ws/
        │
        ├── src/
        │   ├── vehicle_manager/
        │   ├── battery_ecu/
        │   ├── sensor_ecu/
        │   ├── motor_ecu/
        │   ├── diagnostics_ecu/
        │   ├── security_ecu/
        │   ├── attack_node/
        │   └── sdv_interfaces/
        │
        ├── docs/
        │
        ├── .gitignore
        ├── README.md
        │
        ├── build/      # Git Ignore
        ├── install/    # Git Ignore
        └── log/        # Git Ignore
```

---

# WSL & Ubuntu

## WSL 설치

PowerShell (관리자 권한)

```powershell
wsl --install
```

---

## 설치 가능한 Ubuntu 확인

```powershell
wsl --list --online
```

---

## Ubuntu 24.04 설치

```powershell
wsl --install Ubuntu-24.04
```

---

## 설치된 WSL 확인

```powershell
wsl -l -v
```

---

## Ubuntu 접속

```powershell
wsl -d Ubuntu-24.04
```

---

## Ubuntu 버전 확인

```bash
cat /etc/os-release
```

Expected

```text
VERSION_ID="24.04"
UBUNTU_CODENAME=noble
```

---

## VSCode 연동

VSCode Extension

```text
WSL (Microsoft)
Python
C/C++
ROS
Git Graph
Error Lens
```

설치

---

WSL 터미널에서

```bash
code .
```

실행

또는

```text
VSCode
↓
좌측 하단 >< 클릭
↓
Connect to WSL
↓
Ubuntu-24.04 선택
```

---

# ROS2

## WSL 터미널

```bash
lsb_release -a          # expect = Ubuntu 24.04
uname -m                # expect = x86_64

# Locale
sudo apt update
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Universe Repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# ROS Repository Key
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS Repository 등록
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 패키지 목록 갱신
sudo apt update

# ROS2 Jazzy 설치
sudo apt install ros-jazzy-desktop -y

# 환경변수 등록
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 설치 확인
ros2

# 개발 툴 설치
sudo apt install python3-colcon-common-extensions -y

# 확인
colcon --help

# Demo Test
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

---

# ROS2 Workspace 생성

```bash
mkdir -p ~/dev/sdv_platform_ws/src

cd ~/dev/sdv_platform_ws

colcon build

source install/setup.bash
```

---

# 자주 사용하는 ROS2 명령어

```bash
# Node 조회
ros2 node list

# Topic 조회
ros2 topic list

# Service 조회
ros2 service list

# Action 조회
ros2 action list

# Topic 모니터링
ros2 topic echo /topic_name

# Topic 발행
ros2 topic pub

# Interface 조회
ros2 interface list
```
