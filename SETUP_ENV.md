# Git Setting

# Directory

# WSL & Ubuntu


# ROS2

1. WSL 터미널
```Bash
lsb_release -a
uname -m // CPU Architecture , expect = x86_64

sudo apt update
sudo apt upgrade -y

// ROS 저장소 준비
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

// 키 등록
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

// 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

// 설치
sudo apt install ros-jazzy-desktop -y
```

