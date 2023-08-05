#!/bin/bash

# Disable system sleep and hibernation targets
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# Create and enable swap space
sudo apt install -y fallocate
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Install Git and Curl
sudo apt update
sudo apt install git
sudo apt install curl

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-core
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional ROS packages
sudo apt-get install python-is-python3 python3-rosdep python3-future python3-lxml python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs ros-noetic-cv-bridge ros-noetic-vision-opencv ros-noetic-angles -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Set up a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

# Fetch MAVLink and MAVROS packages
rosinstall_generator --rosdistro noetic mavlink mavros --deps --tar | tee /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j$(nproc)

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -y

# Additional commands
cd src/mavlink
git checkout 03f809c9

cd ../mavros
git checkout tags/1.15.0

# Build catkin_ws first
cd ..
catkin build -j1

# Clone Project_Drone in the parent directory of catkin_ws
cd ..
git clone https://github.com/PNisargkumar/Project_Drone

# Build Project_Drone package
cd Project_Drone/
catkin build

cd ../catkin_ws/
source devel/setup.bash