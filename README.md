# A ROS2 application designed for the offboard control of a PX4 UAV
## Overview
This tutorial explains how to integrate **ROS2** and **PX4** to control a simulated **UAV** using velocity. An **XBOX** controller is used for user controls. The **UAV** system is simulated with either **Gazebo** or **JMAVSim**.

The primary objective is to develop a **Python** example that anyone can follow, understand, and utilize comprehensively, based on **ROS2** and **PX4**.

## YouTube
To demonstrate the project's functionality and provide some gentle guidance, a short video was published on YouTube. You can find the link below:

[![Watch the video](https://img.youtube.com/vi/__MgYtL8G18/0.jpg)](https://www.youtube.com/watch?v=__MgYtL8G18)

## The application
Essentially, the entire project consists of two nodes. 

The first node controls the **PX4** autopilot through the **Micro XRCE-DDS bridge**. Commands are sent using speed information, but the code for position control also exists in the application and can be used as is. 

The second node receives information from the XBOX controller via the Python pygame library. Communication between nodes occurs through two topics: __/offboard_velocity_cmd__ (of Twist type – used for sending velocity commands) and __/action_message__ (of UInt8 data type).

In the [xbox.py](https://github.com/dmdobrea/ROS2_offboard_control/blob/main/offboard_control/offboard_control/xbox.py) file (used to interface with Xbox controller), the variable __m3_vs_m2__ sets the mode mapping of the Xbox’s joysticks. If __m3_vs_m2 = 0__, **Mode 2** is active. In the case of __m3_vs_m2 = 1__, **Mode 3** is active. See the following figures.

<p align="center">
  <img src="Images/mode2.png" width="300"/>
  <img src="Images/mode2.png" width="300"/>
</p>

This repo is based on: 
* [Jaeyoung Lim's Offboard example](https://github.com/Jaeyoung-Lim/px4-offboard) and
* [ARK Electronics example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example)

## Prerequisites
* **ROS2 Humble**
* **PX4 Autopilot** (the application was tested on the latest version 1.16.0 at the moment of writing this text, September 6, 2025)
* **Micro XRCE-DDS Agent**
* **px4_msgs**
* **Ubuntu 22.04**
* **Python** - tested on 3.10

## Setup Steps
All the steps needed to install and configure the various components required to support this application are outlined as follows:

### Install PX4
You only need to install **PX4** if you require the simulator, as we do in this guide. If you're working on a real **UAV** system, the **PX4 autopilot** is already installed on your drone.
Set up a **PX4** development environment on Ubuntu like this:

```
$ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
$ cd PX4-Autopilot/

$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Please be advised that the aforementioned commands will also install the recommended simulators corresponding to your version of Ubuntu.

### Install ROS 2 
Install **ROS2** and its dependencies:

```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```
The instructions provided above are reproduced from the official installation guide:

[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

One may install either the desktop version (ros-humble-desktop), the minimalist version (ros-humble-ros-base), or the development tools (ros-dev-tools).
Specific Python dependencies are also required to be installed (using pip or apt):

```
$ pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools
```
### Set up Micro XRCE-DDS Agent & Client





