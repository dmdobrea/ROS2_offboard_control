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
A companion computer can connect to a **PX4 autopilot** operating on the **FMU** via various methods, including **serial**, **UDP**, **TCP**, or **CAN FD**. All of these connection methods are facilitated by a **uXRCE-DDS middleware** component, which consists of a client application deployed on the **PX4 autopilot** and an agent application running on the companion computer. Even when operating within a simulated environment, our **ROS2** application must establish a connection to the **PX4 autopilot** via the **uXRCE-DDS bridge**.

The initial step involves installing the agent onto the companion computer or the Ubuntu system (such as in the case of a simulator) from the source. Therefore, in a terminal window, build and install the agent.

```
$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
$ cd Micro-XRCE-DDS-Agent
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig /usr/local/lib/
```

Use the following to find more information for executing and configuring the agent, and connect to **PX4** (located on **FMU**) through the **uXRCE-DDS client**.

```
$ MicroXRCEAgent --help
Usage: 'MicroXRCEAgent <udp4|udp6|tcp4|tpc6|canfd|serial|multiserial|pseudoterminal> <<args>>'

Available arguments (per transport):
  * COMMON
    -h/--help.
    -m/--middleware <value> (ced, dds, rtps) [default: 'dds'].
    -r/--refs <value>.
    -v/--verbose <value> ( - ) [default: ''].
    -d/--discovery <value> [default: '7400'].
    -P/--p2p <value>.
  * IPvX (udp4, udp6, tcp4, tcp6)
    -p/--port <value>.
  * SERIAL (serial, multiserial, pseudoterminal)
    -b/--baudrate <value> [default: '115200'].
    -D/--dev <value>.  * CAN FD (canfd)
    -D/--dev <value>.
```

Now, initiate the agent with configurations designated for establishing a connection to the **uXRCE-DDS component** operating within the simulator.

```
$ MicroXRCEAgent udp4 -p 8888
```

### Start the simulator
The build system facilitates the process of constructing and initiating **PX4** on **SITL**, launching a simulator, and establishing connections. The simplified syntax appears as follows:

```
$ make px4_sitl simulator[_vehicle-model]
```

Where simulator is __gz__ (for Gazebo), __gazebo-classic__, __jmavsim__ or some other simulator, and vehicle-model is a particular vehicle type supported by that simulator.
If you encounter issues loading Gazebo, please install the following packages:










