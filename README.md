# A ROS2 application designed for the offboard control of a PX4 UAV
## Overview
This tutorial explains how to integrate **ROS2** and **PX4** to control a simulated **UAV** using velocity. An **XBOX** controller is used for user controls. The **UAV** system is simulated with either **Gazebo** or **JMAVSim**.

The primary objective is to develop a **Python** example that anyone can follow, understand, and utilize comprehensively, based on **ROS2** and **PX4**.

## YouTube
To demonstrate the project's functionality and provide some gentle guidance, a short video was published on YouTube. You can find the link below:

[Watch on YouTube](https://youtu.be/__MgYtL8G18)

## The application
Essentially, the entire project consists of two nodes. 

The first node controls the **PX4** autopilot through the **Micro XRCE-DDS bridge**. Commands are sent using speed information, but the code for position control also exists in the application and can be used as is. 

The second node receives information from the XBOX controller via the Python pygame library. Communication between nodes occurs through two topics: * */offboard_velocity_cmd* * (of Twist type – used for sending velocity commands) and * */action_message* * (of UInt8 data type).
