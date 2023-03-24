# Unity3D Industrial Robotics: ABB IRB 120 EGM (Externally Guided Motion)

## Requirements:

**Software:**
```bash
ABB RobotStudio, Unity3D 2021.3.16f1 (LTS), Visual Studio 2019/2022
```

**Supported on the following operating systems:**
```bash
Universal Windows Platform, Android
```

| Software/Package      | Link                                                                                  |
| --------------------- | ------------------------------------------------------------------------------------- |
| Unity3D               | https://unity3d.com/get-unity/download/archive                                        |
| ABB RobotStudio       | https://new.abb.com/products/robotics/robotstudio/downloads                           |
| Visual Studio         | https://visualstudio.microsoft.com/downloads/                                         |
| NuGetForUnity         | https://github.com/GlitchEnzo/NuGetForUnity                                           |

## Project Description:

The project focuses on a simple demonstration of client-server communication via User Datagram Protocol (UDP), which is implemented in Unity3D. More precisely, it is the control of the robot by EGM (Externally Guided Motion). The main idea is to generate the motion of the absolute positions of the robot's joints through the Unity3D development platform. An additional feature of the project is to visualization of the robot's target position (with or without the end-effector).

**WARNING: RobotWare version 6.15.01 or lower must be used.**

The 3D parts of the robot were downloaded from the official ABB website here: [ABB Library](https://library.abb.com/)

This solution can be used to control a real robot or to simulate one. The Unity3D Digital-Twin application has been tested on the ABB IRB 120 robotic arm, both on real hardware and in simulation.

The application can be installed on a mobile phone, tablet or computer, but for communication with the robot it is necessary to be in the same network. The application uses performance optimization using multi-threaded programming. The High Definition Render Pipeline (HDRP) technology has been used for better graphics.

The project was realized at the Institute of Automation and Computer Science, Brno University of Technology, Faculty of Mechanical Engineering (NETME Centre - Cybernetics and Robotics Division).

**IP Address Settings:**

|          | ABB RobotStudio (CRB 15000 GoFa) | PC |
| :------: | :-----------: | :-----------: |
| Simulation Control  | 127.0.0.1 | 127.0.0.1 |
| Real - World Control | 192.168.125.1  | 192.168.125.22 |

|          | PORT |
| :------: | :-----------: |
| UDPUC | 6511  |

**WARNING: To control the robot in the real world, it is necessary to disable the firewall.**

**Notes:**

EGM (Externally Guided Motion) is an interface for ABB robots that allows smoothless control of the robotic arm from an external application (in our case it is a Unity3D developmentpPlatform ). The EGM can be used to transfer positions to the robot controller in either Joint/ Cartesian space. In our case it is the control of the robot using Cartesian coordinates.

```bash
The file "egm.proto" can be found in the installation folder of RobotWare. For example on Windows with RobotWare 7.6.1:
C:\Users\<user_name>\AppData\Local\ABB Industrial IT\Robotics IT\RobotWare\RobotControl_7.6.1\utility\Template\EGM
```

The Protobuf code generator can be used to generate code from a *.proto file into individual programming languages.

Link: [Protobuf Code Generator and Parser](https://protogen.marcgravell.com)
