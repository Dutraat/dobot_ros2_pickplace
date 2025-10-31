# 🤖 Robotic Manipulation Scenario with Dobot Magician + ROS 2

This repository is part of a **research project in collaborative robotics**, aiming to develop an **experimental robotic manipulation scenario** using the **Dobot Magician** robotic arm integrated with **ROS 2 (Humble)** and **Python**.

In this experiment, the Dobot Magician will be trained to recognize objects moving on a **conveyor belt**, determine their positions within its workspace, and autonomously perform **pick-and-place** operations, depositing them in predefined locations.  

The system combines **physical robotics**, **computer vision**, **simulation in RViz/Dobot Studio**, and intelligent control algorithms, serving as a foundation for research in **automation and robot learning**.

---

## 🎯 Project Objectives

- Create an **experimental robotic scenario** with the Dobot Magician.  
- Implement object recognition for items moving on a conveyor belt.  
- Estimate object positions within the robot’s workspace using sensors/computer vision.  
- Program the Dobot to perform **pick-and-place** according to sorting criteria.  
- Use **ROS 2 + Python** as the integration and control framework.  
- Validate experiments in **simulation (RViz, Dobot Studio)** before running on real hardware.  

---

## ⚙️ Development Environment

- **Operating System:** Manjaro Linux  
- **ROS 2:** Humble Hawksbill  
- **Language:** Python 3.10+  
- **Environment Manager:** Mamba   

Simulation/Testing Tools:
- **RViz:** 3D visualization of the robot and workspace  
- **Dobot Studio:** Workspace analysis and trajectory validation  
- **(Future)** Kinect or other RGB-D camera for object recognition  

---

# 🚀 How to Run

Follow the steps below to set up and run the project.

### 1. Install Python
Make sure you have **Python 3.10+** installed.

Check your version:
```bash
python3 --version
```
### 2. Install ROS 2 Humble
Follow the official instructions for your OS:

👉 [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html)

### 3. Create the environment 
If you don't have **ROS 2**, install it first:

Follow the official instructions for your OS:

👉 [Create the environment](https://robostack.github.io/GettingStarted.html)

After installation, don't forget to source the setup file in every new terminal:
```bash
source /opt/ros/humble/setup.zsh
```

### 4. Clone the Dobot repository
> 🦾 You need two repositories — one for the ROS 2 integration and another for this research project’s scripts.
```bash
git clone https://github.com/jkaniuka/magician_ros2
cd magician_ros2
```
Then clone this project repository, which contains the scripts and configurations for the pick-and-place scenario:
```bash
git clone git@github.com:Dutraat/dobot_ros2_pickplace.git
```
### 5. Install dependencies
Inside your environment, install the Python dependencies:
```bash
pip install -r requirements.txt
```

### 6. Build the workspace
From the root of the repository:
```bash
colcon build
source install/setup.zsh
```

### 7. Run the system
Example (launch files will be provided):
```bash
ros2 launch dobot_bringup dobot_magician_control_system.launch.py
```

⚠️ **Note:** This project is being developed on **Manjaro Linux**.  
> Some commands may vary depending on your operating system.  
> Please refer to the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for details specific to your OS.

---

# 🧠 System Overview

The current system integrates the **Dobot Magician** robotic arm with the **ROS 2 Humble** ecosystem, enabling control and communication between hardware and software components.  
The project’s goal is to create an experimental scenario for **object manipulation** using ROS 2 nodes, topics, and services.

Below is a simplified diagram of the system’s structure and data flow:



## 🗂️ Architecture Diagram

<p align="center">
  <img src="https://github.com/user-attachments/assets/ff0a9144-458d-431d-b85d-d35860501129" alt="System Diagram" width="700"/>
</p>


> Figure 1 — Overview of the current ROS 2 + Dobot Magician ecosystem. For a detailed technical explanation of the system, check the [Technical Study]().


## ⚙️ System Description

The system is composed of multiple interconnected modules:

- **ROS 2 Nodes:** Handle robot control, command publishing, and data exchange.  
- **Dobot Interface (magician_ros2):** Provides low-level communication between ROS 2 and the robotic arm.  
- **Pick-and-Place Scripts:** Implement motion planning and execution for object manipulation.  
- **Workspace Definition:** Defines safe areas, limits, and target positions for pick-and-place operations.  
- **Simulation Tools:** Used to validate motion and workspace boundaries before executing on real hardware.

---

## 🧾 Script Development

So far, the following scripts have been developed and tested:

- `move_linear.py` → Executes linear movements between Cartesian coordinates.  
- `move_relative.py` → Implements relative motion commands based on current position.  
- `dobot_diagnostic.py` → Performs a full system check on the Dobot Magician, verifying communication, calibration, and current position limits to ensure the robot is within its safe workspace.   

Each script is modular and can be executed individually or integrated into a combined motion routine.

---

## 🔄 Current Progress

✅ ROS 2 environment successfully configured  
✅ Dobot connection established and tested  
✅ Basic movement and gripper control implemented  
✅ Position validation
🔄 In progress: relative motion  
🔜 Next steps: conveyor integration and object sorting automation  

---

## 🔬 Notes

- The workspace boundaries and Cartesian coordinates are defined experimentally to avoid overextension of the Dobot’s servos.  
- Testing is initially performed using **RViz** and **Dobot Studio** for safety before executing on the physical hardware.  
- The modular node-based structure allows future integration with sensors or cameras (e.g., Kinect).

---
✨ You’re on your own now… don’t worry, the universe loves a good blooper reel.
---
 ## 🦾 Optional but Mandatory Step

Every robotic arm needs a name — it’s the law of robotics (well, almost ).  
In this project, the Dobot Magician was baptized as **Ismaildo**.  
Feel free to choose your own… but beware, unnamed robots may refuse to cooperate!

