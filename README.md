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

### 2. Create the environment with Mamba (or Conda)
If you don't have **Mamba**, install it first:
```bash
conda install mamba -n base -c conda-forge
```

Then create and activate the environment:
```bash
mamba create -n dobot-ros2 python=3.10
mamba activate dobot-ros2
```

### 3. Install ROS 2 Humble
Follow the official instructions for your OS:

👉 [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html)

After installation, don't forget to source the setup file in every new terminal:
```bash
source /opt/ros/humble/setup.zsh
```

### 4. Clone the Dobot repository
```bash
git clone https://github.com/jkaniuka/magician_ros2
cd magician_ros2
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
ros2 launch magician_ros2 bringup.launch.py
```

✨ **Ready to go!** Your Dobot ROS2 environment is now set up and ready for use.

---

⚠️ **Note:** This project is being developed on **Manjaro Linux**.  
> Some commands may vary depending on your operating system.  
> Please refer to the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for details specific to your OS.

---
 ## 🦾 Optional but Mandatory Step

Every robotic arm needs a name — it’s the law of robotics (well, almost ).  
In this project, the Dobot Magician was baptized as **Ismaildo**.  
Feel free to choose your own… but beware, unnamed robots may refuse to cooperate!
