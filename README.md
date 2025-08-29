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
- **Environment Manager:** Mamba (or Conda)  

Simulation/Testing Tools:
- **RViz:** 3D visualization of the robot and workspace  
- **Dobot Studio:** Workspace analysis and trajectory validation  
- **(Future)** Kinect or other RGB-D camera for object recognition  

---

## 🚀 How to Run

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-username/dobot-magician-ros2-research.git
   cd dobot-magician-ros2-research
