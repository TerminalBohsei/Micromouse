# Micromouse
USYD Micromouse competition

## Overview
A self-guiding robot designed to navigate a maze-course via **Time of Flight (ToF)** sensors and **gyroscope** sensors to determine its surrounding, orientation, and position. Sensor data is then processed by an **ESP32**, which uses an algorithm that prioritises following the left-hand wall to guide the robot through the course. Where, the end-zone is indicated by a different colour.

Objective:
The primary goal of the competition is for each team's robot to autonomously navigate to the end-zone within the least amount of time taken. 

** Competition Rules:**
Robot can only attempt the course **ONCE**.

CANNOT have detachable components - no projectiles.

Physical contact with the course results in time penalties.

**Robot Dimensions:**

Area Limit: 15x15cm

Height Limit: 100cm

Maze: 2x2m 

## Hardware
- **1x** ESP32
- **2x** DC Motors
- **1x** Motor Speed Control (L298N)
- **3x** ToF Sensors 
- **1x** IR Sensor 
- **1x** Gyro (MPU-6050) 
- **1x** 9V Battery
- 3D Printed Chassis

## Software
Language: C++

Platform: Arduino

IDE: Arduino IDE

## Repository Structure
src/

include/

hardware/

docs/

tests/
