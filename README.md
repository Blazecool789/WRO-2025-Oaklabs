Oak-Labs

This repository contains the work of Team Oaklabs, developed for the WRO 2025 Future Engineers competition. Our robot combines simplicity with advanced technical systems to ensure efficiency, reliability, and adaptability across all phases of the challenge.

Members:
Supratiik Koppuravuri
Aneesh Kulkarni
Krishna Hanumanthu

Project/bot description :

Our robot is designed around a distributed control system that uses two microcontrollers: an Arduino Uno and a Raspberry Pi 5.

The Arduino Uno handles real-time actuation, low-latency sensor processing, and precise motor control.

The Raspberry Pi 5 performs high-level decision-making, camera-based vision processing, and computationally intensive tasks.

Key hardware components include:

Arduino subsystem: motor driver shield, brushed DC motor, high-torque servo motor, 2 × HC-SR04 ultrasonic sensors.

Raspberry Pi subsystem: wide-angle camera module, 1 × ultrasonic sensor for redundancy.

The robot is capable of obstacle detection, vision-based navigation, line pattern recognition, and a parallel parking maneuver. By combining camera data with ultrasonic readings, the system achieves reliable sensor fusion and fast decision-making.

Table of contents :

team-photos : photos of each team member (formal & informal)

bot-photos : multiple views of the robot (front, back, left, right, top, bottom)

diagrams and schemes : visual representations of system architecture, wiring, and component interconnections

materials and components : detailed list of all hardware and electronics used

3D models : CAD designs and animations of the robot build

reference documents : reports/manuals covering design decisions and logistics

TeamCode : complete codebase with explanations for each competition phase

videos : demonstration of the robot performing different tasks

Introduction

Team Oaklabs proudly presents its robot for the World Robot Olympiad 2025 (Future Engineers).

Our design philosophy emphasizes efficiency, precision, and modularity. By distributing responsibilities between the Arduino and Raspberry Pi, the robot executes movements with speed while simultaneously processing complex vision-based tasks.

Highlights of our approach:

Distributed architecture: separates time-critical motor control from computationally heavy tasks.

Sensor fusion: combines ultrasonic data with camera vision for robust navigation.

Adaptive behavior: the robot dynamically switches strategies between obstacle navigation, line-following, and parallel parking.

Deterministic communication protocol: ensures that commands from the Raspberry Pi to the Arduino translate into predictable actions.

Modularity: components can be easily replaced or upgraded, making the system flexible and competition-ready.

By balancing advanced algorithms with practical engineering, Oaklabs has built a robot capable of tackling every stage of the competition with speed, consistency, and adaptability.
