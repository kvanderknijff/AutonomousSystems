#### AutonomousSystems Git Repository | Hogeschool Rotterdam Technische Informatica | Jaar 3 Semester 6
# Autonomous Forms Forming Forms-Forming-Bots System

Welcome to the central repository for the Autonomous Forms-Forming-Bots project. This is an end-to-end, closed-loop, multi-agent, robotic ecosystem. The system coordinates a fleet of custom autonomous rovers into dynamic structural formations (Line, Cross, Square) using overhead computer vision feedback and asynchronous MQTT communication.

This repository contains all hardware designs, embedded firmware, computer vision code, and server-side coordination algorithms.

## Repository Structure

To keep the project modular, the repository is organized into the following directories:

*   `📁 ./server/` — The central fleet "brain" containing the core coordination modules.
    *   `📁 /server_code/` — Contains code of the main server handling: Robot connections, MQTT communication & database.
        *   `📁 /web/` — Frontend web interface displaying live camera feeds and hosting user formation controls.
    *   `📁 /pathPlanning/` — Program that devides the formation into different tasks for the robots


*   `📁 camera/` — Computer vision scripts utilizing OpenCV for real-time ArUco marker tracking and LED flash detection.
    *   `detector.py`

*   `📁 firmware/` — Embedded Python code running locally on the chariots for hardware interface and MQTT telemetry.
    *   `main.py` — Main code controlling the chariot
    *   `credentials.py` — Contains credentials for WiFi etc.
    *   `nav_goal.py` — Calculates the shortest path to a target given by the server
    *   `robot_config.py` — A map of PWM signals per chariot
  
*   `📁 hardware/` — 3D-printing STL models (custom rover top brackets) and schematic references.

*   `📁 webots/` — Contains a WeBots world which is a simulation of the physical system.



---

## Hardware Overview & Specifications

The physical layer consists of custom-engineered wheeled rovers featuring a decoupled structural and control design:

*   **Chassis & Drivetrain:** A large integrated PCB acting as the main chassis, supported by two continuous-rotation servo motors for driving and a rear steel ball caster for balance.
*   **Wheel Encoders (Odometry):** Dual infrared (IR) break-beam sensors aligned with perforated wheels. These act as wheel encoders, measuring perforation speed (RPM) to locally calibrate and stabilize differential drive speeds.
*   **Top Bracket (Custom 3D-Print):** A custom-designed, 3D-printed enclosure mounted on top of the PCB. It securely houses a unique **ArUco marker** for optical tracking and dual LED signaling indicators used during network registration.

---

## System Architecture

The software is strictly decoupled using software engineering best practices (**Strategy Pattern** and **Observer Pattern**). It is fully static type-safe, validated via `mypy`.

The project is structured into modular sub-systems:
*   `path_planner.py` (**Core Coordinator**): Manages the central real-time database state machine for all discovered active robots.
*   `formations.py` (**Geometry Generator**): Computes discrete target coordinates for shapes. Easily extendable due to the Strategy Pattern.
*   `assignment.py` (**Task Allocator**): Implements global fleet cost-minimization to match robots to targets.
*   `avoidance.py` (**Reactive Navigator**): Implements local collision avoidance vector calculations.

---
