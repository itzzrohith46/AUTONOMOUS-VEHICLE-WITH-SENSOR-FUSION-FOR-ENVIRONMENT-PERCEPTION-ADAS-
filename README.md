This project demonstrates a basic Autonomous Driving System built and tested using the CARLA simulator. It integrates lane detection, radar-based object detection, and autopilot functionality under foggy weather conditions to evaluate vehicle behavior in reduced visibility scenarios. By simulating dynamic environments with AI-controlled traffic and pedestrians, this project highlights the importance of sensor fusion and simulation-based testing in developing future-ready ADAS (Advanced Driver Assistance Systems).

Project Objectives
* Simulate and analyze autonomous vehicle performance in low-visibility (foggy) conditions.

* Implement real-time lane detection using a front-facing RGB camera.

* Detect obstacles (vehicles, pedestrians) using a radar sensor.

* Test autopilot control for safe navigation amid dynamic AI traffic and pedestrian interactions.

* Provide a cost-effective, modular, and realistic virtual prototype for autonomous driving research.

Tools & Technologies Used
* CARLA Simulator 0.9.13 – Autonomous driving simulation platform

* Python 3.x – Core programming language

* OpenCV – Image processing for lane detection

* NumPy – Efficient numerical operations

* Tesla Model 3 Blueprint – Ego vehicle used in the simulation

* Radar Sensor – Object detection based on distance and velocity

* Spectator Camera – Third-person dynamic view

* Threading – Real-time spectator camera movement

* AI Traffic and Pedestrians – Simulation of real-world dynamics

Working Principle
* Launches a foggy driving environment with adjustable fog density and visibility.

* Spawns a Tesla Model 3 and enables autopilot for self-navigation.

* Attaches an RGB camera for lane detection using Canny edge detection and Hough Transform.

* Integrates a Radar sensor for detecting moving and stationary obstacles with velocity tracking.

* Spawns AI vehicles and pedestrians to create complex, real-world-like conditions.

* Uses a Spectator Camera in a separate thread for dynamic third-person monitoring.

* Continuously processes sensor data to evaluate the system’s robustness under environmental constraints.

Key Features
* Lane Detection in Fog: Successfully detects lanes even under dense fog.

* Radar-Based Obstacle Detection: Identifies objects and tracks their speed and distance.

* Dynamic Environment: AI-driven traffic and pedestrian behavior to test vehicle responsiveness.

* Spectator View: Real-time third-person camera movement for better visualization.

* Simulation of Extreme Conditions: Focused on safety and perception under low-visibility weather.

Results & Discussion

* Lane Detection Module: Performed reliably even with visibility reduction.

* Radar Detection Module: Successfully identified vehicles and pedestrians in real-time.

* Autopilot System: Managed lane-keeping and obstacle avoidance efficiently.

* Simulation Environment: Provided a realistic and controlled setup for repeated and safe testing of autonomous features.
