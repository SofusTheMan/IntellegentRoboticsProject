## Simultaneous Maze Navigation and Localization Robot

# Project Overview

This project addresses a fundamental problem in robotics: enabling an autonomous robot to navigate and localize within a known environment but with an unknown starting position.

The system integrates various robotic functionalities—perception, localization, path planning, and control—to allow an e-puck robot to traverse a maze, determine its location using a Particle Filter, and calculate the optimal path to a goal.

#Objectives

Localization: Autonomously determine position within the maze using sensor data and particle filtering.

Navigation: Efficiently reach a designated goal state.

Integration: Combine probabilistic localization with deterministic path planning to maximize the probability of success.

Robustness: Maintain performance under varying levels of sensor noise and maze complexity.

# Tech Stack & Simulation

Simulation Environment: Webots

Language: Python

Robot Model: E-puck

Key Algorithms:

Particle Filter (MCL): For estimating the robot's pose $(x, y, \theta)$.

Dijkstra's Algorithm: For finding the shortest path on the graph representation of the maze.

# Project Structure

Based on the codebase organization:

IntellegentRoboticsProject/
├── Simulation/
├── Worlds/
└── controllers/
    ├── maze_generator/
    └── nav_controller/
        ├── epuck_maze_controller.py
        ├── particle_filter.py
        ├── path_planner.py
        ├── nav_controller.py
        ├── motion_primitives.py
        ├── wall_perception.py
        ├── map_graph_client.py
        └── sensors.py


# Getting Started

Prerequisites

Webots: Ensure you have Webots installed (compatible with Python controllers).

Python 3.x: Required for running the controller scripts.

Installation & Running

Clone the repository: git clone https://github.com/SofusTheMan/IntellegentRoboticsProject.git

Open Webots.

Load the world file from the Worlds/ directory.

Ensure the E-puck robot's controller points specifically to the epuck_maze_controller.py wrapper.

Run the simulation.


## TEAM:
Abdullahi A.
Anas k.
Sofus J.
