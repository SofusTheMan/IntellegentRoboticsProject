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
├── README.md
├── Simulation/
│   └── simulationRandomLocalIntoFinish.py
├── Worlds/
│   └── beginner_move_your_e-puck.wbt
└── controllers/
    ├── maze_generator/
    │   └── maze_generator.py
    ├── nav_controller/
    │   ├── epuck_maze_controller.py
    │   ├── Localisation.py
    │   ├── particle_filter.py
    │   ├── planner.py
    │   ├── nav_controller.py
    │   ├── motion_primitives.py
    │   ├── wall_perception.py
    │   ├── sensors.py
    │   ├── map_graph_client.py
    │   ├── maze_graph_adapter.py
    │   ├── maze_graph.json
    │   ├── maze_map.py
    │   └── simple_policy.py
    ├── nav_controllerRemake/
    │   ├── nav_controllerRemake.py
    │   ├── make_action.py
    │   ├── next_action.py
    │   ├── update_possible_positions.py
    │   ├── orientation.py
    │   ├── maze_graph_adapter.py
    │   └── maze_graph.json
    └── simple_wall_follower/
        └── simple_wall_follower.py


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
