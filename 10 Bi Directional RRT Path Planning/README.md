# Bidirectional RRT Path Planning Algorithm

## Overview

The **Bidirectional RRT (Rapidly-exploring Random Tree)** algorithm is a path planning technique that simultaneously grows two trees: one from the **start** and one from the **goal**. These trees expand toward random points in the space, and the algorithm terminates when the two trees meet. This approach significantly reduces the time required to find a feasible path compared to single-tree RRT.

Bidirectional RRT is widely used in robotics and motion planning, especially for environments with obstacles.

---

## Key Features

- **Two Trees**: Expands one tree from the start and another from the goal, connecting them when they meet.
- **Faster Pathfinding**: Reduces computational time compared to single-tree RRT.
- **Obstacle Avoidance**: Navigates through environments with obstacles.
- **High-Dimensional Spaces**: Handles complex spaces such as robotic arms with multiple degrees of freedom.

---

## Algorithm Steps

1. **Setup the Environment**:
   - Define the map, obstacles, start, and goal points.
2. **Initialize the Trees**:
   - Start with two separate trees: one from the start and one from the goal.
3. **Iteratively Build the Trees**:
   - Randomly sample points in the space.
   - Extend the trees toward the sampled points by a fixed step size.
   - Check for collisions with obstacles and add the new nodes if they are collision-free.
4. **Connect the Trees**:
   - Check if the two trees can connect when their nodes come close to each other.
5. **Reconstruct the Path**:
   - Combine the paths from the two trees into a single path from start to goal.

---

## Applications

Bidirectional RRT is widely used in robotics and other fields for solving pathfinding and motion planning problems. Common applications include:

- **Robotics Navigation**: Pathfinding for autonomous vehicles and drones.
- **Motion Planning**: Trajectory planning for robotic arms, manipulators, and dynamic systems.
- **Game Development**: AI navigation for characters in dynamic environments.
- **High-Dimensional Spaces**: Planning in multi-degree-of-freedom systems.

---

## Repository Contents

This repository contains:
- **`bidirectional_rrt_algorithm.m`**: MATLAB implementation of the Bidirectional RRT algorithm.
- **Example Map**: A sample 2D occupancy grid for testing the algorithm.
- **Visualization**: Real-time tree expansion and path reconstruction.

---

## Usage:


https://github.com/user-attachments/assets/499b819c-4f27-4040-b687-a08506f92161

![Bidirectional_RRT_Path_Planning](https://github.com/user-attachments/assets/dbe2e70b-9014-464a-b0fa-75f14d2d56ee)
