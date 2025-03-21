# RRT (Rapidly-exploring Random Tree) Path Planning Algorithm

## Overview

The **Rapidly-exploring Random Tree (RRT)** algorithm is a powerful pathfinding and motion planning technique used in robotics and other domains. RRT is particularly effective for solving problems in high-dimensional spaces and environments with obstacles. It incrementally builds a tree by sampling random points and connecting them to the nearest node in the tree, ensuring feasible paths toward the goal.

RRT is **probabilistically complete**, meaning it will eventually find a solution if one exists, given enough time. However, it does not guarantee optimality unless extended variants like **RRT*** are used.

---

## Features

- **Probabilistically Complete**: Finds a solution if one exists.
- **Obstacle Avoidance**: Navigates through environments with obstacles.
- **High-Dimensional Spaces**: Handles complex spaces such as robotic manipulators with multiple degrees of freedom.
- **Customizable Parameters**: Adjust step size, maximum iterations, and goal thresholds to suit your environment.

---

## Algorithm Steps

1. **Setup the Environment**:
   - Define the map, obstacles, start, and goal points.
2. **Initialize the Tree**:
   - Start with the root node (start point).
3. **Iteratively Build the Tree**:
   - Randomly sample points in the space.
   - Find the nearest node in the tree to the sampled point.
   - Extend the tree toward the sampled point by a fixed step size.
   - Check for collisions with obstacles and add the new node if it is collision-free.
4. **Check Goal Condition**:
   - If the new node is close to the goal, connect it and terminate.
5. **Reconstruct the Path**:
   - Traverse the tree backward through parent nodes to reconstruct the path.

---

## Applications

RRT is widely used in robotics and other fields for solving pathfinding and motion planning problems. Common applications include:

- **Robotics Navigation**: Pathfinding for mobile robots in obstacle-rich environments.
- **Motion Planning**: Trajectory planning for robotic arms, drones, and autonomous vehicles.
- **Game Development**: AI navigation for characters in dynamic environments.
- **High-Dimensional Spaces**: Planning in multi-degree-of-freedom systems.

---

## Variants of RRT

Several variants of RRT exist to improve its performance and capabilities:

1. **RRT***:
   - Guarantees an optimal path by refining the tree as new nodes are added.
2. **Bi-directional RRT**:
   - Builds two trees simultaneously—one from the start and one from the goal—and connects them.
3. **RRT-Connect**:
   - Focuses on quickly expanding the tree toward the goal by reducing unnecessary sampling.

---

## Repository Contents

This repository contains:
- **`rrt_algorithm.m`**: MATLAB implementation of the RRT algorithm.
- **Example Map**: A sample 2D occupancy grid for testing the algorithm.
- **Visualization**: Real-time tree expansion and path reconstruction.

---

## Usage


https://github.com/user-attachments/assets/8ab989c9-1445-488e-b8fd-997107a59592

![RRT_Path_Planning2](https://github.com/user-attachments/assets/8f6450e3-3b4b-442c-a7b7-74c6781c2221)

https://github.com/user-attachments/assets/ed13709d-8adb-4f1c-a077-543cadaf0864

![RRT_Path_Planning1](https://github.com/user-attachments/assets/c07b3eee-af3e-4255-be54-a9cb6841bcc1)
