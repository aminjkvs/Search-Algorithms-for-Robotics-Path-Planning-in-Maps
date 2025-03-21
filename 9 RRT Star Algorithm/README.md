# RRT* (Rapidly-exploring Random Tree Star) Path Planning Algorithm

## Overview

The **Rapidly-exploring Random Tree Star (RRT*)** algorithm is an optimized version of RRT that guarantees **path optimality**. While the original RRT algorithm efficiently finds a feasible path in complex environments, RRT* improves it by refining the tree structure through **rewiring** and **cost optimization**. This ensures that the algorithm converges to the optimal path as the number of iterations increases.

RRT* is **asymptotically optimal**, meaning that the longer the algorithm runs, the closer it gets to finding the best possible path.

---

## Key Features

- **Path Optimality**: RRT* guarantees the shortest or least-cost path.
- **Probabilistically Complete**: Finds a solution if one exists.
- **Rewiring**: Continuously improves the tree structure for better paths.
- **Obstacle Avoidance**: Navigates through environments with obstacles.
- **High-Dimensional Spaces**: Handles complex spaces such as robotic arms with multiple degrees of freedom.

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
4. **Rewiring**:
   - For each new node, check nearby nodes in a specified radius.
   - If a cheaper path to nearby nodes exists, update their parent and cost.
5. **Check Goal Condition**:
   - If the new node is close to the goal, connect it and terminate.
6. **Reconstruct the Path**:
   - Traverse the tree backward through parent nodes to reconstruct the path.

---

## Applications

RRT* is widely used in robotics and other fields for solving pathfinding and motion planning problems. Common applications include:

- **Robotics Navigation**: Optimal pathfinding for autonomous vehicles and drones.
- **Motion Planning**: Trajectory planning for robotic arms, manipulators, and dynamic systems.
- **Game Development**: AI navigation for characters in dynamic environments.
- **High-Dimensional Spaces**: Planning in multi-degree-of-freedom systems.

---

## Repository Contents

This repository contains:
- **`rrt_star_algorithm.m`**: MATLAB implementation of the RRT* algorithm.
- **Example Map**: A sample 2D occupancy grid for testing the algorithm.
- **Visualization**: Real-time tree expansion, rewiring, and path reconstruction.

---

## Usage


https://github.com/user-attachments/assets/10b53bca-4c10-4409-90dc-50e5f2de7649

![RRT_Star_Path_Planning](https://github.com/user-attachments/assets/d73117c8-3383-42c1-a47b-994b9181a866)

