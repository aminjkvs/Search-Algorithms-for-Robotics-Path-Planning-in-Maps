# Breadth-First Search (BFS) Algorithm  

## Overview  

Breadth-First Search (BFS) is a fundamental algorithm used for traversing or searching tree or graph data structures. It starts at the root (or an arbitrary node) and explores the neighbor nodes at the present depth prior to moving on to nodes at the next depth level. This method utilizes a queue data structure to keep track of the nodes that need to be explored.  

## Algorithm Steps  

1. **Start at the root node** (or any arbitrary node if it's a graph).  
2. **Mark the node** as visited and enqueue it.  
3. **Dequeue a node** from the front of the queue to explore it.  
4. **Explore each adjacent unvisited node**, marking it as visited and enqueuing it.  
5. **Repeat** steps 3 and 4 until the queue is empty.  

## Usage in Robotics  

In robotics, the BFS algorithm can be employed in various applications, including:  

- **Pathfinding**: BFS can be used to find the shortest path in unweighted graphs, making it useful for navigation tasks in robots.  
  
- **Exploration**: Robots can utilize BFS to systematically explore an environment, ensuring that all areas are visited while efficiently identifying obstacles. 

- **Search Problems**: BFS is effective in scenarios where the shortest path to a target is desired, such as in search-and-rescue operations or when navigating cluttered spaces.  

## Usage Example  



https://github.com/user-attachments/assets/d85d0f41-f834-4b2d-b153-d84efe6d0c3c

![BreadthFirstSearchBFS](https://github.com/user-attachments/assets/db1ea4b9-dc81-45cc-a1cd-2ed795c9cd04)
