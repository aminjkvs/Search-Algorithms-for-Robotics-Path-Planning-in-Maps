# Depth-First Search (DFS) Algorithm  

## Overview  

Depth-First Search (DFS) is a fundamental algorithm used for traversing or searching tree or graph data structures. It starts at the root (or an arbitrary node) and explores as far as possible along each branch before backtracking. This method utilizes a stack data structure, either explicitly or through recursion, to keep track of the path taken.  

## Algorithm Steps  

1. **Start at the root node** (or any arbitrary node if it's a graph).  
2. **Mark the node** as visited.  
3. **Explore each adjacent unvisited node**, recursively applying the same steps.  
4. **Backtrack** when you reach a dead end, returning to the previous node to explore alternatives.  

## Usage in Robotics  

In robotics, the DFS algorithm can be employed in various applications, including:  

- **Pathfinding**: DFS can be used to navigate through environments or mazes. It helps robots determine the path to a target location, especially in unstructured environments.  
  
- **Exploration**: Robots can use DFS to explore areas by traversing through accessible nodes, identifying obstacles, and mapping unknown spaces.  

- **Search Problems**: When tasked with finding an object or navigating a complex space, DFS can efficiently traverse through paths to find the goal.  

## Usage Example  


https://github.com/user-attachments/assets/c4b187cb-ba2b-4d01-b1af-ec9119fe774a

