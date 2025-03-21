# Greedy Best-First Search (GBFS) Algorithm  

## Overview  

Greedy Best-First Search (GBFS) is an informed search algorithm used for traversing or searching tree or graph data structures. It selects the node that appears to be the most promising based on a heuristic evaluation function. By always choosing the node considered to be closest to the goal, GBFS can efficiently navigate through complex search spaces but does not guarantee the shortest path.  

## Algorithm Steps  

1. **Start at the initial node** (or any arbitrary node if it's a graph).  
2. **Evaluate** the neighbors of the current node using a heuristic function.  
3. **Select the neighbor** with the best heuristic value (lowest cost).  
4. **Mark the selected node** as visited.  
5. **Repeat** the process until the goal node is reached or all nodes have been evaluated.  

## Usage in Robotics  

In robotics, the GBFS algorithm can be employed in various applications, including:  

- **Pathfinding**: GBFS is useful for navigating environments where quick decision-making is required. It helps robots select the most promising path towards their destination based on heuristic evaluations.  

- **Exploration**: Robots can use GBFS to explore areas efficiently by prioritizing paths that appear to lead to desirable locations or tasks.  

- **Search Problems**: In scenarios requiring optimization (like finding the shortest path), GBFS can rapidly traverse towards the goal, making it suitable for time-sensitive applications.  

## Usage Example 


https://github.com/user-attachments/assets/b39bed82-88ee-40aa-ad4f-0cd332dc9bab

![Greedy_Best_First_Search](https://github.com/user-attachments/assets/daaef6db-49ea-478c-99d9-aacdd4262b17)
