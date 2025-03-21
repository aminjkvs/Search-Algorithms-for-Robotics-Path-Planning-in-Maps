# Uniform Cost Search (UCS) Algorithm  

## Overview  

Uniform Cost Search (UCS) is an informed search algorithm that expands the least-cost node first. It is a special case of Dijkstra's algorithm that guarantees finding the optimal solution in scenarios where path costs can vary. UCS uses a priority queue to explore paths based on their cumulative cost, making it suitable for weighted graphs.  

## Algorithm Steps  

1. **Start at the initial node** (or any arbitrary node if it's a graph).  
2. **Initialize a priority queue** with the starting node, setting its cost to zero.  
3. **While the queue is not empty**:  
   - Dequeue the node with the lowest cumulative cost.  
   - If it is the goal node, return the path and cost.  
   - Otherwise, explore its neighbors, calculating the cumulative cost to reach each neighbor.  
   - If a neighbor has not been visited or if a cheaper path to it is found, enqueue the neighbor with its cost.  

## Usage in Robotics  

In robotics, the UCS algorithm can be employed in various applications, including:  

- **Pathfinding**: UCS can determine the optimal path for robots in environments where different paths have different costs, such as terrain difficulty or obstacles.  

- **Resource Management**: Robots operating in dynamic environments can use UCS to allocate resources efficiently while navigating through various obstacles.  

- **Search Problems**: UCS is effective for finding the least costly route in scenarios where cost is a crucial factor, such as in delivery logistics or search-and-rescue missions.  

## Usage Example  


https://github.com/user-attachments/assets/efe0ceb5-1cd7-4cfa-ac33-e0e7b91226f2

![UniformCost](https://github.com/user-attachments/assets/95550c43-c6aa-4769-99f7-ca3cc788c968)
