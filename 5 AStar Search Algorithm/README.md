# A* Search Algorithm  

## Overview  

A* Search Algorithm is a popular and powerful pathfinding and graph traversal algorithm that finds the least-cost path from a start node to a goal node. It combines features of both Uniform Cost Search and Greedy Best-First Search by using a heuristic to guide its search while also considering the cost to reach each node. This makes A* both complete and optimal, provided that the heuristic is admissible (never overestimates the true cost to reach the goal).  

## Algorithm Steps  

1. **Initialize** an open list (priority queue) containing the start node with its cost.  
2. **Initialize** a closed list to keep track of visited nodes.  
3. **While the open list is not empty**:  
   - Remove the node with the lowest estimated total cost \( f(n) = g(n) + h(n) \) from the open list:  
     - \( g(n) \): cost to reach the node.  
     - \( h(n) \): estimated cost (heuristic) from the node to the goal.  
   - If it is the goal node, reconstruct the path and return it.  
   - Otherwise, add the node to the closed list and explore its neighbors.  
   - For each neighbor, calculate its cost and heuristic, and add it to the open list if it’s not already visited or if a cheaper path to it is found.  

## Usage in Robotics  

In robotics, the A* algorithm can be employed in various applications, including:  

- **Pathfinding**: A* is widely used for finding optimal paths for robots in complex environments, such as navigating through obstacles in indoor or outdoor settings.  

- **Navigation**: Robots can utilize A* for real-time path planning, allowing them to efficiently reach their target destinations while avoiding forbidden zones or hazards.  

- **Game Development**: The A* algorithm is commonly applied in AI for games to navigate characters and vehicles through the game's environment smoothly.  

## Usage Example  


https://github.com/user-attachments/assets/1d059669-14fb-4804-9aa8-7803937fc89a

![UniformCost](https://github.com/user-attachments/assets/82bd5d74-8b17-43a9-b986-1be065af9233)
