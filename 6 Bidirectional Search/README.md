# Bidirectional A* Search Algorithm  

## Overview  

Bidirectional A* is an extension of the A* Search Algorithm that aims to improve the efficiency of the search process by simultaneously exploring paths from both the start node and the goal node. By searching from both directions and meeting in the middle, it can significantly reduce the number of nodes explored and find the optimal path more quickly than traditional unidirectional A*.  

## Algorithm Steps  

1. **Initialize** two open lists: one for the forward search from the start node and one for the backward search from the goal node.  
2. **Initialize** two closed lists to keep track of visited nodes for both directions.  
3. **While both open lists are not empty**:  
   - Expand the node with the lowest estimated total cost from one direction (forward or backward).  
   - If it meets the other search (i.e., both searches reach the same node), reconstruct the path from start to goal.  
   - For each neighbor of the current node, calculate the total cost and enqueue it if it has not been visited or if a cheaper path is found.  
4. **Terminate** when the two searches meet, reconstructing the complete path from start to goal.  

## Usage in Robotics  

In robotics, the Bidirectional A* algorithm can be employed in various applications, including:  

- **Pathfinding**: Bidirectional A* can provide faster pathfinding in dynamic environments where obstacles can change frequently.  

- **Navigation and Exploration**: Robots can use Bidirectional A* to efficiently navigate through environments by integrating data from both the start and goal states, reducing computation time.  

- **Multi-Agent Systems**: This approach is useful for coordination in multi-agent scenarios, where multiple robots may require optimal paths simultaneously.  

## Usage Example  


https://github.com/user-attachments/assets/aebda639-01d6-47ab-9936-13ce6241b60c

![BiDirectionalAstarSearch](https://github.com/user-attachments/assets/6e024368-c8e5-4519-804d-9fe6597cf591)
