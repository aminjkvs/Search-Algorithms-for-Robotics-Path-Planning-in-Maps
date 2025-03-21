clc;
clear;
close all;

% Initialize the map and extract the occupancy matrix
map1 = mapMaze(10, 5, 'MapSize', [50 50], 'MapResolution', 1);
occupancyMapSample = occupancyMatrix(map1); % Get the occupancy matrix

% Start and end points
startPoint = [10, 10];
endPoint = [40, 40];

% Get the size of the maze
[rows, cols] = size(occupancyMapSample);

% Initialize visited matrix and parent dictionary
visited = false(rows, cols); % Keeps track of visited cells
parents = cell(rows, cols); % Stores parent information for backtracking

% Priority queue for Uniform Cost Search (UCS)
priorityQueue = [startPoint, 0]; % [Point, Cost]

% Mark the start point as visited
visited(startPoint(1), startPoint(2)) = true;

% Create a figure for visualization
figure;
show(map1); % Show the maze
hold on;
title('Uniform Cost Search Maze Traversal');
xlabel('X');
ylabel('Y');

% Visualization parameters
scatter(startPoint(2), rows - startPoint(1) + 1, 'g', 'filled'); % Start point
scatter(endPoint(2), rows - endPoint(1) + 1, 'r', 'filled'); % End point
pause(0.1);

% Create a Video Writer object
videoWriter = VideoWriter('C:\Users\Amin\Desktop\UniformCostSearch_Maze_Traversal.avi'); % Full path for the output video
open(videoWriter); % Open the video writer for writing

% Uniform Cost Search loop
found = false; % Flag to indicate if the end point was found
while ~isempty(priorityQueue)
    % Sort the priority queue by cost (ascending order)
    [~, idx] = sort(priorityQueue(:, 3));
    priorityQueue = priorityQueue(idx, :);
    
    % Dequeue the cell with the lowest cost
    current = priorityQueue(1, 1:2); % Extract the cell
    currentCost = priorityQueue(1, 3); % Extract the cost
    priorityQueue(1, :) = []; % Remove the cell from the queue
    
    % Check if we reached the end point
    if isequal(current, endPoint)
        found = true;
        break;
    end
    
    % Visualize the exploration process
    scatter(current(2), rows - current(1) + 1, 'b', 'filled'); % Current cell
    pause(0.01);
    frame = getframe(gcf); % Capture the figure as a frame
    writeVideo(videoWriter, frame); % Write the frame to the video
    
    % Get neighbors
    neighbors = getNeighbors(current, rows, cols, occupancyMapSample);
    
    for i = 1:size(neighbors, 1)
        neighbor = neighbors(i, :);
        % Process unvisited neighbors
        if ~visited(neighbor(1), neighbor(2))
            visited(neighbor(1), neighbor(2)) = true; % Mark as visited
            parents{neighbor(1), neighbor(2)} = current; % Store parent
            
            % Compute the cost to reach the neighbor
            neighborCost = currentCost + 1; % Increment cost by 1 (assuming uniform cost)
            
            % Add neighbor to the priority queue
            priorityQueue = [priorityQueue; neighbor, neighborCost];
            
            % Visualize neighbors being added to the queue
            scatter(neighbor(2), rows - neighbor(1) + 1, 'y', 'filled'); % Neighbor cell
            pause(0.01);
            frame = getframe(gcf); % Capture the figure as a frame
            writeVideo(videoWriter, frame); % Write the frame to the video
        end
    end
end

% Backtrack to find the path if the end point was found
if found
    path = [];
    current = endPoint;
    while ~isempty(current)
        path = [current; path]; % Add current cell to the path
        current = parents{current(1), current(2)}; % Move to the parent cell
    end
    
    % Visualize the final path
    for i = 1:size(path, 1)
        scatter(path(i, 2), rows - path(i, 1) + 1, 'r', 'filled'); % Path cell
        pause(0.01);
        frame = getframe(gcf); % Capture the figure as a frame
        writeVideo(videoWriter, frame); % Write the frame to the video
    end
    
    disp('Optimal Path from start to end:');
    disp(path);
else
    disp('End point not reachable!');
end

hold off;

% Close the video writer
close(videoWriter);

% Function to get valid neighbors
function neighbors = getNeighbors(current, rows, cols, map)
    directions = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1]; % Right, Down, Left, Up, Diagonals
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = current + directions(i, :);
        % Check bounds and if the cell is free
        if neighbor(1) > 0 && neighbor(1) <= rows && ...
           neighbor(2) > 0 && neighbor(2) <= cols && ...
           map(neighbor(1), neighbor(2)) == 0 % Free cell
            neighbors = [neighbors; neighbor];
        end
    end
end