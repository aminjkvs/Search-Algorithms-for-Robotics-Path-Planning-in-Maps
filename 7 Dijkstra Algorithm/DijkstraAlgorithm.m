% Step 1: Create the maze and walls
map1 = mapMaze(10, 5, 'MapSize', [50 50], 'MapResolution', 1);  
occupancyMapSample = occupancyMatrix(map1); % Get the occupancy matrix

% Step 2: Generate the heat map
[rows, cols] = size(occupancyMapSample);
heatMap = zeros(rows, cols); % Initialize heat map

% Assign values based on distance from walls
for r = 1:rows
    for c = 1:cols
        if occupancyMapSample(r, c) == 1 % Wall
            heatMap(r, c) = 2000; % Walls have the highest value
        else
            % Calculate distance from nearest wall
            distance = 0;
            maxDistance = 5; % Maximum distance to check
            for d = 1:maxDistance
                nearbyWall = any(any(occupancyMapSample(max(1, r-d):min(rows, r+d), max(1, c-d):min(cols, c+d)) == 1));
                if nearbyWall
                    distance = d;
                    break;
                end
            end
            
            % Assign heat map values based on distance
            switch distance
                case 1
                    heatMap(r, c) = 100;
                case 2
                    heatMap(r, c) = 80;
                case 3
                    heatMap(r, c) = 60;
                case 4
                    heatMap(r, c) = 40;
                otherwise
                    heatMap(r, c) = 0; % Far from walls
            end
        end
    end
end

% Step 3: Find the shortest path using Dijkstra and animate the search
startNode = [10, 10]; % Start point
endNode = [40, 40]; % End point
[shortestPath, pathCost] = dijkstra_2d(heatMap, startNode, endNode);  

% Create a video writer object
videoFileName = 'C:\Users\Amin\Desktop\PathPlanningAnimation.avi';
v = VideoWriter(videoFileName);
v.FrameRate = 10; % Set frame rate
open(v);

% Animate the search and progress
figure;
imagesc(heatMap); % Show the maze
colorbar;
hold on;
title('Shortest Path Animation');

% Initialize the plot
frame = getframe(gcf); % Capture the initial frame
writeVideo(v, frame);

for i = 1:length(shortestPath)-1
    % Plot the path step by step
    plot(shortestPath(i, 2), shortestPath(i, 1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    pause(0.1); % Pause for animation
    frame = getframe(gcf); % Capture the frame
    writeVideo(v, frame); % Write the frame to the video
end

% Plot the complete path
plot(shortestPath(:, 2), shortestPath(:, 1), 'g-', 'LineWidth', 2);
frame = getframe(gcf); % Capture the final frame
writeVideo(v, frame); % Write the final frame to the video
hold off;

% Close the video writer
close(v);

disp(['Animation saved as ', videoFileName]);

function [path, cost] = dijkstra_2d(map, start, goal)  
    % dijkstra_2d - Finds the shortest path on a 2D grid using Dijkstra's algorithm  
    %  
    % Syntax: [path, cost] = dijkstra_2d(map, start, goal)  
    %  
    % Inputs:  
    %    map  - 2D matrix representing the grid, where each cell is a cost  
    %    start - 1x2 vector [x, y] indicating the starting point in the grid  
    %    goal  - 1x2 vector [x, y] indicating the goal point in the grid  
    %  
    % Outputs:  
    %    path  - Nx2 matrix where each row is a [x, y] coordinate of the path  
    %    cost  - Total cost of the path  
    
    % Initialize parameters  
    [rows, cols] = size(map);  
    dist = inf(rows, cols); % Distance matrix  
    dist(start(1), start(2)) = 0; % Starting point distance  
    prev = NaN(rows, cols); % Previous cell matrix  
    visited = false(rows, cols);  
    
    % Directions for moving in the grid (right, down, left, up)  
    directions = [0 1; 1 0; 0 -1; -1 0];  
    
    % Priority queue (using a simple matrix for the sake of simplicity)  
    queue = [start, dist(start(1), start(2))]; % [x, y, cost]  
    while ~isempty(queue)  
        % Sort and pop the node with the lowest distance  
        [~, idx] = min(queue(:, 3)); % Dijkstra's step  
        current = queue(idx, 1:2);  
        queue(idx, :) = []; % Remove it from the queue  
        
        % Mark as visited  
        visited(current(1), current(2)) = true;  

        % If we reach the goal, construct the path  
        if isequal(current, goal)  
            path = construct_path(prev, goal);  
            cost = dist(goal(1), goal(2));  
            return;  
        end  
        
        % Explore neighbors  
        for d = 1:size(directions, 1)  
            neighbor = current + directions(d, :);  
            if is_valid(neighbor, rows, cols) && ~visited(neighbor(1), neighbor(2))  
                alt = dist(current(1), current(2)) + map(neighbor(1), neighbor(2));  
                if alt < dist(neighbor(1), neighbor(2))  
                    dist(neighbor(1), neighbor(2)) = alt;  
                    prev(neighbor(1), neighbor(2)) = sub2ind(size(map), current(1), current(2));  
                    % Add or update neighbor in the queue  
                    if ~ismember(neighbor, queue(:, 1:2), 'rows')  
                        queue = [queue; neighbor, alt];  
                    end  
                end  
            end  
        end  
    end  
    
    % Return empty path and Inf cost if no path found  
    path = [];  
    cost = inf;  
end  

function valid = is_valid(pos, rows, cols)  
    valid = pos(1) > 0 && pos(1) <= rows && pos(2) > 0 && pos(2) <= cols;  
end  

function path = construct_path(prev, goal)  
    path = goal;  
    while ~isnan(prev(goal(1), goal(2)))  
        index = prev(goal(1), goal(2));  
        [x, y] = ind2sub(size(prev), index);  
        path = [x, y; path]; % Prepend the previous node  
        goal = [x, y]; % Move to the previous node  
    end  
end