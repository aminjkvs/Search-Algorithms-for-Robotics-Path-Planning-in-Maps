clc;
clear;
close all;

% Generate a complex map using mapMaze
map1 = mapMaze(10, 5, 'MapSize', [50 50], 'MapResolution', 1); % Create maze map
occupancyMapSample = occupancyMatrix(map1); % Extract occupancy matrix from the map
[rows, cols] = size(occupancyMapSample); % Get the dimensions of the map

% Define start and goal points
startPoint = [10, 10]; % Start position (row, col)
goalPoint = [40, 40]; % Goal position (row, col)
maxIterations = 3000; % Maximum number of iterations
stepSize = 5; % Step size for extending the tree
goalThreshold = 5; % Threshold distance to stop near the goal

% Check if start and goal are valid
if occupancyMapSample(startPoint(1), startPoint(2)) == 1 || occupancyMapSample(goalPoint(1), goalPoint(2)) == 1
    error('Start or Goal point is inside an obstacle!');
end

% Initialize the RRT tree
tree = [startPoint, 0]; % [x, y, parentIndex]
foundPath = false;

% Create a figure for visualization
figure;
imshow(flipud(occupancyMapSample), 'InitialMagnification', 'fit'); % Flip map vertically for proper visualization
title('RRT Path Planning');
xlabel('X (columns)');
ylabel('Y (rows)');
xticks(1:5:cols); % Add numbers to X axis
yticks(1:5:rows); % Add numbers to Y axis
grid on;
hold on;

% Visualize start and goal points
plot(startPoint(2), rows - startPoint(1) + 1, 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start point in green
plot(goalPoint(2), rows - goalPoint(1) + 1, 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal point in red

% Create a VideoWriter object
videoFileName = 'C:\Users\Amin\Desktop\RRT_Path_Planning.avi'; % Specify the output file name
videoWriter = VideoWriter(videoFileName); % Create a VideoWriter object
open(videoWriter); % Open the file for writing

% RRT Algorithm
for i = 1:maxIterations
    % Generate a random point in the map
    randomPoint = [randi([1, rows]), randi([1, cols])];
    
    % Find the nearest node in the tree to the random point
    distances = sqrt(sum((tree(:, 1:2) - randomPoint).^2, 2));
    [~, nearestIndex] = min(distances);
    nearestNode = tree(nearestIndex, 1:2);
    
    % Generate a new point in the direction of the random point
    direction = (randomPoint - nearestNode) / norm(randomPoint - nearestNode);
    newPoint = nearestNode + stepSize * direction;
    newPoint = round(newPoint); % Round to integer (grid cells)
    
    % Check if the new point is valid (collision-free and within bounds)
    if newPoint(1) >= 1 && newPoint(1) <= rows && ...
       newPoint(2) >= 1 && newPoint(2) <= cols && ...
       occupancyMapSample(newPoint(1), newPoint(2)) == 0
        % Add the new point to the tree
        tree = [tree; newPoint, nearestIndex];
        
        % Visualize the connection
        plot([nearestNode(2), newPoint(2)], [rows - nearestNode(1) + 1, rows - newPoint(1) + 1], 'b-', 'LineWidth', 1);
        scatter(newPoint(2), rows - newPoint(1) + 1, 10, 'b', 'filled'); % New node in blue
        
        % Capture the current frame
        frame = getframe(gcf);
        writeVideo(videoWriter, frame); % Write frame to the video
        
        % Check if the new point is near the goal
        if norm(newPoint - goalPoint) < goalThreshold
            foundPath = true;
            tree = [tree; goalPoint, size(tree, 1)]; % Add goal to the tree
            break;
        end
    end
end

% Path Reconstruction
if foundPath
    path = [goalPoint];
    currentIndex = size(tree, 1); % Start from the goal
    while currentIndex ~= 0
        path = [tree(currentIndex, 1:2); path];
        currentIndex = tree(currentIndex, 3); % Move to the parent node
    end
    
    % Visualize the path
    plot(path(:, 2), rows - path(:, 1) + 1, 'r-', 'LineWidth', 2); % Path in red
    scatter(path(:, 2), rows - path(:, 1) + 1, 50, 'r', 'filled'); % Path nodes in red
    
    % Capture the final path frame
    frame = getframe(gcf);
    writeVideo(videoWriter, frame); % Write frame to the video
    
    disp('Path found:');
    disp(path);
else
    disp('No path found!');
end

% Close the video file
close(videoWriter);

hold off;

disp(['Animation saved as ', videoFileName]);