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
stepSize = 2; % Step size for extending the tree
goalThreshold = 5; % Threshold distance to stop near the goal

% Check if start and goal points are valid
if occupancyMapSample(startPoint(1), startPoint(2)) == 1 || occupancyMapSample(goalPoint(1), goalPoint(2)) == 1
    error('Start or Goal point is inside an obstacle!');
end

% Initialize the trees
startTree = [startPoint, 0]; % [x, y, parentIndex]
goalTree = [goalPoint, 0]; % [x, y, parentIndex]
foundPath = false;

% Create a figure for visualization
figure;
imshow(flipud(occupancyMapSample), 'InitialMagnification', 'fit'); % Flip map vertically for proper visualization
title('Bidirectional RRT Path Planning');
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
videoFileName = 'C:\Users\Amin\Desktop\Bidirectional_RRT_Path_Planning.avi'; % Specify the output file name
videoWriter = VideoWriter(videoFileName); % Create a VideoWriter object
open(videoWriter); % Open the file for writing

% Bidirectional RRT Algorithm
for i = 1:maxIterations
    % Randomly expand the start tree
    randomPointStart = [randi([1, rows]), randi([1, cols])];
    distancesStart = sqrt(sum((startTree(:, 1:2) - randomPointStart).^2, 2));
    [~, nearestIndexStart] = min(distancesStart);
    nearestNodeStart = startTree(nearestIndexStart, 1:2);
    directionStart = (randomPointStart - nearestNodeStart) / norm(randomPointStart - nearestNodeStart);
    newPointStart = nearestNodeStart + stepSize * directionStart;
    newPointStart = round(newPointStart); % Round to integer (grid cells)
    
    % Check if the new point is valid (collision-free and within bounds)
    if newPointStart(1) >= 1 && newPointStart(1) <= rows && ...
       newPointStart(2) >= 1 && newPointStart(2) <= cols && ...
       occupancyMapSample(newPointStart(1), newPointStart(2)) == 0
        % Add the new point to the start tree
        startTree = [startTree; newPointStart, nearestIndexStart];
        
        % Visualize the connection
        plot([nearestNodeStart(2), newPointStart(2)], [rows - nearestNodeStart(1) + 1, rows - newPointStart(1) + 1], 'b-', 'LineWidth', 1);
        scatter(newPointStart(2), rows - newPointStart(1) + 1, 10, 'b', 'filled'); % New node in blue
        frame = getframe(gcf);
        writeVideo(videoWriter, frame); % Write frame to the video
    end
    
    % Randomly expand the goal tree
    randomPointGoal = [randi([1, rows]), randi([1, cols])];
    distancesGoal = sqrt(sum((goalTree(:, 1:2) - randomPointGoal).^2, 2));
    [~, nearestIndexGoal] = min(distancesGoal);
    nearestNodeGoal = goalTree(nearestIndexGoal, 1:2);
    directionGoal = (randomPointGoal - nearestNodeGoal) / norm(randomPointGoal - nearestNodeGoal);
    newPointGoal = nearestNodeGoal + stepSize * directionGoal;
    newPointGoal = round(newPointGoal); % Round to integer (grid cells)
    
    % Check if the new point is valid (collision-free and within bounds)
    if newPointGoal(1) >= 1 && newPointGoal(1) <= rows && ...
       newPointGoal(2) >= 1 && newPointGoal(2) <= cols && ...
       occupancyMapSample(newPointGoal(1), newPointGoal(2)) == 0
        % Add the new point to the goal tree
        goalTree = [goalTree; newPointGoal, nearestIndexGoal];
        
        % Visualize the connection
        plot([nearestNodeGoal(2), newPointGoal(2)], [rows - nearestNodeGoal(1) + 1, rows - newPointGoal(1) + 1], 'g-', 'LineWidth', 1);
        scatter(newPointGoal(2), rows - newPointGoal(1) + 1, 10, 'g', 'filled'); % New node in green
        frame = getframe(gcf);
        writeVideo(videoWriter, frame); % Write frame to the video
    end
    
    % Check for connection between the two trees
    distancesToStartTree = sqrt(sum((startTree(:, 1:2) - newPointGoal).^2, 2));
    [minDistance, connectingIndexStart] = min(distancesToStartTree);
    if minDistance < stepSize
        foundPath = true;
        % Connect the two trees
        startTree = [startTree; newPointGoal, connectingIndexStart];
        break;
    end
    
    distancesToGoalTree = sqrt(sum((goalTree(:, 1:2) - newPointStart).^2, 2));
    [minDistance, connectingIndexGoal] = min(distancesToGoalTree);
    if minDistance < stepSize
        foundPath = true;
        % Connect the two trees
        goalTree = [goalTree; newPointStart, connectingIndexGoal];
        break;
    end
end

% Path Reconstruction
if foundPath
    % Reconstruct the path from start tree
    pathStart = [startTree(end, 1:2)];
    currentIndex = size(startTree, 1); % Start from the last node in start tree
    while currentIndex ~= 0
        pathStart = [startTree(currentIndex, 1:2); pathStart];
        currentIndex = startTree(currentIndex, 3); % Move to the parent node
    end
    
    % Reconstruct the path from goal tree
    pathGoal = [goalTree(end, 1:2)];
    currentIndex = size(goalTree, 1); % Start from the last node in goal tree
    while currentIndex ~= 0
        pathGoal = [goalTree(currentIndex, 1:2); pathGoal];
        currentIndex = goalTree(currentIndex, 3); % Move to the parent node
    end
    
    % Remove the duplicate meeting point from pathGoal
    pathGoal(1, :) = []; % Remove the first point (meeting point) in goal tree path
    
    % Combine the two paths
    path = [pathStart; pathGoal];
    
    % Visualize the path
    plot(path(:, 2), rows - path(:, 1) + 1, 'r-', 'LineWidth', 2); % Path in red
    scatter(path(:, 2), rows - path(:, 1) + 1, 50, 'r', 'filled'); % Path nodes in red
    disp('Path found:');
    disp(path);
else
    disp('No path found!');
end

% Close the video file
close(videoWriter);

hold off;

disp(['Animation saved as ', videoFileName]);