clc;
clear;
close all;

% Initialize the map and extract the occupancy matrix
map1 = mapMaze(10, 5, 'MapSize', [50 50], 'MapResolution', 1);
occupancyMapSample = occupancyMatrix(map1); % Get the occupancy matrix

% Start and end points
startPoint = [10, 10];
endPoint = [10, 25];

% Get the size of the maze
[rows, cols] = size(occupancyMapSample);

% Initialize visited matrices and parent dictionaries for both searches
visitedForward = false(rows, cols); % Keeps track of visited cells in forward search
visitedBackward = false(rows, cols); % Keeps track of visited cells in backward search
parentsForward = cell(rows, cols); % Stores parent information for backtracking in forward search
parentsBackward = cell(rows, cols); % Stores parent information for backtracking in backward search

% Open sets for both directions
openSetForward = [startPoint, 0]; % [x, y, g_score]
openSetBackward = [endPoint, 0]; % [x, y, g_score]

% gScores for both directions
gScoreForward = containers.Map();
gScoreBackward = containers.Map();
gScoreForward(sprintf('%d,%d', startPoint(1), startPoint(2))) = 0;
gScoreBackward(sprintf('%d,%d', endPoint(1), endPoint(2))) = 0;

% Create a figure for visualization
figure;
show(map1); % Show the maze
hold on;
title('Bidirectional A* Maze Traversal');
xlabel('X');
ylabel('Y');

% Visualization parameters
scatter(startPoint(2), rows - startPoint(1) + 1, 'g', 'filled'); % Start point
scatter(endPoint(2), rows - endPoint(1) + 1, 'r', 'filled'); % End point
pause(0.1);

% Create a Video Writer object
videoWriter = VideoWriter('C:\Users\Amin\Desktop\Bidirectional_AStar_Maze_Traversal.avi'); % Full path for the output video
open(videoWriter); % Open the video writer for writing

% Bidirectional A* Search loop
found = false; % Flag to indicate if the end point was found
meetingPoint = [];

while ~isempty(openSetForward) && ~isempty(openSetBackward)
    % Forward search: calculate f_scores and pick the node with the lowest f_score
    fScoresForward = zeros(size(openSetForward, 1), 1);
    for i = 1:size(openSetForward, 1)
        node = openSetForward(i, 1:2);
        fScoresForward(i) = gScoreForward(sprintf('%d,%d', node(1), node(2))) + heuristic(node, endPoint);
    end
    [~, idxForward] = min(fScoresForward);
    currentForward = openSetForward(idxForward, 1:2);
    openSetForward(idxForward, :) = []; % Remove the node from the open set

    % Backward search: calculate f_scores and pick the node with the lowest f_score
    fScoresBackward = zeros(size(openSetBackward, 1), 1);
    for i = 1:size(openSetBackward, 1)
        node = openSetBackward(i, 1:2);
        fScoresBackward(i) = gScoreBackward(sprintf('%d,%d', node(1), node(2))) + heuristic(node, startPoint);
    end
    [~, idxBackward] = min(fScoresBackward);
    currentBackward = openSetBackward(idxBackward, 1:2);
    openSetBackward(idxBackward, :) = []; % Remove the node from the open set

    % Mark nodes as visited
    visitedForward(currentForward(1), currentForward(2)) = true;
    visitedBackward(currentBackward(1), currentBackward(2)) = true;

    % Get neighbors for both forward and backward searches
    neighborsForward = getNeighbors(currentForward, rows, cols, occupancyMapSample);
    neighborsBackward = getNeighbors(currentBackward, rows, cols, occupancyMapSample);

    % Check if any neighbor of forward overlaps with any neighbor of backward
    for i = 1:size(neighborsForward, 1)
        for j = 1:size(neighborsBackward, 1)
            if isequal(neighborsForward(i, :), neighborsBackward(j, :))
                found = true;
                meetingPoint = neighborsForward(i, :); % Set the meeting point
                break; % Exit the inner neighbor loop
            end
        end
        if found
            break; % Exit the outer neighbor loop
        end
    end

    % If searches meet, break out of the main loop
    if found
        break; % Exit the main loop immediately
    end

    % Visualize the exploration process (forward search)
    scatter(currentForward(2), rows - currentForward(1) + 1, 'b', 'filled'); % Current cell (forward)
    pause(0.01);
    frame = getframe(gcf); % Capture the figure as a frame
    writeVideo(videoWriter, frame); % Write the frame to the video

    % Visualize the exploration process (backward search)
    scatter(currentBackward(2), rows - currentBackward(1) + 1, 'c', 'filled'); % Current cell (backward)
    pause(0.01);
    frame = getframe(gcf); % Capture the figure as a frame
    writeVideo(videoWriter, frame); % Write the frame to the video

    % Forward search: explore neighbors
    for i = 1:size(neighborsForward, 1)
        neighbor = neighborsForward(i, :);
        tentativeGScore = gScoreForward(sprintf('%d,%d', currentForward(1), currentForward(2))) + 1; % Assuming uniform cost

        if ~isKey(gScoreForward, sprintf('%d,%d', neighbor(1), neighbor(2))) || tentativeGScore < gScoreForward(sprintf('%d,%d', neighbor(1), neighbor(2)))
            parentsForward{neighbor(1), neighbor(2)} = currentForward; % Store parent
            gScoreForward(sprintf('%d,%d', neighbor(1), neighbor(2))) = tentativeGScore; % Update g_score

            if all(~ismember(openSetForward(:, 1:2), neighbor, 'rows'))
                openSetForward = [openSetForward; neighbor, tentativeGScore];
                scatter(neighbor(2), rows - neighbor(1) + 1, 'y', 'filled'); % Neighbor cell
                pause(0.01);
                frame = getframe(gcf); % Capture the figure as a frame
                writeVideo(videoWriter, frame); % Write the frame to the video
            end
        end
    end

    % Backward search: explore neighbors
    for j = 1:size(neighborsBackward, 1)
        neighbor = neighborsBackward(j, :);
        tentativeGScore = gScoreBackward(sprintf('%d,%d', currentBackward(1), currentBackward(2))) + 1; % Assuming uniform cost

        if ~isKey(gScoreBackward, sprintf('%d,%d', neighbor(1), neighbor(2))) || tentativeGScore < gScoreBackward(sprintf('%d,%d', neighbor(1), neighbor(2)))
            parentsBackward{neighbor(1), neighbor(2)} = currentBackward; % Store parent
            gScoreBackward(sprintf('%d,%d', neighbor(1), neighbor(2))) = tentativeGScore; % Update g_score

            if all(~ismember(openSetBackward(:, 1:2), neighbor, 'rows'))
                openSetBackward = [openSetBackward; neighbor, tentativeGScore];
                scatter(neighbor(2), rows - neighbor(1) + 1, 'm', 'filled'); % Neighbor cell
                pause(0.01);
                frame = getframe(gcf); % Capture the figure as a frame
                writeVideo(videoWriter, frame); % Write the frame to the video
            end
        end
    end
end

% Backtrack to find the path if the searches met
if found
    % Forward path from start to meeting point
    pathForward = [];
    current = meetingPoint;
    while ~isempty(current)
        pathForward = [current; pathForward]; % Add current cell to the path
        current = parentsForward{current(1), current(2)}; % Move to the parent cell
    end

    % Backward path from end to meeting point
    pathBackward = [];
    current = meetingPoint;
    while ~isempty(current)
        pathBackward = [current; pathBackward]; % Add current cell to the path
        current = parentsBackward{current(1), current(2)}; % Move to the parent cell
    end

    % Merge paths (remove duplicate meeting point)
    pathBackward = flip(pathBackward(2:end, :)); % Remove duplicate meeting point
    path = [pathForward; pathBackward];

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

% Heuristic function: Manhattan distance
function h = heuristic(point, goal)
    h = abs(point(1) - goal(1)) + abs(point(2) - goal(2)); % Manhattan distance
end