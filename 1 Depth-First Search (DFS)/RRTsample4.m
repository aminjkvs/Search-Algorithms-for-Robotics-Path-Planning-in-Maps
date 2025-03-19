clc;
close all;
clear all;

% Create a 100x100 occupancy map  
occupancyMapsample = zeros(100, 100); % Empty map  

% Adding a vertical obstacle in the center  
occupancyMapsample(5:40, 15:20) = 1;
occupancyMapsample(60:95, 15:20) = 1;
occupancyMapsample(5:80, 30:35) = 1;
occupancyMapsample(20:25, 50:95) = 1;
occupancyMapsample(40:95, 60:65) = 1;

% Vertical obstacle from (40, 50) to (60, 50)  
occupancyMapsample(1:100, 95:100) = 1;
occupancyMapsample(1:100, 1:5) = 1;
occupancyMapsample(1:5, 1:100) = 1;
occupancyMapsample(95:100, 1:100) = 1;
map = occupancyMap(occupancyMapsample,1);
start = [10, 10, 0];
goal = [10, 29, 0];
[pathMetricsObj,pathMetricsObjModified]=RRTPlannerStarFunction(map,start,goal);

% load exampleMaps.mat; % simpleMap
% mapResolution = 1; % cells/meter
% map = occupancyMap(simpleMap,mapResolution);
% start = [5, 5, 0]; % [meters, meters, radians]
% goal = [20, 20, 0];
% 
% [pathMetricsObj,pathMetricsObjModified]=RRTPlannerStarFunction(map,start,goal);






function [pathMetricsObj,pathMetricsObjModified]=RRTPlannerStarFunction(map,start,goal)
    
    statespace = stateSpaceDubins;
    statevalidator = validatorOccupancyMap(statespace);
    statevalidator.Map = map;
    statevalidator.ValidationDistance = 0.01;
    statespace.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];

	planner = plannerRRTStar(statespace,statevalidator);
    planner.ContinueAfterGoalReached = true;
    planner.MaxIterations = 2500;
    planner.MaxConnectionDistance = 1;
    rng(100,'twister') % repeatable result
    [path,solutionInfo] = plan(planner,start,goal);
    pathMetricsObj = pathmetrics(path,statevalidator);
    
    % Assuming you have the original path from the planner in a navPath object  
    originalPath = path; % This is a navPath object   
    pathData = originalPath.States; % Assumes States contains the [x, y, heading] data  
    length1=node2NodeLength(pathData)
    distanceMatrix=MAtrixCreator(map,pathData)
    [shortestPath, minPathLength] = dijkstraShortestPath(distanceMatrix)
    desiredIndices=shortestPath;

    newPathData = pathData(desiredIndices, :); % Select only the desired nodes
    length2=node2NodeLength(newPathData)
    modifiedPath = navPath(statespace, newPathData); 
    % Verify if the modified path is valid using the state validator  
    pathMetricsObjModified = pathmetrics(modifiedPath, statevalidator); 
    
    isPathValid(pathMetricsObj)
    clearance(pathMetricsObj)
    smoothness(pathMetricsObj)
    show(pathMetricsObj)
    legend('Planned Path','Minimum Clearance')

    isPathValidModified = isPathValid(pathMetricsObjModified); % Check if the modified path is valid  
    clearanceModified = clearance(pathMetricsObjModified);     % Get clearance information  
    smoothnessModified = smoothness(pathMetricsObjModified);     % Check smoothness  

    disp('Modified path validity:');
    disp(isPathValidModified);
    disp('Clearance of modified path:');
    disp(clearanceModified);
    disp('Smoothness of modified path:');
    disp(smoothnessModified);

    %show(pathMetricsObjModified);
    %legend('Modified Path','Minimum Clearance');

end

function totalLength=node2NodeLength(pathData)
% Initialize total length  
totalLength = 0;  

% Loop through each pair of consecutive nodes  
for i = 1:size(pathData, 1) - 1  
    % Calculate the distance between consecutive nodes  
    distance = norm(pathData(i, 1:2) - pathData(i + 1, 1:2)); % Calculates 2D Euclidean distance  
    totalLength = totalLength + distance; % Sum the distances  
end   
end

function distanceMatrix=MAtrixCreator(map,pathNodes)
% Initialize parameters  
pathNodes=pathNodes(:,1:2);
numNodes = size(pathNodes, 1);  
largeNumber = inf; % Large number for inaccessible paths  
distanceMatrix = largeNumber * ones(numNodes, numNodes); % Initialize distance matrix  

% Create the occupancy map (assuming you have a function to do that)   
stateValidator = validatorOccupancyMap(stateSpaceDubins);  
stateValidator.Map = map;  

% Calculate distances  
for i = 1:numNodes  
    for j = 1:numNodes  
        if i ~= j  
            % Check if the path between node i and node j is valid  
            start = [pathNodes(i, :), 0]; % Append heading (0 radians)  
            goal = [pathNodes(j, :), 0];  % Append heading (0 radians)  
            
            % Use isPathValid function to check if the path is valid  
            pathCheck = pathmetrics(navPath(stateSpaceDubins, [start; goal]), stateValidator);  
            if isPathValid(pathCheck)  
                % Calculate the Euclidean distance  
                distanceMatrix(i, j) = norm(pathNodes(i, :) - pathNodes(j, :));  
            end  
        end  
    end  
end  

% Display the distance matrix  
% disp('Distance Matrix:');  
% disp(distanceMatrix);
end

function [shortestPath, minPathLength] = dijkstraShortestPath(distanceMatrix)  

    numNodes = size(distanceMatrix, 1);  
    startNode = 1; % Start from first node  
    endNode = numNodes; % End at last node  

    % Initialize distances and visited nodes  
    distances = inf(1, numNodes);  
    distances(startNode) = 0; % Distance to the start node is zero  
    visited = false(1, numNodes);  
    previous = nan(1, numNodes); % To reconstruct the path  

    % Create a priority queue for node indices based on distances  
    queue = [startNode];  

    while ~isempty(queue)  
        % Get the node with the smallest distance  
        [~, idx] = min(distances(queue));   
        currentNode = queue(idx);  

        % Remove currentNode from the queue  
        queue(idx) = [];  

        if visited(currentNode)  
            continue; % Skip if already visited  
        end  
        if currentNode == endNode  
            break; % Stop if we reached the end node  
        end  

        visited(currentNode) = true; % Mark the node as visited  

        % Update distances for neighboring nodes  
        for neighbor = 1:numNodes  
            if ~visited(neighbor) && distanceMatrix(currentNode, neighbor) < inf  
                newDistance = distances(currentNode) + distanceMatrix(currentNode, neighbor);  
                if newDistance < distances(neighbor)  
                    distances(neighbor) = newDistance; % Update the distance  
                    previous(neighbor) = currentNode; % Track the previous node  

                    % Add neighbor to the queue if not already present  
                    if ~ismember(neighbor, queue)  
                        queue(end + 1) = neighbor;  
                    end  
                end  
            end  
        end  
    end  

    % Construct the path  
    if distances(endNode) < inf  
        minPathLength = distances(endNode);  
        shortestPath = [];  
        currentNode = endNode;  

        while ~isnan(currentNode)  
            shortestPath = [currentNode, shortestPath]; % Prepend current node  
            currentNode = previous(currentNode); % Move to the previous node  
        end  
    else  
        minPathLength = inf; % No path found  
        shortestPath = [];  
    end  
end