function [path, explored] = dijkstra(p)
    % Dijkstra algorithm implementation for finding optimal path in an N-dimensional grid
    % Inputs:
    %   grid - N-dimensional matrix where 1 represents obstacles and 0 represents free space
    %   start - 1xN vector with coordinates of start position
    %   goal - 1xN vector with coordinates of goal position
    %   connectivity - optional, specify connectivity type:
    %                  'minimal' = 2*N connections (only orthogonal moves)
    %                  'full' = 3^N-1 connections (all possible moves including diagonals)
    %                  Default is 'minimal'
    % Outputs:
    %   path - MxN matrix containing the coordinates of the optimal path
    %   explored - KxN matrix containing all explored nodes
    
    % Check inputs
    grid = p.ToGrid;
    start = p.Start;
    goal = p.End;
    connectivity = p.Conn;
    if nargin < 4
        connectivity = 'minimal';
    end
    
    % Get dimensionality and grid size
    dims = ndims(grid);
    gridSize = size(grid);
    
    % Verify start and goal dimensions match the grid
    if length(start) ~= dims || length(goal) ~= dims
        error('Start and goal dimensions must match grid dimensions');
    end
    
    % Generate moves based on dimensionality and connectivity type
    moves = generateMoves(dims, connectivity);
    
    % Initialize data structures
    % openSet stores nodes to be evaluated: [coords, distance, parent_idx]
    % distance = accumulated cost from start, parent_idx = index in closedSet
    maxNodes = prod(gridSize); % Maximum possible nodes
    openSet = zeros(maxNodes, dims+2);
    openSet(1, 1:dims) = start;
    openSet(1, dims+1:dims+2) = [0, 0]; % distance=0, parent=0
    openCount = 1;
    
    % closedSet stores evaluated nodes: [coords, distance, parent_idx]
    closedSet = zeros(maxNodes, dims+2);
    closedCount = 0;
    
    % Main loop
    goalReached = false;
    
    while openCount > 0 && ~goalReached
        % Find node with lowest distance value in openSet
        [~, currentIdx] = min(openSet(1:openCount, dims+1));
        current = openSet(currentIdx, 1:dims);
        currentDist = openSet(currentIdx, dims+1);
        currentParentIdx = openSet(currentIdx, dims+2);
        
        % Remove current from openSet by swapping with last element and reducing count
        openSet(currentIdx, :) = openSet(openCount, :);
        openCount = openCount - 1;
        
        % Add current to closedSet
        closedCount = closedCount + 1;
        closedSet(closedCount, :) = [current, currentDist, currentParentIdx];
        
        % Check if we've reached the goal
        if isequal(current, goal)
            goalReached = true;
            break;
        end
        
        % Explore neighbors
        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);
            
            % Check if neighbor is valid (within grid bounds)
            valid = true;
            for d = 1:dims
                if neighbor(d) < 1 || neighbor(d) > gridSize(d)
                    valid = false;
                    break;
                end
            end
            if ~valid
                continue;
            end
            
            % Get linear index for checking grid value
            linearIdx = coordToLinearIndex(neighbor, gridSize);
            
            % Check if neighbor is obstacle
            if grid(linearIdx) == 1
                continue;
            end
            
            % Calculate tentative distance (cost from start)
            % Use Euclidean distance for move cost when using 'full' connectivity
            if strcmp(connectivity, 'full')
                moveCost = sqrt(sum(moves(i, :).^2));
            else
                moveCost = 1; % Uniform cost for 'minimal' connectivity
            end
            tentativeDist = currentDist + moveCost;
            
            % Check if neighbor is in closedSet with better distance
            inClosed = false;
            for j = 1:closedCount
                if isequal(closedSet(j, 1:dims), neighbor)
                    inClosed = true;
                    if closedSet(j, dims+1) <= tentativeDist
                        break;  % Skip this neighbor as we already have better path
                    else
                        % Remove from closedSet to reevaluate
                        closedSet(j, :) = closedSet(closedCount, :);
                        closedCount = closedCount - 1;
                        inClosed = false;
                        break;
                    end
                end
            end
            if inClosed
                continue;
            end
            
            % Check if neighbor is in openSet
            inOpen = false;
            for j = 1:openCount
                if isequal(openSet(j, 1:dims), neighbor)
                    inOpen = true;
                    if openSet(j, dims+1) <= tentativeDist
                        break;  % Skip this neighbor as we already have better path
                    else
                        % Update distance and parent
                        openSet(j, dims+1) = tentativeDist;
                        openSet(j, dims+2) = closedCount;
                        break;
                    end
                end
            end
            
            % If not in openSet, add it
            if ~inOpen
                openCount = openCount + 1;
                openSet(openCount, 1:dims) = neighbor;
                openSet(openCount, dims+1) = tentativeDist;
                openSet(openCount, dims+2) = closedCount;
            end
        end
    end
    
    % Reconstruct path if goal was reached
    path = [];
    explored = closedSet(1:closedCount, 1:dims);
    
    if goalReached
        % Backtrack from goal to start using parent indices
        path = zeros(closedCount, dims);
        pathLength = 0;
        current = closedCount;
        
        while current ~= 0
            pathLength = pathLength + 1;
            path(pathLength, :) = closedSet(current, 1:dims);
            current = closedSet(current, dims+2);
        end
        
        % Reverse path to get from start to goal
        path = path(pathLength:-1:1, :);
    else
        %disp('No path found to goal!');
    end
end

function moves = generateMoves(dimensions, connectivityType)
    % Generate possible moves based on dimensionality and connectivity type
    if strcmp(connectivityType, 'minimal')
        % Only orthogonal moves (2*dimensions moves)
        moves = zeros(2*dimensions, dimensions);
        for d = 1:dimensions
            move = zeros(1, dimensions);
            move(d) = 1;
            moves(2*d-1, :) = move;    % Positive direction
            moves(2*d, :) = -move;     % Negative direction
        end
    else % 'full' connectivity
        % All possible moves including diagonals (3^dimensions-1 moves)
        % Generate all combinations of -1, 0, 1 for each dimension
        [moveComponents{1:dimensions}] = ndgrid([-1 0 1]);
        movesList = zeros(3^dimensions, dimensions);
        
        for d = 1:dimensions
            componentValues = moveComponents{d}(:);
            movesList(:, d) = componentValues;
        end
        
        % Remove the all-zeros move (center point)
        zeroMove = all(movesList == 0, 2);
        moves = movesList;
        moves(zeroMove, :) = [];
    end
end

function idx = coordToLinearIndex(coords, gridSize)
    % Convert n-dimensional coordinates to linear index
    idx = coords(1);
    for d = 2:length(coords)
        idx = idx + (coords(d)-1) * prod(gridSize(1:d-1));
    end
end

function visualizePathND(map, path, explored, start, goal)
    % Visualize the path found by the algorithm
    % Currently supports visualization for 2D and 3D grids
    dims = ndims(map);
    
    if dims == 2
        % 2D visualization
        figure;
        imagesc(map);
        colormap([1 1 1; 0.5 0.5 0.5]); % White for free space, gray for obstacles
        hold on;
        
        % Plot explored points
        if ~isempty(explored)
            scatter(explored(:,2), explored(:,1), 20, 'blue', 'filled', 'MarkerFaceAlpha', 0.3);
        end
        
        % Plot path
        if ~isempty(path)
            plot(path(:,2), path(:,1), 'g-', 'LineWidth', 3);
            scatter(path(:,2), path(:,1), 30, 'green', 'filled');
        end
        
        % Plot start and goal
        scatter(start(2), start(1), 100, 'red', 'filled', 'MarkerEdgeColor', 'k');
        scatter(goal(2), goal(1), 100, 'magenta', 'filled', 'MarkerEdgeColor', 'k');
        
        title('Dijkstra Path Planning (2D)');
        axis image;
        
    elseif dims == 3
        % 3D visualization
        figure;
        
        % Plot obstacles
        [x, y, z] = ind2sub(size(map), find(map == 1));
        scatter3(x, y, z, 15, 'black', 'filled', 'MarkerFaceAlpha', 0.3);
        hold on;
        
        % Plot explored nodes
        if ~isempty(explored)
            scatter3(explored(:,1), explored(:,2), explored(:,3), 20, 'blue', 'filled', 'MarkerFaceAlpha', 0.3);
        end
        
        % Plot path
        if ~isempty(path)
            plot3(path(:,1), path(:,2), path(:,3), 'g-', 'LineWidth', 3);
            scatter3(path(:,1), path(:,2), path(:,3), 30, 'green', 'filled');
        end
        
        % Plot start and goal
        scatter3(start(1), start(2), start(3), 100, 'red', 'filled');
        scatter3(goal(1), goal(2), goal(3), 100, 'magenta', 'filled');
        
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Dijkstra Path Planning (3D)');
        grid on;
        axis equal;
        view(30, 30);
        
    else
        % For higher dimensions, just display a message
        disp('Visualization is only supported for 2D and 3D grids.');
        disp(['Path found with ' num2str(size(path, 1)) ' steps.']);
        disp(['Explored ' num2str(size(explored, 1)) ' nodes.']);
    end
end

% Example usage for 2D, 3D, and higher dimensions:
%{
% 2D Example
grid2D = zeros(20, 20);
grid2D(5:15, 10) = 1; % Add a wall
start2D = [1, 1];
goal2D = [20, 20];
[path2D, explored2D] = dijkstraND(grid2D, start2D, goal2D, 'full');
visualizePathND(grid2D, path2D, explored2D, start2D, goal2D);

% 3D Example
grid3D = zeros(20, 20, 20);
grid3D(:, 10, 5:15) = 1; % Add a wall
start3D = [1, 1, 1];
goal3D = [20, 20, 20];
[path3D, explored3D] = dijkstraND(grid3D, start3D, goal3D, 'full');
visualizePathND(grid3D, path3D, explored3D, start3D, goal3D);

% % 4D Example (no visualization)
% grid4D = zeros(10, 10, 10, 10);
% start4D = [1, 1, 1, 1];
% goal4D = [10, 10, 10, 10];
% [path4D, explored4D] = dijkstraND(grid4D, start4D, goal4D);

%}