% Unified Adaptive Dimensionality Path Planning
% This script combines adaptive dimensionality with user-specified path planning algorithms
% Currently supports A* and Dijkstra's algorithm for N-dimensional spaces

function adaptive_dim_demo()
    % Main demo function that creates examples and visualizes results
    
    clc; close all;
    
    % Choose which example to run
    example = 2;  % 1=2D, 2=3D, 3=4D, 4=7D arm
    
    % Choose which planning algorithm to use
    planner = 'astar';  % Options: 'astar', 'dijkstra'
    
    switch example
        case 1
            % 2D Example
            map = zeros(30, 30);
            map(10:20, 15) = 1;  % vertical obstacle
            map(15, 5:10) = 1;   % horizontal obstacle
            
            start = [5, 5];
            goal = [25, 25];
            
            % Run the planner with specified algorithm
            path = adaptive_dim_planner(start, goal, map, planner);
            
            % Visualize
            figure; imagesc(1 - map); colormap(gray); axis equal tight; hold on;
            title(['2D Adaptive Dimensionality Path Planning with ' upper(planner)]);
            plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            if ~isempty(path)
                plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2);
            else
                disp('No path found.');
            end
            
        case 2
            % 3D Example
            map_size = [20, 20, 20];
            start = [3, 3, 3];
            goal = [17, 17, 17];
            
            % Create a 3D map with a planar obstacle
            map = false(map_size);
            map(8:12, 8:12, :) = true;    % Create a "wall" obstacle
            map(:, 12:16, 8:12) = true;   % Create another obstacle
            
            % Use the 3D variant of the algorithm
            path = adaptive_dim_planner_nd(start, goal, map, planner);
            
            % Visualize
            if ~isempty(path)
                figure;
                % Visualize the obstacles
                [x, y, z] = ind2sub(size(map), find(map));
                plot3(x, y, z, 'r.', 'MarkerSize', 8);
                hold on;
                
                % Visualize the path
                plot3(path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);
                plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
                plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
                
                grid on;
                axis equal;
                title(['3D Adaptive Dimensionality Path Planning with ' upper(planner)]);
                xlabel('X'); ylabel('Y'); zlabel('Z');
                view(45, 30);
            else
                disp('No path found.');
            end
    end
end
            
function path = adaptive_dim_planner(start, goal, map, planner_type)
    % Adaptive dimensionality path planner for 2D maps
    %
    % Inputs:
    %   start - [x, y] coordinates of start point
    %   goal - [x, y] coordinates of goal point
    %   map - 2D binary map where 1 represents obstacles and 0 represents free space
    %   planner_type - String specifying which planner to use ('astar' or 'dijkstra')
    %
    % Outputs:
    %   path - N x 2 matrix of waypoints from start to goal, empty if no path found
    
    % Check inputs
    if nargin < 4
        planner_type = 'astar';  % Default to A* if not specified
    end
    
    % Initialize regions that use full dimensionality
    fullDimRegions = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Initialize with start and goal regions
    radius = 2;
    fullDimRegions = addFullDimRegion(fullDimRegions, start, radius);
    fullDimRegions = addFullDimRegion(fullDimRegions, goal, radius);
    
    % Maximum iterations to prevent infinite loops
    max_iterations = 20;
    iter = 0;
    
    while iter < max_iterations
        iter = iter + 1;
        fprintf('Iteration %d\n', iter);
        
        % Create a hybrid dimensionality map
        hybrid_map = createHybridMap(map, fullDimRegions);
        
        % Find low-dimensional path with selected planner
        if strcmpi(planner_type, 'astar')
            path_ad = astar(map, start, goal, false, hybrid_map); % low-dim A*
        else
            path_ad = dijkstra(map, start, goal, false, hybrid_map); % low-dim Dijkstra
        end
        
        if isempty(path_ad)
            fprintf('No low-dimensional path found. Giving up.\n');
            path = [];
            return;
        end
        
        % Create tunnel based on low-dim path
        tunnel = createTunnel(map, path_ad);
        
        % Find high-dimensional path within tunnel with selected planner
        if strcmpi(planner_type, 'astar')
            path_tau = astar(tunnel, start, goal, true, hybrid_map); % high-dim A*
        else
            path_tau = dijkstra(tunnel, start, goal, true, hybrid_map); % high-dim Dijkstra
        end
        
        if isempty(path_tau)
            fprintf('No high-dimensional path found in tunnel.\n');
            % Add problematic region to full dimensionality
            X_end = path_ad(end, :);
            fullDimRegions = growFullDimRegion(fullDimRegions, X_end, radius);
            continue;
        end
        
        % Check if high-dimensional path is acceptable
        cost_ad = pathCost(path_ad);
        cost_tau = pathCost(path_tau);
        epsilon = 1.5;
        
        if cost_tau > epsilon * cost_ad
            fprintf('High-dimensional path too costly (%.2f vs %.2f).\n', cost_tau, cost_ad);
            % Identify and add trouble spot to full dimensionality
            X_r = identifyTroubleSpot(path_tau);
            fullDimRegions = addFullDimRegion(fullDimRegions, X_r, radius);
            continue;
        end
        
        % Path is acceptable
        fprintf('Found acceptable path in %d iterations.\n', iter);
        path = path_tau;
        return;
    end
    
    % Fallback to high-dimensional search if we reach max iterations
    fprintf('Reached maximum iterations. Using high-dimensional search.\n');
    if strcmpi(planner_type, 'astar')
        path = astar(map, start, goal, true, map); % high-dim A*
    else
        path = dijkstra(map, start, goal, true, map); % high-dim Dijkstra
    end
end

function path = adaptive_dim_planner_nd(start, goal, map, planner_type)
    % Adaptive dimensionality path planner for N-dimensional maps
    %
    % Inputs:
    %   start - 1 x N coordinates of start point
    %   goal - 1 x N coordinates of goal point
    %   map - N-dimensional binary map where true/1 represents obstacles and false/0 represents free space
    %   planner_type - String specifying which planner to use ('astar' or 'dijkstra')
    %
    % Outputs:
    %   path - M x N matrix of waypoints from start to goal, empty if no path found
    
    % Check inputs
    if nargin < 4
        planner_type = 'astar';  % Default to A* if not specified
    end
    
    % Convert logical map to numeric if needed
    if islogical(map)
        map = double(map);
    end
    
    % Initialize regions that use full dimensionality
    fullDimRegions = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Initialize with start and goal regions
    radius = 2;
    fullDimRegions = addFullDimRegion(fullDimRegions, start, radius);
    fullDimRegions = addFullDimRegion(fullDimRegions, goal, radius);
    
    % Maximum iterations to prevent infinite loops
    max_iterations = 20;
    iter = 0;
    
    while iter < max_iterations
        iter = iter + 1;
        fprintf('Iteration %d\n', iter);
        
        % Create a hybrid dimensionality map
        hybrid_map = createHybridMapND(map, fullDimRegions);
        
        % Find low-dimensional path with selected planner
        if strcmpi(planner_type, 'astar')
            path_ad = astarND(map, start, goal, false, hybrid_map); % low-dim A*
        else
            path_ad = dijkstraND(map, start, goal, false, hybrid_map); % low-dim Dijkstra
        end
        
        if isempty(path_ad)
            fprintf('No low-dimensional path found. Giving up.\n');
            path = [];
            return;
        end
        
        % Create tunnel based on low-dim path
        tunnel = createTunnelND(map, path_ad);
        
        % Find high-dimensional path within tunnel with selected planner
        if strcmpi(planner_type, 'astar')
            path_tau = astarND(tunnel, start, goal, true, hybrid_map); % high-dim A*
        else
            path_tau = dijkstraND(tunnel, start, goal, true, hybrid_map); % high-dim Dijkstra
        end
        
        if isempty(path_tau)
            fprintf('No high-dimensional path found in tunnel.\n');
            % Add problematic region to full dimensionality
            X_end = path_ad(end, :);
            fullDimRegions = growFullDimRegion(fullDimRegions, X_end, radius);
            continue;
        end
        
        % Check if high-dimensional path is acceptable
        cost_ad = pathCost(path_ad);
        cost_tau = pathCost(path_tau);
        epsilon = 1.5;
        
        if cost_tau > epsilon * cost_ad
            fprintf('High-dimensional path too costly (%.2f vs %.2f).\n', cost_tau, cost_ad);
            % Identify and add trouble spot to full dimensionality
            X_r = identifyTroubleSpot(path_tau);
            fullDimRegions = addFullDimRegion(fullDimRegions, X_r, radius);
            continue;
        end
        
        % Path is acceptable
        fprintf('Found acceptable path in %d iterations.\n', iter);
        path = path_tau;
        return;
    end
    
    % Fallback to high-dimensional search if we reach max iterations
    fprintf('Reached maximum iterations. Using high-dimensional search.\n');
    if strcmpi(planner_type, 'astar')
        path = astarND(map, start, goal, true, map); % high-dim A*
    else
        path = dijkstraND(map, start, goal, true, map); % high-dim Dijkstra
    end
end

function fullDimRegions = addFullDimRegion(fullDimRegions, coord, radius)
    % Add a full-dimensionality region to the map
    %
    % Inputs:
    %   fullDimRegions - Map container of existing regions
    %   coord - Coordinates of the new region center
    %   radius - Radius of the new region
    %
    % Outputs:
    %   fullDimRegions - Updated map container with the new region
    
    h = hashCoord(coord);
    fullDimRegions(h) = struct('center', coord, 'radius', radius);
    fprintf("Added full-dim region at [%s] with radius %d\n", num2str(coord), radius);
end

function fullDimRegions = growFullDimRegion(fullDimRegions, coord, growBy)
    % Grow an existing full-dimensionality region or add a new one
    %
    % Inputs:
    %   fullDimRegions - Map container of existing regions
    %   coord - Coordinates of the region to grow
    %   growBy - Amount to increase the radius by
    %
    % Outputs:
    %   fullDimRegions - Updated map container with the modified region
    
    h = hashCoord(coord);
    if isKey(fullDimRegions, h)
        region = fullDimRegions(h);
        region.radius = region.radius + growBy;
        fullDimRegions(h) = region;
        fprintf("Grew full-dim region at [%s] to radius %d\n", num2str(coord), region.radius);
    else
        fullDimRegions = addFullDimRegion(fullDimRegions, coord, growBy);
    end
end

function h = hashCoord(coord)
    % Convert coordinates to a string hash for use as a map key
    %
    % Inputs:
    %   coord - Vector of coordinates
    %
    % Outputs:
    %   h - String hash of the coordinates
    
    h = sprintf('%d', coord(1));
    for i = 2:length(coord)
        h = [h, '_', sprintf('%d', coord(i))];
    end
end

function hybrid_map = createHybridMap(map, fullDimRegions)
    % Create a 2D map where 1 indicates high-dimensionality regions
    %
    % Inputs:
    %   map - Original 2D map
    %   fullDimRegions - Map container of full-dimensionality regions
    %
    % Outputs:
    %   hybrid_map - 2D map with 1s in high-dimensionality regions
    
    % Create a map where 1 indicates high-dimensionality regions
    hybrid_map = zeros(size(map));
    keys = fullDimRegions.keys;
    
    for i = 1:length(keys)
        region = fullDimRegions(keys{i});
        center = region.center;
        radius = region.radius;
        
        % Mark the region in the hybrid map
        [rows, cols] = size(map);
        for r = max(1, center(1)-radius):min(rows, center(1)+radius)
            for c = max(1, center(2)-radius):min(cols, center(2)+radius)
                if norm([r c] - center) <= radius
                    hybrid_map(r, c) = 1;
                end
            end
        end
    end
end

function hybrid_map = createHybridMapND(map, fullDimRegions)
    % Create an N-dimensional map where 1 indicates high-dimensionality regions
    %
    % Inputs:
    %   map - Original N-dimensional map
    %   fullDimRegions - Map container of full-dimensionality regions
    %
    % Outputs:
    %   hybrid_map - N-dimensional map with 1s in high-dimensionality regions
    
    % Create a map where 1 indicates high-dimensionality regions
    hybrid_map = zeros(size(map));
    keys = fullDimRegions.keys;
    
    for i = 1:length(keys)
        region = fullDimRegions(keys{i});
        center = region.center;
        radius = region.radius;
        
        % Get number of dimensions
        n_dims = length(center);
        map_size = size(map);
        
        % Generate all grid points within a hypersphere around the center
        % This is an approximation - we enumerate all points in a hypercube
        % and then filter by distance
        
        % Build ranges for each dimension
        ranges = cell(1, n_dims);
        for d = 1:n_dims
            ranges{d} = max(1, center(d)-radius):min(map_size(d), center(d)+radius);
        end
        
        % Generate all combinations using ndgrid
        grid_points = cell(1, n_dims);
        [grid_points{:}] = ndgrid(ranges{:});
        
        % Calculate distances from center for all points
        dist = zeros(size(grid_points{1}));
        for d = 1:n_dims
            dist = dist + (grid_points{d} - center(d)).^2;
        end
        dist = sqrt(dist);
        
        % Find points within radius
        indices = dist <= radius;
        
        % Convert to linear indices
        linear_indices = sub2ind(size(dist), find(indices));
        
        % Mark these points in the hybrid map
        for j = 1:length(linear_indices)
            % Get subscripts for this point
            subs = cell(1, n_dims);
            for d = 1:n_dims
                subs{d} = grid_points{d}(linear_indices(j));
            end
            
            % Mark the point in the hybrid map
            hybrid_map(subs{:}) = 1;
        end
    end
end

function tunnel = createTunnel(map, path)
    % Create a tunnel around a low-dimensional path for high-dimensional search
    %
    % Inputs:
    %   map - Original 2D map
    %   path - Low-dimensional path (N x 2 matrix)
    %
    % Outputs:
    %   tunnel - Map where only points near the path are free space
    
    tunnel = ones(size(map));  % Start with all obstacles
    
    % Tunnel radius
    radius = 2;
    
    % Mark free space around each path point
    for i = 1:size(path, 1)
        point = path(i, :);
        [rows, cols] = size(map);
        
        for r = max(1, point(1)-radius):min(rows, point(1)+radius)
            for c = max(1, point(2)-radius):min(cols, point(2)+radius)
                if norm([r c] - point) <= radius && map(r, c) == 0
                    tunnel(r, c) = 0;  % Mark as free space
                end
            end
        end
    end
end

function tunnel = createTunnelND(map, path)
    % Create a tunnel around a low-dimensional path for high-dimensional search in N-dimensional space
    %
    % Inputs:
    %   map - Original N-dimensional map
    %   path - Low-dimensional path (M x N matrix)
    %
    % Outputs:
    %   tunnel - Map where only points near the path are free space
    
    tunnel = ones(size(map));  % Start with all obstacles
    
    % Tunnel radius
    radius = 2;
    
    % Get number of dimensions
    n_dims = size(path, 2);
    map_size = size(map);
    
    % Mark free space around each path point
    for i = 1:size(path, 1)
        point = path(i, :);
        
        % Build ranges for each dimension
        ranges = cell(1, n_dims);
        for d = 1:n_dims
            ranges{d} = max(1, point(d)-radius):min(map_size(d), point(d)+radius);
        end
        
        % Generate all combinations using ndgrid
        grid_points = cell(1, n_dims);
        [grid_points{:}] = ndgrid(ranges{:});
        
        % Calculate distances from center
        dist = zeros(size(grid_points{1}));
        for d = 1:n_dims
            dist = dist + (grid_points{d} - point(d)).^2;
        end
        dist = sqrt(dist);
        
        % Find points within radius
        indices = dist <= radius;
        
        % Mark these points in the tunnel
        for j = 1:numel(grid_points{1})
            if indices(j)
                % Get subscripts for this point
                subs = cell(1, n_dims);
                for d = 1:n_dims
                    subs{d} = grid_points{d}(j);
                end
                
                % Check if this point is free in original map
                is_free = (map(subs{:}) == 0);
                
                if is_free
                    tunnel(subs{:}) = 0;  % Mark as free space
                end
            end
        end
    end
end

function cost = pathCost(path)
    % Calculate the cost of a path as the sum of Euclidean distances
    %
    % Inputs:
    %   path - N x M matrix of waypoints
    %
    % Outputs:
    %   cost - Total path cost
    
    cost = 0;
    for i = 1:size(path, 1) - 1
        cost = cost + norm(path(i+1, :) - path(i, :));
    end
end

function X_r = identifyTroubleSpot(path)
    % Identify a trouble spot in the path for adding full dimensionality
    % This is a simple implementation that picks a point near the middle
    %
    % Inputs:
    %   path - N x M matrix of waypoints
    %
    % Outputs:
    %   X_r - Coordinates of the trouble spot
    
    % A more sophisticated implementation would identify points with high curvature
    % or where the path deviates significantly from a straight line
    
    % For now, select a point near the middle of the path
    mid_idx = round(size(path, 1) / 2);
    X_r = path(mid_idx, :);
end

function path = astar(map, start, goal, highDim, hybrid_map)
    % A* search algorithm for path planning
    %
    % Inputs:
    %   map - 2D map where 1s represent obstacles
    %   start - [row, col] coordinates of start point
    %   goal - [row, col] coordinates of goal point
    %   highDim - Boolean flag indicating if high-dimensionality search should be used
    %   hybrid_map - Map indicating high-dimensionality regions
    %
    % Outputs:
    %   path - N x 2 matrix of waypoints, empty if no path found
    
    if nargin < 5
        hybrid_map = [];
    end
    
    % Define grid dimensions
    [rows, cols] = size(map);
    
    % Initialize data structures
    closed_set = zeros(rows, cols);  % Nodes that have been evaluated
    open_set = zeros(rows, cols);    % Nodes to be evaluated
    came_from = cell(rows, cols);    % For path reconstruction
    
    g_score = inf(rows, cols);       % Cost from start to current node
    f_score = inf(rows, cols);       % Estimated total cost (g + heuristic)
    
    % Add start node to open set
    open_set(start(1), start(2)) = 1;
    g_score(start(1), start(2)) = 0;
    f_score(start(1), start(2)) = heuristic(start, goal);
    
    % Define movement directions
    if highDim && ~isempty(hybrid_map) && hybrid_map(start(1), start(2)) == 1
        % High-dimensionality: allow 8-connected movements
        moves = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    else
        % Low-dimensionality: restrict to 4-connected movements
        moves = [-1 0; 1 0; 0 -1; 0 1];
    end
    
    % Main A* loop
    while any(open_set(:))
        % Find node in open set with lowest f_score
        [min_f, ~] = min(f_score(open_set == 1));
        [current_i, current_j] = ind2sub([rows, cols], find(open_set == 1 & f_score == min_f, 1));
        current = [current_i, current_j];
        
        % Check if we've reached the goal
        if all(current == goal)
            path = reconstructPath(came_from, goal);
            return;
        end
        
        % Remove current from open set and add to closed set
        open_set(current(1), current(2)) = 0;
        closed_set(current(1), current(2)) = 1;
        
        % Check neighbors
        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > rows || ...
               neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end
            
            % Check if neighbor is in closed set
            if closed_set(neighbor(1), neighbor(2)) == 1
                continue;
            end
            
            % Check if neighbor is an obstacle
            if map(neighbor(1), neighbor(2)) == 1
                continue;
            end
            
            % Determine movement cost (diagonal moves cost more)
            move_cost = norm(moves(i, :));
            
            % Additional cost for transitioning between dimensionalities
            dim_transition_cost = 0;
            if ~isempty(hybrid_map)
                current_dim = hybrid_map(current(1), current(2));
                neighbor_dim = hybrid_map(neighbor(1), neighbor(2));
                if current_dim ~= neighbor_dim
                    dim_transition_cost = 0.5;  % Penalty for dimension transitions
                end
            end
            
            % Calculate tentative g score
            tentative_g = g_score(current(1), current(2)) + move_cost + dim_transition_cost;
            
            % Check if this path is better
            if tentative_g < g_score(neighbor(1), neighbor(2))
                % This is a better path, record it
                came_from{neighbor(1), neighbor(2)} = current;
                g_score(neighbor(1), neighbor(2)) = tentative_g;
                f_score(neighbor(1), neighbor(2)) = tentative_g + heuristic(neighbor, goal);
                
                % Add neighbor to open set if not already there
                open_set(neighbor(1), neighbor(2)) = 1;
            end
        end
    end
    
    % No path found
    path = [];
end

function path = dijkstra(map, start, goal, highDim, hybrid_map)
    % Dijkstra's algorithm for path planning
    %
    % Inputs:
    %   map - 2D map where 1s represent obstacles
    %   start - [row, col] coordinates of start point
    %   goal - [row, col] coordinates of goal point
    %   highDim - Boolean flag indicating if high-dimensionality search should be used
    %   hybrid_map - Map indicating high-dimensionality regions
    %
    % Outputs:
    %   path - N x 2 matrix of waypoints, empty if no path found
    
    if nargin < 5
        hybrid_map = [];
    end
    
    % Define grid dimensions
    [rows, cols] = size(map);
    
    % Initialize data structures
    closed_set = zeros(rows, cols);  % Nodes that have been evaluated
    open_set = zeros(rows, cols);    % Nodes to be evaluated
    came_from = cell(rows, cols);    % For path reconstruction
    
    dist = inf(rows, cols);          % Distance from start
    
    % Add start node to open set
    open_set(start(1), start(2)) = 1;
    dist(start(1), start(2)) = 0;
    
    % Define movement directions
    if highDim && ~isempty(hybrid_map) && hybrid_map(start(1), start(2)) == 1
        % High-dimensionality: allow 8-connected movements
        moves = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    else
        % Low-dimensionality: restrict to 4-connected movements
        moves = [-1 0; 1 0; 0 -1; 0 1];
    end
    
    % Main Dijkstra loop
    while any(open_set(:))
        % Find node in open set with lowest distance
        [min_dist, ~] = min(dist(open_set == 1));
        [current_i, current_j] = ind2sub([rows, cols], find(open_set == 1 & dist == min_dist, 1));
        current = [current_i, current_j];
        
        % Check if we've reached the goal
        if all(current == goal)
            path = reconstructPath(came_from, goal);
            return;
        end
        
        % Remove current from open set and add to closed set
        open_set(current(1), current(2)) = 0;
        closed_set(current(1), current(2)) = 1;
        
        % Check neighbors
        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > rows || ...
               neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end
            
            % Check if neighbor is in closed set
            if closed_set(neighbor(1), neighbor(2)) == 1
                continue;
            end
            
            % Check if neighbor is an obstacle
            if map(neighbor(1), neighbor(2)) == 1
                continue;
            end
            
            % Determine movement cost (diagonal moves cost more)
            move_cost = norm(moves(i, :));
            
            % Additional cost for transitioning between dimensionalities
            dim_transition_cost = 0;
            if ~isempty(hybrid_map)
                current_dim = hybrid_map(current(1), current(2));
                neighbor_dim = hybrid_map(neighbor(1), neighbor(2));
                if current_dim ~= neighbor_dim
                    dim_transition_cost = 0.5;  % Penalty for dimension transitions
                end
            end
            
            % Calculate alternative distance
            alt_dist = dist(current(1), current(2)) + move_cost + dim_transition_cost;
            
            % Check if this path is better
            if alt_dist < dist(neighbor(1), neighbor(2))
                % This is a better path, record it
                came_from{neighbor(1), neighbor(2)} = current;
                dist(neighbor(1), neighbor(2)) = alt_dist;
                
                % Add neighbor to open set if not already there
                open_set(neighbor(1), neighbor(2)) = 1;
            end
        end
    end
    
    % No path found
    path = [];
end

function path = astarND(map, start, goal, highDim, hybrid_map)
    % A* search algorithm for path planning in N-dimensional space
    %
    % Inputs:
    %   map - N-dimensional map where 1s represent obstacles
    %   start - 1 x N vector of start coordinates
    %   goal - 1 x N vector of goal coordinates
    %   highDim - Boolean flag indicating if high-dimensionality search should be used
    %   hybrid_map - Map indicating high-dimensionality regions
    %
    % Outputs:
    %   path - M x N matrix of waypoints, empty if no path found
    
    % For N-dimensional spaces, we'll use a different approach with a priority queue
    % This is a simplified implementation for demonstration
    
    % Number of dimensions
    n_dims = length(start);
    map_size = size(map);
    
    % Create priority queue for open set (using a simple sorted list)
    open_set = struct('pos', start, 'g', 0, 'f', heuristicND(start, goal), 'parent', []);
    open_set = {open_set};
    
    % Track visited nodes using a hash map
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    
    % Generate movement directions for N dimensions
    moves = generateMovesND(n_dims, highDim);
    
    while ~isempty(open_set)
        % Get node with lowest f-score
        [~, min_idx] = min(cellfun(@(x) x.f, open_set));
        current = open_set{min_idx};
        open_set(min_idx) = [];
        
        % Check if goal is reached
        if all(current.pos == goal)
            % Reconstruct path
            path = reconstructPathND(current);
            return;
        end
        
        % Mark as visited
        visited(hashCoord(current.pos)) = true;
        
        % Check all neighbors
        for i = 1:size(moves, 1)
            neighbor_pos = current.pos + moves(i, :);
            
            % Check bounds
            if any(neighbor_pos < 1) || any(neighbor_pos > map_size)
                continue;
            end
            
            % Skip if already visited
            if isKey(visited, hashCoord(neighbor_pos)) && visited(hashCoord(neighbor_pos))
                continue;
            end
            
            % Check if neighbor is an obstacle
            % Convert position to linear index
            neighbor_idx = sub2ind(map_size, neighbor_pos(1), neighbor_pos(2), neighbor_pos(3:end));
            if map(neighbor_idx) == 1
                continue;
            end
            
            % Determine movement cost
            move_cost = norm(moves(i, :));
            
            % Additional cost for transitioning between dimensionalities
            dim_transition_cost = 0;
            if ~isempty(hybrid_map)
                current_idx = sub2ind(map_size, current.pos(1), current.pos(2), current.pos(3:end));
                current_dim = hybrid_map(current_idx);
                neighbor_dim = hybrid_map(neighbor_idx);
                if current_dim ~= neighbor_dim
                    dim_transition_cost = 0.5;  % Penalty for dimension transitions
                end
            end
            
            % Calculate g score
            g_score = current.g + move_cost + dim_transition_cost;
            
            % Check if node is already in open set with better g score
            already_in_open = false;
            for j = 1:length(open_set)
                if all(open_set{j}.pos == neighbor_pos)
                    already_in_open = true;
                    if open_set{j}.g <= g_score
                        break;  % Already found better path
                    else
                        % Update with better path
                        open_set{j}.g = g_score;
                        open_set{j}.f = g_score + heuristicND(neighbor_pos, goal);
                        open_set{j}.parent = current;
                    end
                    break;
                end
            end
            
            % Add to open set if not already there
            if ~already_in_open
                f_score = g_score + heuristicND(neighbor_pos, goal);
                neighbor = struct('pos', neighbor_pos, 'g', g_score, 'f', f_score, 'parent', current);
                open_set{end+1} = neighbor;
            end
        end
    end
    
    % No path found
    path = [];
end

function path = dijkstraND(map, start, goal, highDim, hybrid_map)
    % Dijkstra's algorithm for path planning in N-dimensional space
    %
    % Inputs:
    %   map - N-dimensional map where 1s represent obstacles
    %   start - 1 x N vector of start coordinates
    %   goal - 1 x N vector of goal coordinates
    %   highDim - Boolean flag indicating if high-dimensionality search should be used
    %   hybrid_map - Map indicating high-dimensionality regions
    %
    % Outputs:
    %   path - M x N matrix of waypoints, empty if no path found
    
    % Number of dimensions
    n_dims = length(start);
    map_size = size(map);
    
    % Create priority queue for open set (using a simple sorted list)
    open_set = struct('pos', start, 'dist', 0, 'parent', []);
    open_set = {open_set};
    
    % Track visited nodes using a hash map
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    
    % Generate movement directions for N dimensions
    moves = generateMovesND(n_dims, highDim);
    
    while ~isempty(open_set)
        % Get node with lowest distance
        [~, min_idx] = min(cellfun(@(x) x.dist, open_set));
        current = open_set{min_idx};
        open_set(min_idx) = [];
        
        % Check if goal is reached
        if all(current.pos == goal)
            % Reconstruct path
            path = reconstructPathND(current);
            return;
        end
        
        % Mark as visited
        visited(hashCoord(current.pos)) = true;
        
        % Check all neighbors
        for i = 1:size(moves, 1)
            neighbor_pos = current.pos + moves(i, :);
            
            % Check bounds
            if any(neighbor_pos < 1) || any(neighbor_pos > map_size)
                continue;
            end
            
            % Skip if already visited
            if isKey(visited, hashCoord(neighbor_pos)) && visited(hashCoord(neighbor_pos))
                continue;
            end
            
            % Check if neighbor is an obstacle
            % Convert position to linear index
            neighbor_idx = sub2ind(map_size, neighbor_pos(1), neighbor_pos(2), neighbor_pos(3:end));
            if map(neighbor_idx) == 1
                continue;
            end
            
            % Determine movement cost
            move_cost = norm(moves(i, :));
            
            % Additional cost for transitioning between dimensionalities
            dim_transition_cost = 0;
            if ~isempty(hybrid_map)
                current_idx = sub2ind(map_size, current.pos(1), current.pos(2), current.pos(3:end));
                current_dim = hybrid_map(current_idx);
                neighbor_dim = hybrid_map(neighbor_idx);
                if current_dim ~= neighbor_dim
                    dim_transition_cost = 0.5;  % Penalty for dimension transitions
                end
            end
            
            % Calculate distance
            dist = current.dist + move_cost + dim_transition_cost;
            
            % Check if node is already in open set with better distance
            already_in_open = false;
            for j = 1:length(open_set)
                if all(open_set{j}.pos == neighbor_pos)
                    already_in_open = true;
                    if open_set{j}.dist <= dist
                        break;  % Already found better path
                    else
                        % Update with better path
                        open_set{j}.dist = dist;
                        open_set{j}.parent = current;
                    end
                    break;
                end
            end
            
            % Add to open set if not already there
            if ~already_in_open
                neighbor = struct('pos', neighbor_pos, 'dist', dist, 'parent', current);
                open_set{end+1} = neighbor;
            end
        end
    end
    
    % No path found
    path = [];
end

function h = heuristic(node, goal)
    % Manhattan distance heuristic for A*
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function h = heuristicND(node, goal)
    % Manhattan distance heuristic for N-dimensional A*
    h = sum(abs(node - goal));
end

function path = reconstructPath(came_from, goal)
    % Reconstruct path from start to goal using parent pointers
    %
    % Inputs:
    %   came_from - Cell array of parent pointers
    %   goal - [row, col] coordinates of goal point
    %
    % Outputs:
    %   path - N x 2 matrix of waypoints from start to goal
    
    path = goal;
    current = goal;
    
    while ~isempty(came_from{current(1), current(2)})
        current = came_from{current(1), current(2)};
        path = [current; path];
    end
end

function path = reconstructPathND(node)
    % Reconstruct path from start to goal using parent pointers in N dimensions
    %
    % Inputs:
    %   node - Structure with pos and parent fields
    %
    % Outputs:
    %   path - M x N matrix of waypoints from start to goal
    
    path = node.pos;
    current = node;
    
    while ~isempty(current.parent)
        current = current.parent;
        path = [current.pos; path];
    end
end

function moves = generateMovesND(n_dims, highDim)
    % Generate movement directions for N-dimensional space
    %
    % Inputs:
    %   n_dims - Number of dimensions
    %   highDim - Boolean flag indicating if high-dimensionality movements should be used
    %
    % Outputs:
    %   moves - Matrix of movement vectors
    
    if highDim
        % Generate all possible combinations of -1, 0, 1 for each dimension
        % Excluding the zero vector (no movement)
        
        % Generate all combinations using recursive function
        directions = [-1, 0, 1];
        combos = generate_combinations(directions, n_dims);
        
        % Remove the zero vector (all zeros)
        zero_idx = all(combos == 0, 2);
        moves = combos(~zero_idx, :);
    else
        % Low-dimensionality: only axis-aligned movements
        moves = zeros(2*n_dims, n_dims);
        for d = 1:n_dims
            moves(2*d-1, d) = 1;    % Positive movement
            moves(2*d, d) = -1;     % Negative movement
        end
    end
end

function combos = generate_combinations(values, dims)
    % Generate all combinations of values for given dimensions
    %
    % Inputs:
    %   values - Vector of possible values for each dimension
    %   dims - Number of dimensions
    %
    % Outputs:
    %   combos - Matrix of all combinations
    
    if dims == 1
        combos = values(:);
    else
        sub_combos = generate_combinations(values, dims-1);
        n_sub = size(sub_combos, 1);
        n_vals = length(values);
        
        combos = zeros(n_sub * n_vals, dims);
        
        idx = 1;
        for i = 1:n_vals
            for j = 1:n_sub
                combos(idx, 1) = values(i);
                combos(idx, 2:dims) = sub_combos(j, :);
                idx = idx + 1;
            end
        end
    end
end