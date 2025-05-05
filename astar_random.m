function [path, explored] = astar_random(p, num_samples, step_size)
    [path, explored] = astar_nd_sampling(p.Dimensions, p.Start, p.End, (@(c)p.IsObstacle(c)), num_samples, step_size);
end

function [path, explored] = astar_nd_sampling(map_size, start, goal, obstacles, num_samples, step_size)
    % A* path planning with random sampling in N dimensions
    %
    % Inputs:
    %   map_size - vector containing the size of the map in each dimension
    %   start - vector containing start coordinates
    %   goal - vector containing goal coordinates
    %   obstacles - function handle that returns true if a point is in collision
    %   num_samples - number of random points to sample
    %   step_size - maximum distance between connected points
    %
    % Outputs:
    %   path - matrix where each row is a waypoint in the path
    %   cost - total path cost
    
    % Validate inputs
    num_dims = length(map_size);
    if length(start) ~= num_dims || length(goal) ~= num_dims
        error('Start and goal dimensions must match map dimensions');
    end
    
    % Create the roadmap
    % disp('Generating random samples...');
    samples = [start; goal]; % Include start and goal in samples
    
    % Generate random samples
    for i = 1:num_samples
        sample = zeros(1, num_dims);
        for d = 1:num_dims
            sample(d) = rand() * map_size(d) + 1;
        end
        
        % Add to samples if not in collision
        if ~obstacles(sample)
            samples = [samples; sample];
        end
    end
    
    num_vertices = size(samples, 1);
    
    % Create adjacency matrix
    % disp('Creating connectivity graph...');
    adj_matrix = inf(num_vertices, num_vertices);
    
    % Connect vertices if they are within step_size distance
    for i = 1:num_vertices
        for j = i+1:num_vertices
            dist = norm(samples(i,:) - samples(j,:)); % L2 norm
            
            if dist <= step_size
                % Check if connection is collision-free
                collision = false;
                
                % Check points along the path at small intervals
                num_checks = max(3, ceil(dist / (step_size/10)));
                for t = linspace(0, 1, num_checks)
                    p = samples(i,:) * (1-t) + samples(j,:) * t;
                    if obstacles(p)
                        collision = true;
                        break;
                    end
                end
                
                if ~collision
                    adj_matrix(i,j) = dist;
                    adj_matrix(j,i) = dist;
                end
            end
        end
    end
    
    % Run A* on the roadmap
    % disp('Running A* algorithm...');
    [path_indices, closed_set] = astar_graph(adj_matrix, 1, 2, samples, @l2_distance);
    explored = samples(closed_set,:);

    if isempty(path_indices)
        %warning('No path found');
        path = [];
        cost = inf;
        return;
    end
    
    % Convert indices to coordinates
    path = samples(path_indices, :);
    % disp('Path planning complete');
end

function [path, closed_set] = astar_graph(adj_matrix, start_idx, goal_idx, coords, heuristic_fn)
    % A* search on a graph
    %
    % Inputs:
    %   adj_matrix - adjacency matrix with edge costs
    %   start_idx - index of start node
    %   goal_idx - index of goal node
    %   coords - coordinates of each node
    %   heuristic_fn - handle to heuristic function
    %
    % Outputs:
    %   path - sequence of vertex indices from start to goal
    %   cost - total path cost
    
    n = size(adj_matrix, 1);
    
    % Initialize data structures
    closed_set = false(1, n);
    parent = zeros(1, n);
    
    g_score = inf(1, n);
    g_score(start_idx) = 0;
    
    f_score = inf(1, n);
    f_score(start_idx) = heuristic_fn(coords(start_idx,:), coords(goal_idx,:));
    
    % Priority queue (using a simple array for MATLAB)
    open_set = start_idx;
    
    while ~isempty(open_set)
        % Find vertex with smallest f_score
        [~, idx] = min(f_score(open_set));
        current = open_set(idx);
        
        % Check if we reached the goal
        if current == goal_idx
            path = reconstruct_path(parent, goal_idx);
            cost = g_score(goal_idx);
            return;
        end
        
        % Remove current from open set
        open_set(idx) = [];
        
        % Add current to closed set
        closed_set(current) = true;
        
        % Check neighbors
        for neighbor = 1:n
            % Skip if neighbor is not connected or already evaluated
            if adj_matrix(current, neighbor) == inf || closed_set(neighbor)
                continue;
            end
            
            % Calculate tentative g_score
            tentative_g = g_score(current) + adj_matrix(current, neighbor);
            
            % If we found a better path, update
            if tentative_g < g_score(neighbor)
                parent(neighbor) = current;
                g_score(neighbor) = tentative_g;
                f_score(neighbor) = g_score(neighbor) + heuristic_fn(coords(neighbor,:), coords(goal_idx,:));
                
                % Add to open set if not already there
                if ~ismember(neighbor, open_set)
                    open_set = [open_set, neighbor];
                end
            end
        end
    end
    
    % No path found
    path = [];
    cost = inf;
end

function path = reconstruct_path(parent, goal)
    % Reconstructs path from parent pointers
    path = goal;
    current = goal;
    
    while parent(current) ~= 0
        current = parent(current);
        path = [current, path];
    end
end

function dist = l2_distance(a, b)
    % L2 (Euclidean) distance heuristic
    dist = norm(a - b);
end
%{
% Example usage:
% Define map dimensions and obstacles
map_size = [100, 100, 100]; % 3D map
start = [10, 10, 10];
goal = [90, 90, 90];

% Define obstacles as a function that returns true if point is in collision
obstacles = @(p) norm(p - [50, 50, 50]) < 20; % Sphere obstacle

% Run A* with 1000 samples and maximum edge length of 15
[path, cost] = astar_nd_sampling(map_size, start, goal, obstacles, 1000, 15);

% Plot results if in 2D or 3D
if length(map_size) <= 3
    figure;
    if length(map_size) == 2
        plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
        hold on;
        plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        axis([0, map_size(1), 0, map_size(2)]);

        % Plot obstacles (simplified for visualization)
        [x, y] = meshgrid(1:map_size(1), 1:map_size(2));
        obs = zeros(size(x));
        for i = 1:numel(x)
            if obstacles([x(i), y(i)])
                obs(i) = 1;
            end
        end
        contour(x, y, obs, [0.5, 0.5], 'k');

        title('2D Path Planning with A*');
    else
        plot3(path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);
        hold on;
        plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        axis([0, map_size(1), 0, map_size(2), 0, map_size(3)]);

        % Simplified obstacle visualization
        [x, y, z] = meshgrid(linspace(0, map_size(1), 20), linspace(0, map_size(2), 20), linspace(0, map_size(3), 20));
        obs = zeros(size(x));
        for i = 1:numel(x)
            if obstacles([x(i), y(i), z(i)])
                obs(i) = 1;
            end
        end
        isosurface(x, y, z, reshape(obs, size(x)), 0.5);
        alpha(0.3);

        title('3D Path Planning with A*');
    end
    grid on;
end

%}