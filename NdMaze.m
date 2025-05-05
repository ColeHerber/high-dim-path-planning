classdef NdMaze < Problem
    properties 
        % Dimensions
        % Start
        % End
        % Name
        % Conn
        Maze
    end

    methods
        function obj = NdMaze(n, seed, maxV, conn)
            rng(seed);
            obj = obj@Problem();
            obj.Conn = conn;
            out = generateMaze(n, maxV, [], [], false);
            obj.Maze = out{1} / 255;
            obj.Start = out{2};
            obj.End = out{3};
            obj.Name = "Maze: Dim " + n + " Seed " + seed + " Size " + maxV + " Conn " + conn;
            obj.Dimensions = ones(1, n) * maxV;
        end
   
        function out = IsObstacle(obj, coords)
           % if size(coords, 1) == 1
           %     coords = coords.';
           %  end
            target1 = num2cell(floor(min(coords, obj.Dimensions)));
            target2 = sub2ind(size(obj.Maze), target1{:});
            out = (obj.Maze(target2) == 1);
        end
        
        function out = IsPath(obj, coords1, coords2)
            if (floor(coords1) == coords1 && floor(coords2) == coords2 ...
                    && sum(coords1 - coords2) == 1)
                out = isObstacle(coords1) && isObstacle(coords2);
            else 
                pLen = norm(coords1 - coords2);
                out = true;
                step = (coords2 - coords1) / (floor(pLen / 0.1) + 1);
                for i = 1:(floor(pLen / 0.1) + 1)
                    out = out && IsObstacle(obj, coords1 + (i-1) * step);
                end
            end
        end
        
        
        function out = Distance(obj, coords1, coords2)
            out = norm(coords1 - coords2);
        end

        function out = ToGrid(obj)
            out = obj.Maze;
        end

    end
end



function out = generateMaze(mazeDim, mazeSize, startPos, goalPos, display)
% Usage:
%   generateAndVisualizeMaze(mazeDim, mazeSize)
%   generateAndVisualizeMaze(mazeDim, mazeSize, startPos, goalPos)
%
% Inputs:
%   mazeDim  - Dimension of maze: 2 for 2D, 3 for 3D.
%   mazeSize - Size of the maze (must be an odd number).
%   startPos - (Optional) Starting cell coordinates.
%              For 2D: [row, col]. Default: [2, 1].
%              For 3D: [row, col, slice]. Default: [2, 1, 2].
%   goalPos  - (Optional) Goal cell coordinates.
%              For 2D: [row, col]. Default: [mazeSize-1, mazeSize].
%              For 3D: [row, col, slice]. Default: [mazeSize-1, mazeSize, mazeSize-1].
if nargin < 2
    error('At least mazeDim and mazeSize must be provided.');
end

if mod(mazeSize, 2) == 0
    error('mazeSize must be an odd number.');
end

% Set default start and goal positions
if mazeDim == 2
    if nargin < 3 || isempty(startPos)
        startPos = [2, 1];   % Entrance on left side
    end
    if nargin < 4 || isempty(goalPos)
        goalPos = [mazeSize-1, mazeSize];  % Exit on right side
    end
    if nargin < 5 || isempty(display)
        display = true;
    end
elseif mazeDim == 3
    if nargin < 3 || isempty(startPos)
        startPos = [2, 1, 2];   % Entrance on one face
    end
    if nargin < 4 || isempty(goalPos)
        goalPos = [mazeSize-1, mazeSize, mazeSize-1];  % Exit on the opposite face
    end
    if nargin < 5 || isempty(display)
        display = true;
    end
else
    error('mazeDim must be either 2 or 3.');
end

%% Solveable Check
attempt = 0;
if mazeDim == 2
    solved = false;
    while ~solved %I hate matlab wtf is this not symbol
        attempt = attempt + 1;
        maze = generateMaze2D(mazeSize);
        [path, distances] = solveMaze2D(maze, startPos, goalPos);
        if ~isempty(path)
            solved = true;
        else
            disp(['2D Maze unsolvable on attempt ' num2str(attempt) '. Regenerating...']);
        end
    end
    if display
        disp(['2D Maze generated and solved successfully after ' num2str(attempt) ' attempt(s).']);
        visualizeMaze2D(maze, path, startPos, goalPos);
    end
        
elseif mazeDim == 3
    solved = false;
    while ~solved
        attempt = attempt + 1;
        maze = generateMaze3D(mazeSize);
        [path, distances] = solveMaze3D(maze, startPos, goalPos);
        if ~isempty(path)
            solved = true;
        else
            disp(['3D Maze unsolvable on attempt ' num2str(attempt) '. Regenerating...']);
        end
    end
    if display
        disp(['3D Maze generated and solved successfully after ' num2str(attempt) ' attempt(s).']);
        visualizeMaze3D(maze, path, startPos, goalPos);
    end
    
end

out = {maze, startPos, goalPos};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Nested Functions

%% 2D Maze Generation
function maze = generateMaze2D(n)
    maze = 255 * ones(n, n);
    % Start carving from cell (2,2)
    start_r = 2; start_c = 2;
    maze(start_r, start_c) = 0;
    maze = carveMaze2D(maze, start_r, start_c);
    % Create entrance and exit
    maze(2,1) = 0;
    maze(n-1, n) = 0;
end

function maze = carveMaze2D(maze, r, c)
    % Define moves: jumps of 2 cells in four directions.
    directions = [ -2, 0; 2, 0; 0, -2; 0, 2 ];
    directions = directions(randperm(4), :);
    for i = 1:4
        nr = r + directions(i,1);
        nc = c + directions(i,2);
        if nr > 1 && nr < size(maze,1) && nc > 1 && nc < size(maze,2)
            if maze(nr, nc) == 255
                % Remove wall between cells
                maze(r + directions(i,1)/2, c + directions(i,2)/2) = 0;
                maze(nr, nc) = 0;
                maze = carveMaze2D(maze, nr, nc);
            end
        end
    end
end

%% 3D Maze Generation
function maze = generateMaze3D(n)
    maze = 255 * ones(n, n, n);
    % Start from cell (2,2,2)
    start_x = 2; start_y = 2; start_z = 2;
    maze(start_x, start_y, start_z) = 0;
    maze = carveMaze3D(maze, start_x, start_y, start_z);
    % Create entrance and exit
    maze(2,1,2) = 0;
    maze(n-1, n, n-1) = 0;
end

function maze = carveMaze3D(maze, x, y, z)
    % Define moves: jumps of 2 cells in 6 directions.
    directions = [ -2, 0, 0; 2, 0, 0; 0, -2, 0; 0, 2, 0; 0, 0, -2; 0, 0, 2 ];
    directions = directions(randperm(6), :);
    for i = 1:6
        nx = x + directions(i,1);
        ny = y + directions(i,2);
        nz = z + directions(i,3);
        if nx > 1 && nx < size(maze,1) && ny > 1 && ny < size(maze,2) && nz > 1 && nz < size(maze,3)
            if maze(nx, ny, nz) == 255
                maze(x + directions(i,1)/2, y + directions(i,2)/2, z + directions(i,3)/2) = 0;
                maze(nx, ny, nz) = 0;
                maze = carveMaze3D(maze, nx, ny, nz);
            end
        end
    end
end

%% 2D Maze Solver (Wavefront/BFS)
function [path, distances] = solveMaze2D(maze, start, goal)
    [rows, cols] = size(maze);
    distances = -ones(rows, cols);
    goal_r = goal(1);
    goal_c = goal(2);
    distances(goal_r, goal_c) = 1;
    queue = [goal_r, goal_c];
    offsets = [ -1, 0; 1, 0; 0, -1; 0, 1 ];
    
    while ~isempty(queue) %
        current = queue(1,:);
        queue(1,:) = [];
        r = current(1); c = current(2);
        for j = 1:4
            nr = r + offsets(j,1);
            nc = c + offsets(j,2);
            if nr>=1 && nr<=rows && nc>=1 && nc<=cols
                if maze(nr, nc)==0 && distances(nr, nc)==-1
                    distances(nr, nc) = distances(r, c) + 1;
                    queue(end+1,:) = [nr, nc];
                end
            end
        end
    end
    
    if distances(start(1), start(2)) == -1
        path = [];
        return;
    end
    
    % Backtrack from start to goal
    path = start;
    current = start;
    current_dist = distances(start(1), start(2));
    while current_dist > 1
        found = false;
        for j = 1:4
            nr = current(1) + offsets(j,1);
            nc = current(2) + offsets(j,2);
            if nr>=1 && nr<=rows && nc>=1 && nc<=cols
                if distances(nr, nc) == current_dist - 1
                    path(end+1,:) = [nr, nc];
                    current = [nr, nc];
                    current_dist = distances(nr, nc);
                    found = true;
                    break;
                end
            end
        end
        if ~found
            error('Backtracking failed in 2D solver.');
        end
    end
end

%% 3D Maze Solver (Wavefront/BFS)
function [path, distances] = solveMaze3D(maze, start, goal)
    dims = size(maze);
    distances = -ones(dims);
    distances(goal(1), goal(2), goal(3)) = 1;
    queue = [goal];
    offsets = [ -1, 0, 0; 1, 0, 0; 0, -1, 0; 0, 1, 0; 0, 0, -1; 0, 0, 1 ];
    
    while ~isempty(queue)
        current = queue(1,:);
        queue(1,:) = [];
        r = current(1); c = current(2); z = current(3);
        for j = 1:6
            nr = r + offsets(j,1);
            nc = c + offsets(j,2);
            nz = z + offsets(j,3);
            if nr>=1 && nr<=dims(1) && nc>=1 && nc<=dims(2) && nz>=1 && nz<=dims(3)
                if maze(nr, nc, nz)==0 && distances(nr, nc, nz)==-1
                    distances(nr, nc, nz) = distances(r, c, z) + 1;
                    queue(end+1,:) = [nr, nc, nz];
                end
            end
        end
    end
    
    if distances(start(1), start(2), start(3)) == -1
        path = [];
        return;
    end
    
    % Backtrack from start to goal in 3D
    path = start;
    current = start;
    current_dist = distances(start(1), start(2), start(3));
    while current_dist > 1
        found = false;
        for j = 1:6
            nr = current(1) + offsets(j,1);
            nc = current(2) + offsets(j,2);
            nz = current(3) + offsets(j,3);
            if nr>=1 && nr<=dims(1) && nc>=1 && nc<=dims(2) && nz>=1 && nz<=dims(3)
                if distances(nr, nc, nz) == current_dist - 1
                    path(end+1,:) = [nr, nc, nz];
                    current = [nr, nc, nz];
                    current_dist = distances(nr, nc, nz);
                    found = true;
                    break;
                end
            end
        end
        if ~found
            error('Backtracking failed in 3D solver.');
        end
    end
end

%% Visualization for 2D Maze
function visualizeMaze2D(maze, path, start, goal)
    figure;
    imagesc(maze);
    colormap(gray);
    axis equal tight;
    title('2D Maze');
    hold on;
    % Plot the solution path (columns map to x, rows to y)
    plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    % Mark start and goal
    plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(2), goal(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Solution Path','Start','Goal');
    hold off;
end

%% Visualization for 3D Maze


function visualizeMaze3D(maze, path, start, goal)
    figure;
    hold on;
    % For 3D, draw each wall cell as a transparent gray cube.
    % Map matrix indices: row -> y, col -> x, slice -> z.
    [wallRow, wallCol, wallSlice] = ind2sub(size(maze), find(maze == 255));
    for i = 1:length(wallRow)
        drawCube(wallCol(i), wallRow(i), wallSlice(i));
    end
    % Plot the solution path (swap row and col for x,y coordinates)
    plot3(path(:,2), path(:,1), path(:,3), 'r-', 'LineWidth', 2);
    % Mark start and goal positions using scatter3
    scatter3(start(2), start(1), start(3), 100, 'g', 'filled');
    scatter3(goal(2), goal(1), goal(3), 100, 'b', 'filled');
    xlabel('X (columns)');
    ylabel('Y (rows)');
    zlabel('Z (slices)');
    title('3D Maze');
    axis equal;
    grid on;
    view(3);
    hold off;
    
    % Nested function to draw a cube at (x,y,z)
    function drawCube(x, y, z)
        % Create vertices for a cube centered at (x,y,z) with side length 1
        v = [x-0.5, y-0.5, z-0.5;
             x+0.5, y-0.5, z-0.5;
             x+0.5, y+0.5, z-0.5;
             x-0.5, y+0.5, z-0.5;
             x-0.5, y-0.5, z+0.5;
             x+0.5, y-0.5, z+0.5;
             x+0.5, y+0.5, z+0.5;
             x-0.5, y+0.5, z+0.5];
        faces = [1 2 3 4;
                 5 6 7 8;
                 1 2 6 5;
                 2 3 7 6;
                 3 4 8 7;
                 4 1 5 8];
        patch('Vertices', v, 'Faces', faces, 'FaceColor', [0,0,0], ...
              'FaceAlpha', 0.15, 'EdgeColor', 'none');
    end
end

end