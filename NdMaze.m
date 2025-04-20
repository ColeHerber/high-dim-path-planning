classdef NdMaze < Problem
    properties 
        Dimensions
        Maze
        Start
        End
        Name
    end

    methods
        function obj = NdMaze(n)
            obj = obj@Problem();
            switch n
                case 2
                    Maze = %fill in the blanks
                case 3
                
                case 4

                case 5

                case 6

                otherwise
            end
        end
        obj.dimensions = size(obj.Maze);

        function out = IsObstacle(obj, coords)
            out = (obj.Maze(round(max(coords, obj.Dimensions - 1)) == 1);
        end
        
        function IsPath(obj, coords1, coords2)
            %todo, interpolate or high dimensional line cover alg
            %may be overkill given it only applies to 2-3 problems
        end
        
        
        function Distance(obj, coords1, coords2)
            
        end

    end
end