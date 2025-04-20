classdef NAgents < Problem
    properties 
        Dimensions
        Maze
        Name
    end

    methods
        function obj = NAgents(n, s)
            obj = obj@Problem();
            obj.Dimensions = ones(n, 1);
            switch n
                case 2
                
                case 3
                
                case 4

                case 5

                case 6

                otherwise
            end
            obj.Name = n + " Agents, Seed " + s;
        end

        function out = IsObstacle(obj, coords)
            
        end
        
        function IsPath(obj, coords1, coords2)

        end
        
        
        function Distance(obj, coords1, coords2)
            
        end

    end
end