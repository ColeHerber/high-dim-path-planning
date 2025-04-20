classdef Problem
    properties
        Dimensions
        Name
        Start
        End
    end
    methods (Abstract)
        IsObstacle(obj, coords)
        IsPath(obj, coords1, coords2)
        Distance(obj, coords1, coords2)
    end
end