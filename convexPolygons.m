classdef convexPolygons < Problem
    properties 
       Vectors;
       Dists;
       Num;
       Centers;
       nDim;
    end

    methods
        function obj = convexPolygons(scale, num, s)
            if size(scale, 1) == 1
                scale = scale.';
            end
            rng(s);
            obj = obj@Problem();
            obj.Dimensions = scale;
            obj.nDim = length(scale);
            obj.Conn = "full";
            obj.Num = num;
            obj.Vectors = normalize(rand(obj.nDim*2, obj.nDim, num) - 0.5, 2);
            obj.Dists = (rand(obj.nDim*2, num))* 0.4 * sqrt(obj.nDim*mean(scale));
            obj.Name = obj.nDim + " Dimension, " +num + " Objects,  Seed " + s;
            obj.Centers = rand(obj.nDim, num).*scale;
            while 1
                obj.Start = rand(obj.nDim, 1) .* obj.Dimensions;
                while obj.IsObstacle(obj.Start)
                    obj.Start = rand(obj.nDim, 1) .* obj.Dimensions;
                end
                for i = 1:15
                    obj.End = rand(obj.nDim, 1) .* obj.Dimensions;
                    while obj.IsObstacle(obj.End)
                        obj.End = rand(obj.nDim, 1) .* obj.Dimensions;
                    end
                    if norm(obj.Start - obj.End) > 0.7 * sqrt(obj.nDim*mean(scale))
                        return
                    end
                end
            end
        end

        function out = IsObstacle(obj, coords)
            if size(coords, 1) ~= 1
                coords = coords.';
            end
            out = 0;
            for i = 1:obj.Num
                src = rand(obj.nDim*2, obj.nDim) .* (coords - obj.Centers(i,:));
                dists = dot(src, obj.Vectors(:,:,i), 2);
                out = out + min(dists < obj.Dists(:,1));
            end
            if out > 0
                out = 1;
            end
        end
        
        function out = IsPath(obj, coords1, coords2)
            % didn't end up being used
        end
        
        
        function out = Distance(obj, coords1, coords2)
            out = norm(coords1 - coords2);
        end

        function out =ToGrid(obj)
            dim = obj.Dimensions;
             if size(dim, 1) ~= 1
                dim = dim.';
            end
            out = ones(dim);
            for i = 1:numel(out)
                ind1 = cell(1, ndims(out)); 
                [ind1{:}] = ind2sub(size(out),i);
                out(i) = obj.IsObstacle(cell2mat(ind1));
            end
        end

    end
end