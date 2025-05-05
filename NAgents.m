classdef NAgents < Problem
    properties 
        nDim
        nAgents
    end

    methods
        function obj = NAgents(nDim, nAgents, s)
            rng(s);
            obj = obj@Problem();
            obj.Dimensions = ones(1,nDim * nAgents) * 7;
            obj.nDim = nDim;
            obj.nAgents = nAgents;
            obj.Conn = "full";
            obj.Name = nAgents + " Agents, " +nDim + " Dim,  Seed " + s;
            values = 0:2;
            points1 = values(dec2base(0:3^nDim-1,3)-'0'+1);
            points = points1(all(diff(points1.')>=0, 1),:);
            % value = round(rand(1,1) * 3 ^ n) ^ nAgents;
            % Start = zeroes(nAgents*nDim,1);
            starts = randperm(length(points));
            ends = randperm(length(points));
            agentStarts = points(starts(1:nAgents),:) * 3;
            agentEnds = points(ends(1:nAgents),:) * 3;
            obj.Start = reshape(agentStarts.', 1, []) + 1;
            obj.End = reshape(agentEnds.', 1, []) + 1;
        end

        function out = IsObstacle(obj, coords)
           if size(coords, 1) ~= 1
                coords = coords.';
           end
            cRounded = min(floor(coords), obj.Dimensions);
            agents = reshape(cRounded.', [], obj.nDim);
            out = min(sum((agents == 1) + (agents == 4) + (agents == 7), 2)) < obj.nDim - 1;

            if obj.nAgents > 1
                pairs = nchoosek(1:obj.nAgents,2);
                for i = 1:size(pairs,1)
                    out = out || (norm(agents(pairs(i,1),:) - agents(pairs(i,2),:))) <= 1;
                end
            end

        end
        
        function out = IsPath(obj, coords1, coords2)

        end
        
        
        function out = Distance(obj, coords1, coords2)
            out = norm(coords1 - coords2);
        end

        function out =ToGrid(obj)
            out = ones(obj.Dimensions);
            for i = 1:numel(out)
                ind1 = cell(1, ndims(out)); 
                [ind1{:}] = ind2sub(size(out),i);
                out(i) = obj.IsObstacle(cell2mat(ind1));
            end
        end

    end
end