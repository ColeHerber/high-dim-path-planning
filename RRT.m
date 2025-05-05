function [path, explored] = RTT(p, maxV)
    T = [p.Start];
    path = [p.End];
    parent = [0];
    K = maxV;
    maxStep = 0.2*sqrt(p.nDim*mean(p.Dimensions));
    for i = 1:K
        target = rand(length(p.Dimensions),1) .* p.Dimensions;
        if rand() > 0.9 
            target = p.End;
        end
        [v, minI] = min(vecnorm(T - target));
        source = T(:,minI);
        newT = source + normalize((target - source)) * min(maxStep, norm(source + (target - source)));
        if isObstacle1(p, newT, source)
            for j = 1:5
                newT = (newT - source)*0.5 + source;
                if isObstacle1(p, newT, source)
                    continue
                end
                break
            end
        end
        if isObstacle1(p, newT, source)
            continue
        end
        T = [T newT];
        parent = [parent minI];
        if newT == p.End
            ind = length(parent);
            while ind > 0
                path = [path T(:,ind)];
                ind = parent(ind);
            end
            path = [path p.Start];
            path = reverse(path);
        end
        explored = T;
    end    
    
end


function out = isObstacle1(p, startV, endV);
num_checks = max(5, ceil((norm(startV - endV)) / (1/10)));
                for t = linspace(0, 1, num_checks)
                    vert = startV * (1-t) + endV * t;
                    if p.IsObstacle(vert)
                        out = 1;
                        return;
                    end
                end
                out = 0;
                return
end