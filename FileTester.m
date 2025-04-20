

fns = [];

% fns = [fns @(p, x1, x2)alg1(p x1, x2)];
% fns = [fns @(p, x1, x2)alg2(p x1, x2)];
% fns = [fns @(p, x1, x2)alg3(p x1, x2)];
% fns = [fns @(p, x1, x2)alg4(p x1, x2)];

pSpaces = [];

%pSpaces = [pSpaces NdMaze(2)];
%pSpaces = [pSpaces NdMaze(3)];
%pSpaces = [pSpaces NdMaze(4)];
%pSpaces = [pSpaces NAgents(2, 0)];
%pSpaces = [pSpaces tbd(params)];
%pSpaces = [pSpaces tbd(params)];

fs = max(size(fns));
ps = max(size(pSpaces));

durations = zeros(fs, ps);
pDistances = zeros(fs, ps);
sDistances = zeros(fs, ps);

for i = 1:ps
    for j = 1:fs
        p = pSpaces(i);
        f = fns(i);
        tic;
        path = f(p, p.Start, p.End);
        dur = toc;
        pDist = 0;
        sDist = 0;
        nodes = length(path);
        for k = 2:nodes
            sDist = sDist + norm(nodes(k) - nodes(k-1));
            pDist = pDist + p.Distance(p, nodes(k-1), nodes(k));
        end
        pDistances(i, j) = pDist;
        sDistances(i, j) = sDist;
        durations(i, j) = dur;
    end
end

xlabels = pSpaces.Name;
%lineLabels = fSpaces.Name;

figure()
hold on
title("Comparison of time performance")
for i = 1:ps
    scatter(durations(i,:));
    
end
hold off