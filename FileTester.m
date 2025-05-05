close all
clear al;

fns = {
    (@(p, s)dijkstra(p)),
    (@(p, s)astar(p)),
    (@(p, s)dijkstra_random(p, 400*2^(s-1), 3))
     (@(p, s)astar_random(p, 400*2^(s-1), 3))
    };
fNames = {"Dijkstra", "A Star", "Dijk w/ Random Sampling", "A* w/ Random Sampling"};



seeds = 100:10:300;
pSpaces = {
 @(s)(NdMaze(2, s, 5, 'minimal')),
 @(s)(NdMaze(3, s, 5, 'minimal')),
 @(s)(NdMaze(2, s, 9, 'full')),
 @(s)(NdMaze(3, s, 9, 'full')),
 @(s)(NAgents(2, 2, s))
 %@(s)(convexPolygons([10, 10, 10], 3, s))
};
%pSpaces = [pSpaces NdMaze(4)];
%pSpaces = [pSpaces NAgents(2, 0)];
%pSpaces = [pSpaces tbd(params)];
%pSpaces = [pSpaces tbd(params)];

fs = length(fns);
ps = length(pSpaces);

durations = zeros(ps, fs);
pDistances = zeros(ps, fs);
sDistances = zeros(ps, fs);
samples = zeros(ps, fs);

durationsDev = zeros(ps, fs);
pDistancesDev = zeros(ps, fs);
sDistancesDev = zeros(ps, fs);
samplesDev = zeros(ps,fs);

for i = 1:ps
    if ps > 4 
       seeds = [50 150 170];
    end
    for j = 1:fs
        durs = zeros(length(seeds), 1);
        pDists = zeros(length(seeds), 1);
        sDists = zeros(length(seeds), 1);
        samps = zeros(length(seeds), 1);
        for k2 = 1:length(seeds)
            p1 = pSpaces{i};
            p = p1(seeds(k2) + fs);
            solSeed = 0;
            f = fns{j};
            tic;
            [path, cost] = f(p, solSeed);
            while length(path) == 0
                disp("retrying " + ps + " " + fs + " " + k2 + " " + solSeed)
                solSeed = solSeed + 1;
                [path, cost] = f(p, solSeed + fs);
            end
            dur = toc;
            pDist = 0;
            sDist = 0;
            nodes = length(path);
            for k = 2:nodes
                sDist = sDist + norm(path(k,:) - path(k-1,:));
                pDist = pDist + p.Distance(path(k-1,:), path(k,:));
            end
            samps(k2) = length(cost);
            durs(k2) = dur;
            pDists(k2) = pDist;
            sDists(k2) = sDist;
        end
        pDistances(i, j) = mean(pDists);
        sDistances(i, j) = mean(sDists);
        durations(i, j) = mean(durs);
        samples(i,j) = mean(samps);

        durationsDev(i, j) = std(durs);
        pDistancesDev(i, j) = std(pDists);
        sDistancesDev(i, j) = std(sDists);
        samplesDev(i,j) = std(samps);
    end
end

%xlabels = cellfun(@(x)(x(1)), pSpaces);
%xlabels = cellfun(@(x)(x.Name), xlabels1);

%lineLabels = fSpaces.Name;
%{
figure()
hold on
title("Comparison of time performance 1")
for i = 1:ps
    errorbar(1:fs, durations(i,:), durationsDev(i,:));%, "o", "filled");
end
ylabel("Algorithm Duration, Sec");
xticklabels(fNames);
legend(xlabels)
xlim([0, fs+1]);
hold off

%}




figure()
hold on
title("Comparison of Path Length by Test")
for i = 1:fs
    errorbar(1:ps, pDistances(:,i), pDistancesDev(:,i));%, "o", "filled");
end
ylabel("Path Length, units");
legend(fNames);
xticklabels(xlabels)
xlim([0, fs+1]);
hold off