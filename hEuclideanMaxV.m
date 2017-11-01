function h = hEuclideanMaxV(searchProblem, start, goal)
%hEuclideanMaxA is a heuristic where the distance is estimated by the
%Euclidean distance and the time is calculated assuming that the car will
%continue move with the same speed until reaching the goal.
cellL = searchProblem.cellL;
if isequal(pos2cell(start(1:2),cellL),pos2cell(goal,cellL))
    h = 0;
else
    hc = sqrt((start(1) - goal(:,1)).^2 + (start(2) - goal(:,2)).^2);
    ht = hc/start(4);
    h = [hc ht]*searchProblem.w';
end
end