function h = hEuclideanFixedV(searchProblem, start, goal)
%hEuclideanMaxA is a heuristic where the distance is estimated by the
%Euclidean distance and the time is calculated assuming that the car will
%continue move with the same speed until reaching the goal.
if isequal(pos2cell(start(1:2),0.5*arcL),pos2cell(goal,0.5*arcL))
    h = 0;
else
    hc = sqrt((start(1) - goal(:,1)).^2 + (start(2) - goal(:,2)).^2);
    ht = hc/searchProblem.maxV;
    h = [hc ht]*searchProblem.w';
end
end