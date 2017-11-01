function h = hEuclideanMaxA(searchProblem, start, goal)
%hEuclideanMaxA is a heuristic where the distance is estimated by the
%Euclidean distance and the time is calculated assuming that the car will
%continue accelearating with maxA (m/s^2) until reaching the goal.
arcL = searchProblem.arcL;
if isequal(pos2cell(start(1:2),0.5*arcL),pos2cell(goal,0.5*arcL))
    h = 0;
else
    hc = sqrt((start(1) - goal(:,1)).^2 + (start(2) - goal(:,2)).^2);
    maxA = 1; %put the maximum acceleration of the system here.
    ht = (sqrt(start(4)^2+2*maxA*hc)-start(4))/maxA;
    h = [hc ht]*searchProblem.w';
end
end