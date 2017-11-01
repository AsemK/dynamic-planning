function b = isFreeD(searchProblem, agents, pos)
%isFreeD returns true if the location is free from static and dynamic
%obstacles. Checks dynamic obstacles by checking if the input position is
%free from the agents (dynamic obstacl). Doesn't guarantee safety.
b = isFree(searchProblem, pos);
if  b && ~isempty(searchProblem.agents)
    carR = searchProblem.carR;
    b = sum(((agents(:,1)-pos(1)).^2+(agents(:,2)-pos(2)).^2) <= (agents(:,3)+carR).^2) == 0;
end

