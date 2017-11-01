function states = succRightCS(searchProblem, state)
%succRightCS returns the successors of a node on road that is in 
%the right direction. Same as succRightCSV, but collision checking is done
%using an unsafe approach (isFreeD function).
%The output is an array with rows of the form [cost state(4 elements)].
arcL = searchProblem.arcL;
pos = state(1:2);
states = zeros(0,5); %the list of successors states (empty now).
%for all available accelerations
for a = [-1 0 1]
    %claculate the speed (derivation is in the report)
    v = (state(4)+sqrt(state(4)^2+4*a*arcL))/2;
    if v<0 || v>searchProblem.maxV
        continue;
    end
    dt = arcL/v; %time of movement.
    t = state(3) + dt; %time of reaching the successors.
    cost = [arcL dt]*searchProblem.w';
    %agents positions at the time of arrival.
    agents = getAgentsPos(searchProblem.agents, t-searchProblem.start(3));
    for angle = [-30 -20 -10 0 10 20 30]
        if isFreeD(searchProblem, agents, pos+arcL.*[cosd(angle) sind(angle)])
            states = [states; [cost pos+arcL.*[cosd(angle) sind(angle)] t v]];
        end
    end
end
end
