function states = succRight5S(searchProblem, state)
%succRight5S returns the free successors of the input state for a road that
%is in the right direction from the five srrounding nodes.
%The function returns successors with different speeds based on the 
%acceleration constraints.
%Collision checking is done using an unsafe approach (isFreeD function).
%The output is an array with rows of the form [cost state(4 elements)].
arcL = searchProblem.arcL;
pos = state(1:2);  
states = zeros(0,5); %the list of successors states (empty now).
for a = [-1 0 1]
    %claculate the speed (derivation is in the report)
    v = (state(4)+sqrt(state(4)^2+4*a*arcL))/2;
    if v<0 || v>searchProblem.maxV
        continue;
    end
    dt = arcL/v;  %time of movement.
    t = state(3) + dt; %time of reaching the successors.
    cost = [arcL dt]*searchProblem.w';
    %agents positions at the time of arrival.
    agents = getAgentsPos(searchProblem.agents, t-searchProblem.start(3));
    if isFreeD(searchProblem, agents, pos + [arcL 0])
        states = [states; [cost pos+[arcL 0] t v]];
    end
    for y = [-arcL, arcL]
        if isFreeD(searchProblem, agents, pos + [0 y])
            states = [states; [cost pos+[0 y] t v]];
        end
    end
    t = state(3) + 1.4142*dt;
    agents = getAgentsPos(searchProblem.agents, t);
    for y = [-arcL, arcL]
        if isFreeD(searchProblem, agents, pos + [arcL y])
            states = [states; [1.4142*cost pos+[arcL y] t v]];
        end
    end
end
end
