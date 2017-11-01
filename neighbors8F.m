function states = neighbors8F(searchProblem, state)
%neighbors8F returns an array of the free neighbors (successors or 
%predecessors) of a node in an 8-connected grid. The successors have the
%same speed as the state (so the speed remains fixed).
%The output is an array with rows of the form [cost state(4 elements)].
arcL = searchProblem.arcL;
pos = state(1:2);
%the time needed to pass the distance using this speed (the car is
%assumed to take this speed directly after the current state).
dt = arcL/state(4);  
cost = [arcL dt]*searchProblem.w';
t = state(3) + dt;   %time at which the successor will be reached
agents = getAgentsPos(searchProblem.agents, t-searchProblem.start(3));
states = zeros(0,5); %the list of successors states (empty now).
for x = [-arcL, arcL]
    if isFreeD(searchProblem, agents, pos + [x 0])
        states = [states; [cost pos+[x 0] t state(4)]];
    end
    if isFreeD(searchProblem, agents, pos + [0 x])
        states = [states; [cost pos+[0 x] t state(4)]];
    end
end
t = state(3) + 1.4142*dt;
agents = getAgentsPos(searchProblem.agents, t);
for x = [-arcL, arcL]
    for y = [-arcL, arcL]
        if isFreeD(searchProblem, agents, pos + [x y])
            states = [states; [1.4142*cost pos+[x y] t state(4)]];
        end
    end
end
end
