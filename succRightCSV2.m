function states = succRightCSV2(searchProblem, state)
%succRightCSV2 returns the successors of a node on road that is in 
%the right direction. Same as succRightCSV, but collision times are
%calculated in a different way.
%The output is an array with rows of the form [cost state(4 elements)].
arcL = searchProblem.arcL;
pos = state(1:2);
agents = searchProblem.agents;
%agents velocity vectors
agentsV = [agents(:,4).*cosd(agents(:,5)), agents(:,4).*sind(agents(:,5))];
%agents positions at the time of the input state
agentsP = agents(:,1:2) + (state(3)-searchProblem.start(3)).*agentsV;
cc = [agentsP(:,1)-pos(1), agentsP(:,2)-pos(2)];
rr = agents(:,3)+searchProblem.carR;
%the C coefficients of the quadratic equations needed to check the time of
%collision.
C = dot(cc,cc,2)-rr.*rr;  
states = zeros(0,5);  %the list of successors states (empty now).
%for all available acceleration values
for a = [-1 0 1]
    %claculate the speed (derivation is in the report)
    v = (state(4)+sqrt(state(4)^2+4*a*arcL))/2;
    if ~isreal(v) || (v < 0) || (v > searchProblem.maxV)
        continue;
    end
    %the time needed to pass the distance using this speed (the car is
    %assumed to take this speed directly after the current state).
    dt = arcL/v;
    t = state(3) + dt; %the time at which the successor state is reached.
    cost = [arcL dt]*searchProblem.w'; 
    for angle = [-30 -20 -10 0 10 20 30]
        %chack first for the static obstacles
        if ~isFree(searchProblem, pos+arcL.*[cosd(angle) sind(angle)])
            continue;
        end
        %velocity vector of the successor
        stateV = v.*[cosd(angle) sind(angle)];
        aa = [agentsV(:,1)-stateV(1), agentsV(:,2)-stateV(2)];
        %the A and B coefficients of the quadratic equations.
        A = dot(aa,aa,2);
        B = 2*dot(aa,cc,2);
        %check the collision times of the car with all agents
        term1 = B.*B;
        term2 = 4.*A.*C;
        %collision times are considered only when the discriminant is real
        cond  = term1>term2; 
        discr = sqrt(term1(cond)-term2(cond)); %discriminant
        collisionTimes = [(-B(cond)+discr)./(2.*A(cond)), (-B(cond)-discr)./(2.*A(cond))];
        collisionTimes = collisionTimes(collisionTimes>=0);
        %if no obstacle detected add the state to the list of successor states
        if isempty(collisionTimes) || (min(collisionTimes) > dt)
            states = [states; [cost pos+arcL.*[cosd(angle) sind(angle)] t v]];
        end
    end
end
end