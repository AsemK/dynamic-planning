function agent = getAgentsPos(agent, t)
%getAgentsPos returns the agent position after time t passes. Can deal with
%multiple agents.
    length = agent(:,4).*t;
    agent(:,1) = agent(:,1) + length.*cosd(agent(:,5));
    agent(:,2) = agent(:,2) + length.*sind(agent(:,5));
end