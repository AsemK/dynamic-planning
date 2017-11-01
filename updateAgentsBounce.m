function searchProblem = updateAgentsBounce(searchProblem, agentInd, step)
%updateAgentsBounce updates the the position and direction of the agents.
%If an agent hits the a wall it bounces back with a reflection angle. The
%input agentInd is the indix of the agent to be updated (is order when
%added to the searchProblem). Multiple indices are allowed.
agents = searchProblem.agents(agentInd,:);
grid = searchProblem.grid;
agents = getAgentsPos(agents, step);
changes = false(size(agents,1),1);
condition = agents(:,1)<=0 | agents(:,1)>=grid.width;
agents(condition,5) = 180-agents(condition,5);
changes(condition) = true;
condition = agents(:,2)<=0 | agents(:,2)>=grid.length;
agents(condition,5) = - agents(condition,5);
changes(condition) = true;
searchProblem.agents(agentInd,:) = agents;
searchProblem.changes.agents(agentInd) = changes;
end

