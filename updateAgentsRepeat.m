function searchProblem = updateAgentsRepeat(searchProblem, agentInd, step)
%updateAgentsRepeat updates the the position and direction of the agents.
%If an agent hits the a wall it comes back from the other direction with 
%the same angle. The input agentInd is the indix of the agent to be updated
%(its order when added to the searchProblem). Multiple indices are allowed.
agents = searchProblem.agents(agentInd,:);
grid = searchProblem.grid;
agents = getAgentsPos(agents, step);
changes = false(size(agents,1),1);
condition = agents(:,1)<=0 | agents(:,1)>=grid.width;
agents(condition,1) = grid.width-agents(condition,1);
changes(condition) = true;
condition = agents(:,2)<=0 | agents(:,2)>=grid.length;
agents(condition,2) = grid.length-agents(condition,2);
changes(condition) = true;
searchProblem.agents(agentInd,:) = agents;
searchProblem.changes.agents(agentInd) = changes;
end



