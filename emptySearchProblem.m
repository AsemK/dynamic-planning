function searchProblem = emptySearchProblem
%emptySearchProblem returns an empty searchProblem
searchProblem.grid   = [];
searchProblem.agents = zeros(0,5);
searchProblem.start  = zeros(0,4); %state (x, y, time, speed)
searchProblem.goal   = zeros(0,2); %position (x, y)
searchProblem.succ   = [];         %successor function   
searchProblem.h      = [];         %heurisitic function
searchProblem.carR   = 0.5;        %car radius
searchProblem.arcL   = 2;          %length of one movement
searchProblem.maxV   = 35;         %maximum allowed velocity
searchProblem.w      = [0.5 0.5];  %wights of [distance time] in the cost and heuristic
searchProblem.changes.grid = [];   %list of changed cells in the grid.obstacles
searchProblem.changes.agents = 0;  %binary array that is true when an agent is changed.
end

