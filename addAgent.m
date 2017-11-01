function searchProblem = addAgent(searchProblem, x, y, r, speed, direction)
%addAgent adds a circular agent to the searchProblem. the input is expected
%to be: searchProblem, x, y, radius (m), speed (m/s), direction (degrees
%from the horizontal line).
searchProblem.agents = [searchProblem.agents; x, y, r, speed, direction];
end

