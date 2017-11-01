function b = isChanged( searchProblem )
%isChanged returns true if changes have been added to the searchProblem.
if sum(searchProblem.changes.agents) == 0 ...
        && isempty(searchProblem.changes.grid)
    b = false;
else
    b = true;
end
end

