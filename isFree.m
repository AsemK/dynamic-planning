function b = isFree(searchProblem, pos)
%isFree returns true if the location is inside the grid and free from
%obstacles, otherwise false.
%Considers only the position point (so doesn't guarantee that the whole car
%doesn't touch obstacles.
grid = searchProblem.grid;
if (pos(1) > 0) && (pos(1) <= grid.width)...
        && (pos(2) > 0) && (pos(2) <= grid.length)
    node = pos2cell(pos, grid.unitL);
    b = ~grid.obstacles(node(1), node(2));
else
    b = false;
end
end

