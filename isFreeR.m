function b = isFreeR(searchProblem, pos)
%isFreeR returns true if the location is inside the grid and free from
%obstacles, otherwise false.
%Considers the whole car by checking also its edges, and hence very slow
%as it performs five checks for one position.
grid = searchProblem.grid;
carR = searchProblem.carR;
b = false;
[width, length] = size(grid.obstacles);
for y = [-carR 0 carR]
    p = pos + [0 y];
    cell = pos2cell(p, grid.unitL);
    if (cell(1) < 1) || (cell(1) > width)...
        || (cell(2) <= 0) || (cell(2) > length)...
        || grid.obstacles(cell(1), cell(2))
        return;
    end
end
for x = [-carR carR]
    p = pos + [x 0];
    cell = pos2cell(p, grid.unitL);
    if (cell(1) < 1) || (cell(1) > width)...
        || (cell(2) <= 0) || (cell(2) > length)...
        || grid.obstacles(cell(1), cell(2))
        return;
    end
end
b = true;
end