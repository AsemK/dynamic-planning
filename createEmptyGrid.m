function grid = createEmptyGrid(gridWidth, gridLength, unitL)
%createEmptyGrid returns a new grid with no obstacles, start or goal
%positions.
%obstacles are represented by a binary matrix.
grid.unitL  = unitL;
grid.width  = floor(gridWidth/unitL)*unitL;
grid.length = floor(gridWidth/unitL)*unitL;
grid.obstacles = false(floor(gridWidth/unitL), floor(gridLength/unitL));
end

