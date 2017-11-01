function grid = createGridMap(obstaclesMap, unitL)
%createEmptyGrid returns a new grid with obstacles corresponding to the
%input obstaclesMap binary matrix.
obstacles      = obstaclesMap(end:-1:1,:);
obstacles      = obstacles';
grid.unitL     = unitL;
grid.width     = unitL*size(obstacles,1);
grid.length    = unitL*size(obstacles,2);
grid.obstacles = obstacles;
end