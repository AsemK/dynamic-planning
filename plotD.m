function plotD(ax, searchProblem, searchInfo, t)
%plotD plots the elements of the input searchProblem as well as the path in
%the searchInfo.
%    if input searchInfo is empty, only the grid will be drawn. Else, also
%    the path will be drawn.
persistent gridDrawn;
persistent obstacleMap;
persistent agentC;
persistent carC;
persistent pathL;
%% Options
% ----------------------- colors -----------------------
gridColor     = [0.6 0.6 0.6]; %light gray
obstacleColor = [0.1 0.1 0.1]; %nearly black
startColor    = [0.5 0.5 1.0]; %pale blue
goalColor     = [1.0 0.5 0.5]; %pale red
pathColor     = [0.0 0.0 1.0]; %blue
agentsColor   = [1.0 0.0 0.0]; %red
% -------------------- other options --------------------
%if ticks is true, ticks numbers will be shown on each axis representing
%the cooredinates of the grid.
ticks = false;
%if gridLines is true a grid will be shown, otherwise no grid is drawn.
gridLines = false;
% -------------------- graph contents --------------------
plotPath = false;     % allows path line to be poltted.
if gridLines
    gridStyle = '-';
else
    gridStyle = 'none';
end
if isempty(gridDrawn)
    gridDrawn = false;
end
%% If grid is available, delete old graphics (if any) and draw the grid
if ~gridDrawn
    agentC = [];
    carC = [];
    pathL = [];
    Grid = searchProblem.grid;
    unitL = Grid.unitL;
    %reset the figure, produce new one if no figure was not provided.
    if isempty(ax)
        figure 
        ax = gca;
    else
        cla(ax);
    end
    % Figure style and aesthetics
    axis equal;
    axis([0 Grid.width 0 Grid.length])
    ax.XTick = unitL:unitL:Grid.width;
    ax.YTick = unitL:unitL:Grid.length;
    ax.TickDir = 'in';
    if ~ticks
        ax.XTickLabel = {};
        ax.YTickLabel = {};
        ax.TickLength = [0.00 0.035];
    end
    ax.LineWidth = 1.0;
    ax.GridAlpha = 1.0;
    ax.GridColor = gridColor;
    ax.XColor    = gridColor;
    ax.YColor    = gridColor;
    ax.Box = 'on';
    if gridLines
        grid on;
    else
        grid off;
    end
    set(gcf,'color','w');
    xlabel([num2str(Grid.width), ' m']);
    ylabel([num2str(Grid.length), ' m']);
    % plot obstacle
    % construct obstacles map
    obstacleMap = containers.Map('KeyType','double','ValueType','any');
    for x = 1:size(Grid.obstacles,1)
        for y = 1:size(Grid.obstacles,2)
            if Grid.obstacles(x, y) == 1
                obstacleC = rectangle(ax, 'Position', unitL.*[x-1 y-1 1 1], 'FaceColor', obstacleColor,...
                    'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
                obstacleMap(cell2key([x y]))=obstacleC;
            end
        end
    end
    gridDrawn = true;
end
if ~isempty(searchProblem.changes.grid)
    change = searchProblem.changes.grid;
    unitL = searchProblem.grid.unitL;
    for r = 1:size(change,1)
        key = cell2key(change(r,:));
        if isKey(obstacleMap,key)
            delete(obstacleMap(key));
            remove(obstacleMap,key);
        else
            obstacleC = rectangle(ax, 'Position', unitL.*[change(r,:)-1 1 1], 'FaceColor', obstacleColor,...
                    'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
            obstacleMap(key)=obstacleC;
        end
    end
end
if ~isempty(searchProblem.agents)
    agents = searchProblem.agents;
    lena = length(agentC);
    lenb = size(agents,1);
    for r = 1:lena
        rectangleC = agentC(r);
        rectangleC.Position = [agents(r,1:2)-agents(r,3),2*agents(r,3).*[1 1]];
    end
    if lena>lenb
        agentC = agentC(1:lena);
    else
        for r = lena+1:lenb
            rectangleC = rectangle(ax, 'Position', [agents(r,1:2)-agents(r,3), ...
                2*agents(r,3).*[1 1]], 'Curvature', [1 1],...
                'FaceColor', agentsColor, 'LineStyle', 'none');
            agentC = [agentC; rectangleC];
        end
    end
end
if ~isempty(t)
    title(ax, ['Time = ', num2str(t,'%04.2f'), 's']);
    if ~isempty(searchProblem) && ~isempty(searchInfo) && t<searchInfo.path(end,3)
        s = searchProblem.start;
        r = searchProblem.carR;
        if length(carC) < 1
            carC = rectangle(ax, 'Position', [s(1:2)-r, ...
                2*r.*[1 1]], 'Curvature', [1 1],...
                'FaceColor', startColor, 'LineStyle', 'none');
        else
            carC.Position = [s(1:2)-r, 2*r.*[1 1]];
        end
    end
end
if ~isempty(searchProblem) && ~isempty(searchInfo) && plotPath
    if isempty(pathL) || isChanged(searchProblem)
        delete(pathL);
        hold on
        pathL = plot(ax, searchInfo.path(:,1), searchInfo.path(:,2), '--',...
            'Color', pathColor, 'LineWidth', 1);
    end
end
drawnow;