close all;
clear functions; %necessary for correct operation of plotD
%% Define the searchProblem
searchProblem = emptySearchProblem;
searchProblem.grid = roadGrid;
searchProblem = addAgent(searchProblem, 5, 3, 1, 7, 0);
searchProblem = addAgent(searchProblem, 3, 7, 1, 10, 0);
searchProblem = addAgent(searchProblem, searchProblem.grid.width, 10, 1, 6, 180);
searchProblem = addAgent(searchProblem, searchProblem.grid.width, 14, 1, 9, 180);
searchProblem = addAgent(searchProblem, 15, searchProblem.grid.length, 0.5, 2, 270);
searchProblem = addAgent(searchProblem, 30, 0, 0.5, 4, 90);
searchProblem.start = [0.5, 2.5, 0, 20];
searchProblem.goal  = [4.5, 14.5];
searchProblem.carR   = 0.5;        %car radius
searchProblem.arcL   = 2;          %length of one movement
searchProblem.cellL  = 1;
searchProblem.speedR = 10;
searchProblem.maxV   = 35;         %maximum allowed velocity
searchProblem.w     = [0.5 0.5];
searchProblem.succ  = @neighbors8F;
searchProblem.h     = @hEuclideanMaxA;
%% Search for the initial solution
searchInfo = [];
searchInfo = AStarGraph(searchProblem, searchInfo);
%searchInfo = ARAStar(searchProblem, searchInfo);
%% useful plots
figure
plot(searchInfo.path(:,3),searchInfo.path(:,4));
axis tight;
%% Animation (replanning is done whenever a change happens)
simStep = 0.01;
simTime = 100;
t = 0;
figure( 'units','normalized','outerposition',[0 0 1 1]);
ax = gca;
plotD(ax, searchProblem, searchInfo,t);
while t+simStep<simTime && t+simStep<searchInfo.path(end,3);
    t=t+simStep;
    searchProblem = updateAgentsRepeat(searchProblem, 1:4, simStep);
    searchProblem = updateAgentsBounce(searchProblem, 5:6, simStep);
    searchProblem.start = stateAtTime(searchInfo.path, t);
    searchInfo = AStarGraph(searchProblem, searchInfo);
    plotD(ax, searchProblem, searchInfo,t);
    pause(0.01)
end



