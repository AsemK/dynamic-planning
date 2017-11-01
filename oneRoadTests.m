close all
clear functions %necessary for correct operation of plotD
%% Define the searchProblem
carsR = 1;   %cars agents radius
pedsR = 0.5; %pedestrian agents radius;
searchProblem = emptySearchProblem;
searchProblem.grid = oneRoadGrid;
searchProblem = addAgent(searchProblem, 4,  3, carsR, 15, 0);
searchProblem = addAgent(searchProblem, 4,  5, carsR, 15, 0);
searchProblem = addAgent(searchProblem, 4,  7, carsR, 15, 0);
searchProblem = addAgent(searchProblem, 4,  9, carsR, 15, 0);

%searchProblem = addAgent(searchProblem, 10, 3, carsR, 35, 0);
%searchProblem = addAgent(searchProblem, 5,  5, carsR, 25, 0);
%searchProblem = addAgent(searchProblem, 12, 5, carsR, 8, 0);
%searchProblem = addAgent(searchProblem, 4,  7, carsR, 15, 0);
%searchProblem = addAgent(searchProblem, 7,  7, carsR, 40, 0);
%searchProblem = addAgent(searchProblem, 6,  9, carsR, 20, 0);
%searchProblem = addAgent(searchProblem, 10, 9, carsR, 30, 0);
searchProblem = addAgent(searchProblem, 15, searchProblem.grid.length, pedsR, 2, 270);
searchProblem = addAgent(searchProblem, 30, 0, pedsR, 4, 90);
searchProblem = addAgent(searchProblem, 50, searchProblem.grid.length, pedsR, 5, 270);
searchProblem = addAgent(searchProblem, 75, 0, pedsR, 3, 90);
searchProblem = addAgent(searchProblem, 80, searchProblem.grid.length, pedsR, 4, 270);
searchProblem = addAgent(searchProblem, 98, 0, pedsR, 6, 90);

searchProblem.start = [0.5, 2.5, 0, 15.5]; %state (x, y, time, speed)
searchProblem.goal  = [100.5, 2.5];      %position (x, y)
searchProblem.carR   = 0.5;              %car radius
searchProblem.arcL   = 1;              %length of one movement
searchProblem.cellL  = 0.5;             %position discretization
searchProblem.speedR = 0.001;             %speed discretization
searchProblem.maxV   = 25;               %maximum allowed velocity
searchProblem.w      = [1 0];  %wights of [distance time] in the cost and heuristic      
searchProblem.succ  = @succRightCSV;
searchProblem.h     = @hEuclideanMaxV;
%% Search for the initial solution
searchInfo = [];
%use one of these:
searchInfo = AStarGraph(searchProblem, searchInfo)
%searchInfo = ARAStar(searchProblem, searchInfo)
%% useful plots
figure
stairs(searchInfo.path(1:end-1,3),searchInfo.path(2:end,4),'b','LineWidth',1); %plot speed vs. time
axis tight
xlabel('time (s)')
ylabel('speed (m/s)')
set(gcf,'color','w');
%% Animation (replanning is done whenever a change happens)
simStep = 0.01;
simTime = 100;
t = 0;
figure( 'units','normalized','outerposition',[0 0 1 1]);
ax = gca;
ax.Color = 'none';
plotD(ax, searchProblem, searchInfo,t);
%M = [];            %uncomment if you want to creat a video or show movie.
%M = [M, getframe]; %uncomment if you want to creat a video or show movie.
while t+simStep<simTime && t+simStep<searchInfo.path(end,3);
    t=t+simStep;
    searchProblem = updateAgentsRepeat(searchProblem, 1:4, simStep);
    searchProblem = updateAgentsBounce(searchProblem, 5:10, simStep);
    searchProblem.start = stateAtTime(searchInfo.path, t);
    searchInfo = AStarGraph(searchProblem, searchInfo);
    plotD(ax, searchProblem, searchInfo,t);
    %M = [M, getframe]; %uncomment if you want to creat a video or show movie.
    pause(0.01)
end
%% Create a video
% v = VideoWriter('newfile.mp4','MPEG-4');
% v.FrameRate = floor(0.4/simStep);
% open(v);
% writeVideo(v, M);
% close(v);
%% show movie
% figure( 'units','normalized','outerposition',[0 0 1 1]);
% movie(M,1, 10)



