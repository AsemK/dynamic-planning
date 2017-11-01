function searchInfo = ARAStar(searchProblem, searchInfo)
%ARAStar performs an ARA* (Anytime Replanning A*) search

%Anonymouse functions for the heuristic and successors functions (to make
%them take only one input and make the code cleaner).
h    = @(state) searchProblem.h(searchProblem, state, searchProblem.goal);
succ = @(state) searchProblem.succ(searchProblem, state);
Eps0 = 1.5;           %initial inflation factor
dEps = 0.05;         %inflation factor step difference.
allowedTime = 1; %allowed time for the planner in seconds.
%% Nested Function
%nested function that inserts expanded nodes from open to closed then
%removes them for open.
function insertClosed
    closedNodes = open(open(:,end-4)==1,:);
    for i = 1:size(closedNodes, 1)
        [inClosed, loc] = ismember(closedNodes(i,2:5), closed(:,2:5),'rows');
        if inClosed
            closed(loc,[1:end-5, end-3:end]) = closedNodes(i,[1:end-5, end-3:end]);
            closed(loc,end-4) = closed(loc,end-4) + 1;
        else
            closed = [closed; closedNodes(i,:)];
        end
    end
    open(open(:,end-4)==1,:) = [];
    open(:,end-4) = zeros(size(open,1),1);
end
%nested function to save information in the searchInfo.struct
function saveInfo
    searchInfo.success = success;
    searchInfo.goal    = goal;
    searchInfo.open    = open;
    searchInfo.time    = time;
    searchInfo.Eps     = Eps;
end
%nested function that performs the search
function improvePath
    while ~isempty(open(open(:,1)<Inf)) && (time < allowedTime)
        %indices of all states with minimum keys. allows different ties
        %breaking strategies if desired.
        i = find(open(:,1)==min(open(:,1)));
        s = open(i(1),2:5);              %the current expanded state
        g = open(i(1), end-5);           %the g-velure of the state 
        %if everything has less key than the known cost of the goal: save
        %information and end the the search
        if (goalG <= open(i(1), 1))
            insertClosed;
            saveInfo;
            break;
        end
        open(i(1),1) = Inf;                        %pop state
        open(i(1), end-4) = open(i(1), end-4) + 1; %mark as expanded
        % iterate over successors
        suc = succ(s);
        for x = 1:size(suc,1)
            block = state2block(suc(x,2:5), cellL, speedR);
            %the round function is used below to limit the number of states
            %with almost equal speeds in one cell.
            [inOpen, loc] = ismember(block, open(:,6:8),'rows');
            %initialize g-value
            if inOpen
                gs = open(loc, end-5);
            else
                gs = Inf;
            end
            %update if a better path through this state is found
            if gs > (g + suc(x,1))
                gs = (g + suc(x,1));
                hs = h(suc(x,2:end));
                key = gs + Eps*hs;
                %if a better goal is found: change the goal
                if isequal(block(1:2), goalInd) && gs < goalG
                    success = true;
                    goal = suc(x,2:5);
                    goalG = gs;
                end
                %if the state was in open, replace it
                if inOpen
                    if open(loc, end-4) > 0
                        %if it was expanded before, mark it as inconsistent
                        open(i(1), end-4) = 2;
                        continue;
                    end
                    open(loc,:) = [key suc(x,2:end) block hs gs 0 s];
                %else insert it
                else
                    open = [[key suc(x,2:end) block hs gs 0 s]; open];
                end
            end
        end
        time = toc;
    end
end
%% Main code
%if no searchProblem or no change after the last planning: do nothing
if isempty(searchProblem) || (~isChanged(searchProblem) && ~isempty(searchInfo))
    searchInfo.time = 0;
%else (if new search problem or a change is detected): plan from scratch
else
    tic %start measuring time
    %Initializing
    success  = false;
    searchInfo.success = success;
    cellL = searchProblem.cellL;
    speedR = searchProblem.speedR;
    Eps = Eps0;
    start = searchProblem.start;
    %a row in the open queue has the form:
    %[key state(4 elements) positionCell(2 elements) h-value g-value
    %number-of-expands backPointer(4 elements)].
    open = [Eps*h(start) start state2block(start, cellL, speedR) h(start) 0 0 0 0 0 0];
    closed = zeros(0,size(open,2)); %has the same form as open
    goal = zeros(0, 4);
    goalInd = pos2cell(searchProblem.goal, cellL);
    goalG   = Inf;
    time = toc;
    %searching
    improvePath;
    %improve path if time is available
    while success && (Eps >= 1.05) && (time < allowedTime)
        Eps = Eps - dEps;
        open(:,1) = open(:,end-5) + Eps*open(:,end-6);
        time = toc;
        improvePath;
    end
    %Putting all the information in the searchInfo Struct 
    if searchInfo.success
        searchInfo.open = [searchInfo.open; closed];
        searchInfo = tracePath(searchProblem, searchInfo);
    end
end
end