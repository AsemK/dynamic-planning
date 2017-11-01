function searchInfo = AStar(searchProblem, searchInfo)
%AStar performs an A* search

%Anonymouse functions for the heuristic and successors functions (to make
%them take only one input and make the code cleaner).
%The heuristic is inflated because otherwise the solution takes very long
%time.
h    = @(state) 1.1*searchProblem.h(searchProblem, state, searchProblem.goal); 
succ = @(state) searchProblem.succ(searchProblem, state);
%if no searchProblem or no change after the last planning: do nothing
if isempty(searchProblem) || (~isChanged(searchProblem) && ~isempty(searchInfo))
    searchInfo.time = 0;
%else (if new search problem or a change is detected): plan from scratch
else
    tic %start measuring time
    %Initializing
    success = false;
    cellL   = 0.5*searchProblem.arcL; %cell length for descretizing
    start   = searchProblem.start;
    %a row in the open queue has the form:
    %[key state(4 elements) positionCell(2 elements) g-value
    %number-of-expands backPointer(4 elements)].
    open    = [h(start) start pos2cell(start(1:2), cellL) 0 0 0 0 0 0];
    goal    = zeros(0, 4);
    goalInd = pos2cell(searchProblem.goal, cellL);
    %Searching
    while ~isempty(open(open(:,1)<Inf)) && ~success
        %indices of all states with minimum key. allows different ties
        %breaking strategies if desired.
        i = find(open(:,1)==min(open(:,1))); 
        s = open(i(1),2:5);                        %the current expanded state
        open(i(1),1) = Inf;                        %pop state
        open(i(1), end-4) = open(i(1), end-4) + 1; %mark as expanded
        g = open(i(1), end-5);
        %iterate over successors
        suc = succ(s);
        for x = 1:size(suc,1)
            ind = pos2cell(suc(x,2:3),cellL);
            %the round function is used below to limit the number of states
            %with almost equal speeds in one cell.
            [inOpen, loc] = ismember([ind round(suc(x,5),2)], [open(:,6:7) round(open(:,5),2)],'rows');
            %initialize g-value
            if inOpen
                gs = open(loc, end-5);
            else
                gs = Inf;
            end
            %update if a better path through this state is found
            if gs > (g + suc(x,1))
                gs = (g + suc(x,1));
                key = gs + h(suc(x,2:end));
                %if the state was in open, replace it
                if inOpen
                    open(loc, :) = [key suc(x,2:end) ind gs 0 s];
                %else insert it.
                else
                    open = [[key suc(x,2:end) ind gs 0 s]; open];
                    %discover the goal while it is inserted (not expanded).
                    if isequal(ind, goalInd)
                        success = true;
                        goal = s;
                    end
                end
            end
        end
    end
    %putting all the information in the searchInfo Struct
    time = toc; %stop measuring time
    searchInfo.success = success;
    searchInfo.goal    = goal;
    searchInfo.open    = open;
    searchInfo.time    = time;
    %extracting the path
    if success
        searchInfo = tracePath(searchProblem, searchInfo);
    end
end
end