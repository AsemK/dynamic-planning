function searchInfo = tracePath(searchProblem, searchInfo)
%tracePath takes the searchInfo struct with open queue and goal and
%returns the path as an array of nodes.
if searchInfo.success
    foundPath = [searchInfo.goal];
    while ~isequal(foundPath(1,:), searchProblem.start)
        [~, i] = ismember(foundPath(1,:), searchInfo.open(:,2:5),'rows');
        foundPath = [searchInfo.open(i, end-3:end); foundPath];
    end
else
    foundPath = zeros(0,4);
end
searchInfo.path = foundPath;
end