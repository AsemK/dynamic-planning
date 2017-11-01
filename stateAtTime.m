function state = stateAtTime(path, t)
%stateAtTime takes a path and returns the position of an object following
%this path at the input time.

%get the index of path node with maximum time less than or equal t.
s = find(path(:,3)<=t, 1, 'last');
%find the direction of movement
angle = atan2d((path(s+1,2)-path(s,2)),(path(s+1,1)-path(s,1)));
%time since last state;
dt = t-path(s,3);
%velocity now;
v = path(s+1,4);
%new position
distance = v*dt;
pos = path(s,1:2)+distance.*[cosd(angle) sind(angle)];
state = [pos t v];
end

