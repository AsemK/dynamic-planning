function block = state2block(state, cellL, speedR)
%state2block converts a state to a block given the cell length and speed range.
block = [floor(state(1:2)./cellL), floor(state(4)/speedR)] + 1;
end

