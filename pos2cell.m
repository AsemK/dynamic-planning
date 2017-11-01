function cell = pos2cell(pos, cellL)
%pos2cell converts a position to a cell location given the cell length.
cell = floor(pos./cellL)+1;
end

