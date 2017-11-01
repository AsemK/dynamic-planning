function key = cell2key(cell)
%cell2key returns a key (for dictionaries) for a cell assuming that the
%maximum number of cells horizontally and vertically is 255.
key = cell*[256 1]';
end

