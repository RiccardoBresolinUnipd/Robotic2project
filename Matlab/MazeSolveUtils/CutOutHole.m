function mazeMap = CutOutHole(mazeMap)
    %% Set empty space at center of the maze

    hole = round(mazeMap.GridSize / 5);
    topleft = fliplr(round((mazeMap.GridSize - hole ) /(2*mazeMap.Resolution)));
    matrix = zeros(hole);
    matrix(1:end,1) = 1;
    matrix(1:end,end) = 1;
    matrix(1, 1:end) = 1;
    matrix(end, 1:end) = 1;
    matrix(10:15, end) = 0;
    setOccupancy(mazeMap, topleft, matrix);
end
