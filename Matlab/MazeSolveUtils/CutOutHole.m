function mazeMap = CutOutHole(mazeMap)
    %% Set empty space at center of the maze

    hole = mazeMap.GridSize / 5;
    topleft = fliplr(round((mazeMap.GridSize - hole ) /(2*mazeMap.Resolution)));
    setOccupancy(mazeMap, topleft, zeros(hole));
end
