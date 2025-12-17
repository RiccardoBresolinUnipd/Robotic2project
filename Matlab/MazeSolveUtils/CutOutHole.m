function mazeMap = CutOutHole(mazeMap)
    %% Set empty space at the bottom-right of the maze

    hole = round(mazeMap.GridSize / 3);
    topleft = fliplr(round((mazeMap.GridSize - hole) /(mazeMap.Resolution)));
    
    matrix = zeros(hole);
    % --- walls ---
    matrix(1:end,1) = 1;            % top
    matrix(end, 1:end) = 1;         % left
    matrix(1:end,end-2:end) = 1;    % right
    matrix(1:3, 1:end) = 1;         % bottom
    % --- door ---
    matrix(end, 4:7) = 0;

    % topleft -> [x, y]

    setOccupancy(mazeMap, topleft, matrix, "world");
end
