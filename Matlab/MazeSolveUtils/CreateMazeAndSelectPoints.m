function [mazeMap, occupancyMatrixPlot, startGrid, goalGrid, figureMaze, axesMaze] = CreateMazeAndSelectPoints(mazeProps)

    mazeMap = CreateMaze(mazeProps);

    occupancyMatrixPlot = occupancyMatrix(mazeMap);
    [rowCount, colCount] = size(occupancyMatrixPlot);

    pacmanColorMap = [0 0 0; 0.1294 0.1294 1];
    [figureMaze, axesMaze] = PlotMaze(occupancyMatrixPlot, pacmanColorMap, ...
        'Select Start', 'Select START point');

    disp('Select START point.');
    [startX, startY] = ginput(1);

    startRow = max(1, min(rowCount, round(startY)));
    startCol = max(1, min(colCount, round(startX)));

    if occupancyMatrixPlot(startRow, startCol) == 1
        error('Selected START point is a wall.');
    end

    plot(axesMaze, startCol, startRow, 'go', 'MarkerSize', 10, 'LineWidth', 2);

    title(axesMaze, 'Select GOAL point');
    disp('Select GOAL point...');

    [goalX, goalY] = ginput(1);

    goalRow = max(1, min(rowCount, round(goalY)));
    goalCol = max(1, min(colCount, round(goalX)));

    if occupancyMatrixPlot(goalRow, goalCol) == 1
        error('Selected GOAL point is a wall.');
    end

    plot(axesMaze, goalCol, goalRow, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

    startGrid = [startRow, startCol];
    goalGrid  = [goalRow, goalCol];

end
