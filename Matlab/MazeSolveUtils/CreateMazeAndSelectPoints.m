function [mazeMap, occupancyMatrixPlot, startGrid, goalGrid, figureMaze, axesMaze] = CreateMazeAndSelectPoints(mazeProps, choice)
arguments
    mazeProps 
    choice      (1,:) string {mustBeMember(choice, ["manual", "fixed"])} = "manual"
end
    global RANDOM_SEED

    mazeMap = CreateMaze(mazeProps);
    occupancyMatrixPlot = occupancyMatrix(mazeMap);
    pacmanColorMap = [0 0 0; 0.1294 0.1294 1];
    [figureMaze, axesMaze] = PlotMaze(occupancyMatrixPlot, pacmanColorMap, ...
        'Select Start', 'Select START point');

    switch choice
        case "fixed"
            switch RANDOM_SEED
                case 4
                    startRow = 20; startCol = 40;
                case 15
                    startRow = 65; startCol = 5;
                case 20
                    startRow = 40; startCol = 40;
                case 25
                    startRow = 29; startCol = 61;
            end
            goalRow = 10; goalCol = 65;
            assert(occupancyMatrixPlot(startRow, startCol) == 0, 'Selected START point is a wall.');
            assert(occupancyMatrixPlot(goalRow, goalCol) == 0, 'Selected GOAL point is a wall.');
        case "manual"
            [rowCount, colCount] = size(occupancyMatrixPlot);

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
    end

    startGrid = [startRow, startCol];
    goalGrid  = [goalRow, goalCol];

end
