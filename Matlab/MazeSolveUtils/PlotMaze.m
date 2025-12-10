function [figHandle, axHandle] = PlotMaze(occMatrix, colorMap, figName, titleText)

    % Basic maze visualization
    [rowCount, colCount] = size(occMatrix);

    figHandle = figure('Name', figName);
    axHandle = axes('Parent', figHandle);

    imagesc(axHandle, occMatrix);
    axis(axHandle, 'equal', 'tight');
    set(axHandle, 'YDir', 'normal');

    colormap(axHandle, colorMap);
    set(axHandle, 'Color', 'k');

    tickColor = [1 1 1 0.5];

    set(axHandle, ...
        'XTick', 1:10:colCount, ...
        'YTick', 1:10:rowCount, ...
        'XTickLabel', 1:10:colCount, ...
        'YTickLabel', 1:10:rowCount, ...
        'XColor', tickColor, ...
        'YColor', tickColor);

    title(axHandle, titleText);
    xlabel(axHandle, 'Column');
    ylabel(axHandle, 'Row');
    hold(axHandle, 'on');
end
