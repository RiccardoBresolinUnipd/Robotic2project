function mazeMap = CreateMaze(mazeProps)

    passageWidth    = mazeProps.passageWidth;
    wallThickness   = mazeProps.wallThickness;
    mapSize         = mazeProps.mapSize;
    mapResolution   = mazeProps.mapResolution;

    mazeMap = mapMaze(passageWidth, wallThickness, ...
        'MapSize', mapSize, 'MapResolution', mapResolution);

end
