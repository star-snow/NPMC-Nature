function map = generate_map(map, map_size, nb_blocks, bulding_width)

    % Forest-like map parameters
    map.width = map_size; % the forest is of size (width)x(width)
    map.bl_corner_north = +map.width/5;
    map.bl_corner_east = -map.width/2;
    map.max_height = map.width; % maximum height of trees
    map.nb_blocks = nb_blocks; % the number of blocks per row
    
    map.street_width_perc = 0.45; % percentage of block that is empty
    map.building_width = bulding_width;
    map.street_width = map.width/map.nb_blocks*map.street_width_perc;

    map.building_shape = ('cylinder');

    % Create buildings parameters
    map = create_random_buildings(map);
    map.buildings_north = map.buildings_north + map.bl_corner_north;
    map.buildings_east = map.buildings_east + map.bl_corner_east;
    
end