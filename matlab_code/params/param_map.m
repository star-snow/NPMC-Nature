% Forest map parameters
map.ACTIVE_ENVIRONMENT = true;

map.width = 8; % the forest is of size (width)x(width)
map.bl_corner_north = 2;
map.bl_corner_east = -3;
map.max_height = 8; % maximum height of trees
map.nb_blocks = 5; % the number of blocks per row
map.street_width_perc = 0.45; % percentage of block that is empty

map.building_width =  1.1;
map.street_width = 0.9;

map.building_shape = ('cylinder');

% Create buildings parameters
% map = create_shifted_buildings(map);
% map.buildings_north = map.buildings_north + 2;

map.buildings_north = [
    1
    1
    1
    3
    3
    5
    5
    5] + map.bl_corner_north;
    
   
map.buildings_east = [
   1
   3
   5
   2
   4
   1
   3
   5] + map.bl_corner_east;
map.buildings_heights = [
    10
    10
    10
    10
    10
    10
    10
    10];
