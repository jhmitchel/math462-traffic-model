clear;

% set initial values
max_cars = 20000; % for allocation
road_length = 3.4 * 63360; % inches
car_length = 176; % inches
num_spots = floor(road_length / car_length); % length of road in terms of cars
speed_limit = 40; % mph
time_step = (car_length / 63360) * (3600 / speed_limit); % seconds (car lengthing in miles / miles per second)
tot_time = 30 * 60; % seconds (minutes * minutes/second)
num_lights =  10; % NOT ACCURATE - for allocation

% Washtenaw - rows are distance, columns are lanes
% 0s are empty spots, greater than 0s is car index.
washtenaw = zeros(num_spots, 2);

% create table of car data
% each row is a different car IDed by their index
% cols are (more can be added)
%   (1) entrance spot number
%   (2) exit spot number
%   (3) cur spot number
%   (4) time on Watenaw
%   (5) time waiting to get on
car_table = zeros(max_cars, 3); 

% 0s signal no light, greater than 0s is light index.
traffic_map = zeros(num_spots, 1);

% create table of light data
% each row is a different light IDed by their index
% cols are (more can be added)
%   (1) 0 for green down Washtenaw and red for turning,
%       1 for red down Washtenaw and green for turning
light_table = zeros(num_lights, 1);

% TODO: create lights at approx real intersections

% TODO: create queues for each light 

% start simulation
for t = 0:time_step:max_time
    % allocate next road 
    next_washtenaw = zeros(size(washtenaw));
    
    % TODO: run move procedure on every spot
    % ex:
%     for cur_spot = 1:num_spots
%         % update next spot lane 1
%         [next_spot, next_lane] = move(washtenaw, cur_spot, 1);
%         assert(next_washtenaw(next_spot, next_lane) == 0);
%         next_washtenaw(next_spot, next_lane) = washtenaw(cur_spot, 1);
%         
%         % update next spot lane 2
%         [next_spot, next_lane] = move(washtenaw, cur_spot, 2);
%         assert(next_washtenaw(next_spot, next_lane) == 0);
%         next_washtenaw(next_spot, next_lane) = washtenaw(cur_spot, 2);
%     end
    
    % TODO: check intersection turns

end