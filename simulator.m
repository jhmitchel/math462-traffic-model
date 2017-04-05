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
max_queue = 25;

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
%   (2) index of queue turning right onto Washtenaw at intersection
%   (3) index of queue turning left onto Washtenaw at intersection
light_table = zeros(num_lights, 3);

% TODO: create lights at approx real intersections

% create two queues for each light - 1 for left turns and 1 for right turns
% cells in a row are the car index, queue order left to right
queue_table = zeros(2 * num_lights, max_queue);


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
    for cur_spot = 1:num_spots
        light_index = traffic_map(cur_spot);
        
        if light_index > 0
            green_light = light_table(light_index, 1);
            left_queue = light_table(light_index, 2);
            right_queue = light_table(light_index, 3);
            
            % turn logic for green light
            if green_light
                % left turn queue
                left_car = queue_table(left_queue, 1);
                if left_car > 0
                    spot_empty = (washtenaw(cur_spot, 1) == 0);
                    if spot_empty
                        assert(next_washtenaw(cur_spot, 2) == 0);
                        next_washtenaw(cur_spot, 1) = left_car;
                        % TODO: shift queue
                    end
                end
                
                % right turn queue
                right_car = queue_table(right_queue, 2);
                if right_car > 0
                    spot_empty = (washtenaw(cur_spot, 2) == 0);
                    if spot_empty
                        assert(next_washtenaw(cur_spot, 2) == 0);
                        next_washtenaw(cur_spot, 2) = right_car;
                        % TODO: shift queue
                    end
                end
            % turn logic for red light    
            else
                % right turn queue
                right_car = queue_table(right_queue, 2);
                if right_car > 0
                    spot_empty = (washtenaw(cur_spot, 2) == 0 && washtenaw(cur_spot-1, 2) == 0);
                    if spot_empty
                        assert(next_washtenaw(cur_spot, 2) == 0);
                        next_washtenaw(cur_spot, 2) = right_car;
                        % TODO: shift queue
                    end
                end
            end
        end
        
        % TODO: randomly generate new cars
    end

end