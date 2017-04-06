clear all
%Define parameters for analysis
speed_limit = 45;
p = 0.4; % probability of car entering road after timestep dt
num_mins = 30;
num_lanes = 2;

%Define vector of cars on street
car_vec = [];
ind = 0; % will number each car entering the road for reference
car_length = 15; % ft. (average for car)
dt = (speed_limit*5280/3600*1/car_length)^-1; % duration (sec) of each time step
end_time = ceil(1/dt*60*num_mins); %how many time steps simulation will run for
speed_vec = []; %vector of travel speeds for cars

%Define Grid for Street
street_length = 100;
street = zeros(num_lanes,street_length);

%Define Locations of Stoplights

stoplights = [1 33 67; 0 0 0.5; 1 2 2; 0 1 1; 0 0 0; p p/2 p/2; 0 0 0];
%stoplights(1,:) contains location of each SL
%stoplights(2,:) contains offset time of each SL
%stoplights(3,:) contains length of green light in mins
%stoplights(4,:) contains length of red light in mins
%stoplights(5,:) contains state of light (0 for green, 1 for red)
%stoplights(6,:) contains new car parameter
%stoplights(7,:) contains cars in queue

%Define Matrix for Driver Data
driver_data = [];
%driver_data(1,:) contains current lane of each car
%driver_data(2,:) contains distance along lane of each car
%driver_data(3,:) contains entry time of each car
%driver_data(4,:) contains exit time of each car
%driver_data(5,:) contains entry distance of each car
%driver_data(6,:) contais queue position (0 is on road)


for t = 1:end_time
    stoplights = SL_update(stoplights,t,dt);
    
    % TODO: run through queues generating new cars
    % for stoplights
    %   decide if new car
    %   increment size queue
    %   make new car in car vec with queue position equal to queue size
    %   set car distance to light distance
    %   set lane to 0 to signal queue
    
    m = num_lanes;
    n = street_length;
    G1 = zeros(m,n);
    
    for kk = car_vec
        ii = driver_data(1,kk);
        jj = driver_data(2,kk);
        if jj < n
            if any(jj==stoplights(1,:))
              ss = find(jj==stoplights(1,:));
              if driver_data(1, kk) > 0
                  if stoplights(5,ss)==1
                      G1(ii,jj) = 1;
                      driver_data(2,kk) = jj;
                      driver_data(4,kk) = t;
                  else
                      G1(ii,jj+1) = 1;
                      driver_data(2,kk) = jj+1;
                      driver_data(4,kk) = t;
                  end
              elseif driver_data(1,kk) == 0
                  % TODO: logic for turns 
                  % check light
                  % if green 
                  %   if entry point
                  %     entrance logic
                  %   else
                  %     turn logic
                  % else
                  %   turn on red logic
              end
            elseif street(ii,jj+1) == 0
                G1(ii,jj+1) = 1;
                driver_data(2,kk) = jj+1;
                driver_data(4,kk) = t;
            else
                if m > 1
                    if ii == 1 %if in left lane
                        if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0
                            G1(ii+1,jj+1) = 1;
                            driver_data(1,kk) = ii+1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        end
                    elseif ii == m %if in right lane
                        if street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0
                            G1(ii-1,jj+1) = 1;
                            driver_data(1,kk) = ii-1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        end
                    else   %if lanes on either side
                        if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0
                            G1(ii+1,jj+1) = 1;
                            driver_data(1,kk) = ii+1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        elseif street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0
                            G1(ii-1,jj+1) = 1;
                            driver_data(1,kk) = ii-1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            G1(ii,jj) = 1;
                        end
                    end
                else
                    G1(ii,jj) = 1;
                    driver_data(4,kk) = t;
                end    
            end
        elseif jj == n
            G1(ii,jj) = 0;
            driver_data(2,kk) = jj+1;
            driver_data(4,kk) = t;
        end
    end
    for ii = 1:m %additional entry of new cars into street
        if street(ii,1) == 0
            G1(ii,1) = floor((p/num_lanes)*ceil(1/(p/num_lanes)*rand(1)));
            if G1(ii,1) == 1
                ind = ind+1;
                car_vec = [car_vec ind];
                car_data = [ii;1;t;t;1];
                driver_data = [driver_data car_data];
            end
        end
    end
    street = G1;
end

%collect travel time data
for jj = 1:length(driver_data)
    drive_time = (driver_data(4,jj) - driver_data(3,jj))*dt;
    drive_dist = (driver_data(2,jj) - driver_data(5,jj))*15;
    ave_speed = drive_dist/drive_time; %ft/sec
    ave_speed = ave_speed * 3600/5280; %convert to mph
    speed_vec = [speed_vec ave_speed];
end

%generate figure
figure
hist(speed_vec);
title('Average Speed of Cars');
xlabel('Average Speed (mph)');
ylabel('Number of Cars');

%{
Still to be done:
1. Find way to generate stoplight start/stop patterns
2. Find way to get car to respond to stoplight state
3. Find way to generate new cars at stoplight locations
%}