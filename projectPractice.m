clear all
%Define parameters for analysis
speed_limit = 45;
p = 0.1; % probability of car entering road after timestep dt
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
%driver_data(6,:) contains queue position (0 is on road)


for t = 1:end_time
    stoplights = SL_update(stoplights,t,dt);
    
    % run through queues generating new cars
    for entry = 1:size(stoplights,2)
        %   decide if new car
        prob = stoplights(6, entry);
        new_car = floor((prob/num_lanes)*ceil(1/(prob/num_lanes)*rand(1)));
    
        if new_car
            %   increment size queue
            queue_size = stoplights(7,entry) + 1;
            stoplights(7,entry) = queue_size;
            light_dist = stoplights(1,entry);
            
            %   make new car in car vec with light's dist, queue pos, and lane 0 
            ind = ind+1; 
            car_vec = [car_vec ind];
            car_data = [ 0; light_dist; t; t; light_dist; queue_size ]; 
            driver_data = [driver_data car_data];
            
        end
    end
    
    m = num_lanes;
    n = street_length;
    G1 = zeros(m,n);
    queue_exit = zeros(size(stoplights,2), 1);
    
    for kk = car_vec
        ii = driver_data(1,kk);
        jj = driver_data(2,kk);
        if jj < n
            % if approaching a light
            if any(jj==stoplights(1,:))
              ss = find(jj==stoplights(1,:));
              
              % driver on road
              if ii > 0
                  assert(driver_data(6, kk) == 0);
                  
                  % red light
                  if stoplights(5,ss)==1
                      assert(G1(ii,jj) == 0);
                      G1(ii,jj) = 1;
                      driver_data(2,kk) = jj;
                      driver_data(4,kk) = t;
                  % green light
                  elseif street(ii, jj+1) == 0
                      % no switch lanes at light?
                      assert(G1(ii,jj+1) == 0);
                      G1(ii,jj+1) = 1;
                      driver_data(2,kk) = jj+1;
                      driver_data(4,kk) = t;
                  end
              % driver in queue
              elseif ii == 0
                  assert(driver_data(6, kk) > 0);
                  
                  % driver at front of queue
                  if driver_data(6, kk) == 1
                      % initial entry point
                      if jj == 1
                          assert(stoplights(5,1) == 0);
                          
                          for lane = 1:m
                              if street(lane, 1) == 0
                                  assert(G1(lane,1) == 0);
                                  G1(lane, 1) = 1;
                                  driver_data(1,kk) = lane;
                                  driver_data(4,kk) = t;
                                  queue_exit(1) = 1;
                                  break;
                              end
                          end
                      % right turn onto intersection
                      else
                          ss = find(jj==stoplights(1,:));
                          
                          % green light
                          if stoplights(5, ss) == 0
                              % right lane empty
                              if street(m, jj) == 0
                                  assert(G1(m,jj) == 0);
                                  G1(m, jj) = 1;
                                  driver_data(1,kk) = m;
                                  driver_data(4,kk) = t;
                                  queue_exit(ss) = 1;                                  
                              end
                          % red light
                          else
                              % right lane and preceding spot empty
                              if street(m, jj) == 0 && street(m, jj-1) == 0
                                  assert(G1(m,jj) == 0);
                                  G1(m, jj) = 1;
                                  driver_data(1,kk) = m;
                                  driver_data(4,kk) = t;
                                  queue_exit(ss) = 1;                                  
                              end                              
                          end
                      end
                  end
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
    
    % move up each car in queue if leader left
    for kk = car_vec
        jj = driver_data(2, kk);
        queued = (driver_data(6 ,kk) > 0);
        if sum(queue_exit) > 1
            disp(queue_exit);
        end
        
        if any(jj==stoplights(1,:)) && queued
            ss = find(jj==stoplights(1,:));
            if queue_exit(ss)
                driver_data(6, kk) = driver_data(6, kk) - 1;
            end
        end
    end
    
    for light = 1:size(stoplights, 2)
        if queue_exit(light)
            stoplights(7, light) = stoplights(light) - 1;
        end
    end
    
%     for ii = 1:m %additional entry of new cars into street
%         if street(ii,1) == 0
%             G1(ii,1) = floor((p/num_lanes)*ceil(1/(p/num_lanes)*rand(1)));
%             if G1(ii,1) == 1
%                 ind = ind+1;
%                 car_vec = [car_vec ind];
%                 car_data = [ii;1;t;t;1];
%                 driver_data = [driver_data car_data];
%             end
%         end
%     end
    street = G1;
end
disp(size(driver_data));
%collect travel time data
for jj = 1:size(driver_data, 2)
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