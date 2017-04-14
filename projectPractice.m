clear all
tic
%Define parameters for analysis
speed_limit = 45;
p = 0.1; % probability of car entering road after timestep dt
num_mins = 240;
num_lanes = 2;

spots_must_switch = 40;
spots_try_switch = 150;

prob_straight = 5/8;
prob_left = 1/8;
prob_right = 2/8;

%Define vector of cars on street
car_vec = [];
ind = 0; % will number each car entering the road for reference
car_length = 15; % ft. (average for car)
dt = (speed_limit*5280/3600*1/car_length)^-1; % duration (sec) of each time step
end_time = ceil(1/dt*60*num_mins); %how many time steps simulation will run for
speed_vec = []; %vector of travel speeds for cars

%Define Grid for Street
street_length = 1250;
street = zeros(num_lanes,street_length);

%Define Locations of Stoplights
stoplights = [0 70 140 175 210 315 490 525 735 770 910 980 1015 1040; 
              0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
              1 2 2 2 2 2 2 2 2 2 2 2 2 2; 
              0 1 1 1 1 1 1 1 1 1 1 1 1 1; 
              0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
              0.1 0.0005 0.0005 0.0005 0.008 0.025 0.0005 0.0005 0.125 0.0005 0.0005 0.016 0.0005 0.0005; 
              0 0 0 0 0 0 0 0 0 0 0 0 0 0];
long_queue = zeros(1,length(stoplights));
%stoplights(1,:) contains location of each SL
%stoplights(2,:) contains offset time of each SL
%stoplights(3,:) contains length of green light in mins
%stoplights(4,:) contains length of red light in mins
%stoplights(5,:) contains state of light (0 for green, 1 for red)
%stoplights(6,:) contains new car parameter
%stoplights(7,:) contains cars in queue
%stoplights(8,:) contains time steps unchanged

%Define Matrix for Driver Data
driver_data = [];
%driver_data(1,:) contains current lane of each car
%driver_data(2,:) contains distance along lane of each car
%driver_data(3,:) contains entry time of each car
%driver_data(4,:) contains exit time of each car
%driver_data(5,:) contains entry distance of each car
%driver_data(6,:) contains queue position (0 is on road)
%driver_data(7,:) contains exit destination (0-Washtenaw, 1-Arborland, num_lanes-US23)

lambda_dist = normpdf(((1:end_time) - (end_time/2)) / end_time, 0, 1/3.5);


for t = 1:end_time
    stoplights = SL_update(stoplights,t,dt);
    %stoplights = SL_update_sensor(stoplights, street);
    
    % run through queues generating new cars
    for entry = 1:size(stoplights,2)
        %   decide if new car
        light_popularity = stoplights(6, entry);
        num_cars = poissrnd(light_popularity * lambda_dist(t));
    
        while num_cars > 0
            %   increment size queue
            queue_size = stoplights(7,entry) + 1;
            stoplights(7,entry) = queue_size;
            light_dist = stoplights(1,entry);
            
            % exit location
            exit_loc = 0;
            seed = rand();
            if seed < prob_straight % 5/8
                exit_loc = 0;
            elseif seed < prob_straight + prob_left % 1/8
                exit_loc = 1;
            else
                exit_loc = num_lanes; % 2/8
            end
            
            %   make new car in car vec with light's dist, queue pos, and lane 0 
            ind = ind+1; 
            car_vec = [car_vec ind];
            car_data = [ 0; light_dist; t; t; light_dist; queue_size; exit_loc ]; 
            driver_data = [driver_data car_data];
            
            num_cars = num_cars - 1;            
        end
    end
    
    m = num_lanes;
    n = street_length;
    G1 = zeros(m,n);
    queue_exit = zeros(size(stoplights,2), 1);
    
    
    for kk = car_vec
        ii = driver_data(1,kk);
        jj = driver_data(2,kk);
        dest = driver_data(7,kk);
        
        
        if jj < n
            if driver_data(6,kk) == 0 
                % assert(street(ii,jj) == 1);
            end
        
            % if approaching a light
            if any(jj==stoplights(1,:))
              ss = find(jj==stoplights(1,:));
              
              % driver on road at light
              if ii > 0
                  % assert(driver_data(6, kk) == 0);
                  
                  % red light
                  if stoplights(5,ss)==1
                      % assert(G1(ii,jj) == 0);
                      G1(ii,jj) = 1;
                      driver_data(2,kk) = jj;
                      driver_data(4,kk) = t;
                  % green light and open
                  else
                      if street(ii, jj+1) == 0
                          % assert(G1(ii,jj+1) == 0);
                          G1(ii,jj+1) = 1;
                          driver_data(2,kk) = jj+1;
                          driver_data(4,kk) = t;
                      else
                          % no switch lanes at light?
                          % assert(G1(ii,jj) == 0);
                          G1(ii,jj) = 1;
                          driver_data(4,kk) = t;
                      end
                  end
              % driver in queue (at light)
              elseif ii == 0
                  % assert(driver_data(6, kk) > 0);
                  
                  % driver at front of queue
                  if (driver_data(6, kk) == 1) && (driver_data(3,kk) < t)
                      % initial entry point
                      if jj == 0
                          % assert(stoplights(5,1) == 0);
                          
                          for lane = 1:m
                              if street(lane, 1) == 0
                                  % assert(G1(lane,1) == 0);
                                  G1(lane, 1) = 1;
                                  driver_data(1,kk) = lane;
                                  driver_data(2,kk) = 1;
                                  driver_data(4,kk) = t;
                                  % assert(queue_exit(1) == 0);
                                  queue_exit(1) = 1;
                                  break;
                              end
                          end
                      % right turn onto intersection
                      else
                          ss = find(jj==stoplights(1,:));
                          
                          % green light
                          if stoplights(5, ss) == 1
                              % right lane empty
                              if street(m, jj+1) == 0
                                  % assert(G1(m,jj+1) == 0);
                                  G1(m, jj+1) = 1;
                                  driver_data(1,kk) = m;
                                  driver_data(2,kk) = jj + 1;
                                  driver_data(4,kk) = t;
                                  % assert(queue_exit(ss) == 0);
                                  queue_exit(ss) = 1;                                  
                              end
                          % red light
                          else
                              % right lane and preceding spot empty
                              if street(m, jj) == 0 && street(m, jj+1) == 0
                                  % assert(G1(m,jj+1) == 0);
                                  G1(m, jj+1) = 1;
                                  driver_data(1,kk) = m;
                                  driver_data(2,kk) = jj + 1;
                                  driver_data(4,kk) = t;
                                  % assert(queue_exit(ss) == 0);
                                  queue_exit(ss) = 1;                                  
                              end                              
                          end
                      end
                  else
                      % assert(driver_data(1, kk) == 0);
                      driver_data(4, kk) = t;
                  end
              end
            % not at light, spot in front is empty
            elseif street(ii,jj+1) == 0
                % if close to exit and in wrong lane try to switch lanes 
                if (jj > n-spots_must_switch) && (ii ~= dest) && (dest ~= 0)
                    if ii == 1 %if in left lane
                        % assert(dest == m);
                        if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0 && G1(ii+1,jj+1) == 0
                            % assert(G1(ii+1,jj+1) == 0);
                            G1(ii+1,jj+1) = 1;
                            driver_data(1,kk) = ii+1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4,kk) = t;
                        end
                    elseif ii == m %if in right lane
                        % assert(dest == 1);
                        if street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0 && G1(ii-1,jj+1) == 0
                            % assert(G1(ii-1,jj+1) == 0);
                            G1(ii-1,jj+1) = 1;
                            % assert(ii-1 > 0);
                            driver_data(1,kk) = ii-1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4,kk) = t;
                        end
                    else %middle lane
                        ii_up = 0;
                        if (ii < dest)
                            ii_up = ii + 1;
                        else
                            ii_up = ii - 1;
                        end 
                        
                        if street(ii_up,jj) == 0 && street(ii_up,jj+1) == 0 && G1(ii_up,jj+1) == 0
                            % assert(G1(ii_up,jj+1) == 0);
                            G1(ii_up,jj+1) = 1;
                            driver_data(1,kk) = ii_up;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4,kk) = t;
                        end
                    end
                % if far down road in long lane try to switch lanes before going forward
                elseif (jj > 0.7*size(street, 2)) && (ii ~= dest) && (dest ~= 0)
                    if ii == 1 %if in left lane
                        % assert(dest == m);
                        if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0 && G1(ii+1,jj+1) == 0
                            % assert(G1(ii+1,jj+1) == 0);
                            G1(ii+1,jj+1) = 1;
                            driver_data(1,kk) = ii+1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj+1) == 0);
                            G1(ii,jj+1) = 1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        end
                    elseif ii == m %if in right lane
                        % assert(dest == 1);
                        if street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0 && G1(ii-1,jj+1) == 0
                            % assert(G1(ii-1,jj+1) == 0);
                            G1(ii-1,jj+1) = 1;
                            % assert(ii-1 > 0);
                            driver_data(1,kk) = ii-1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj+1) == 0);
                            G1(ii,jj+1) = 1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        end
                    else %middle lane
                        ii_up = 0;
                        if (ii < dest)
                            ii_up = ii + 1;
                        else
                            ii_up = ii - 1;
                        end 
                        
                        if street(ii_up,jj) == 0 && street(ii_up,jj+1) == 0 && G1(ii_up,jj+1) == 0
                            % assert(G1(ii_up,jj+1) == 0);
                            G1(ii_up,jj+1) = 1;
                            driver_data(1,kk) = ii_up;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj+1) == 0);
                            G1(ii,jj+1) = 1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        end
                    end
                else
                    % assert(G1(ii,jj+1) == 0);
                    G1(ii,jj+1) = 1;
                    driver_data(2,kk) = jj+1;
                    driver_data(4,kk) = t;
                end
            % not at light, car blocking forward path
            else
                if m > 1
                    if ii == 1 %if in left lane
                        if jj < n-spots_try_switch || dest == m
                            if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0 && G1(ii+1,jj+1) == 0
                                % assert(G1(ii+1,jj+1) == 0);
                                G1(ii+1,jj+1) = 1;
                                driver_data(1,kk) = ii+1;
                                driver_data(2,kk) = jj+1;
                                driver_data(4,kk) = t;
                            else
                                % assert(G1(ii,jj) == 0);
                                G1(ii,jj) = 1;
                                driver_data(4,kk) = t;
                            end
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4,kk) = t;
                        end
                    elseif ii == m %if in right lane
                        if jj < n-spots_try_switch || dest == 1
                            if street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0 && G1(ii-1,jj+1) == 0
                                % assert(G1(ii-1,jj+1) == 0);
                                G1(ii-1,jj+1) = 1;
                                % assert(ii-1 > 0);
                                driver_data(1,kk) = ii-1;
                                driver_data(2,kk) = jj+1;
                                driver_data(4,kk) = t;
                            else
                                % assert(G1(ii,jj) == 0);
                                G1(ii,jj) = 1;
                                driver_data(4,kk) = t;
                            end
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4,kk) = t;                            
                        end
                    else   %if lanes on either side
                        if street(ii+1,jj) == 0 && street(ii+1,jj+1) == 0 && G1(ii+1,jj+1) == 0 && (jj < n-spots_try_switch || dest ~= 1)
                            % assert(G1(ii+1,jj+1) == 0);
                            G1(ii+1,jj+1) = 1;
                            driver_data(1,kk) = ii+1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        elseif street(ii-1,jj) == 0 && street(ii-1,jj+1) == 0 && G1(ii-1,jj+1) == 0 && (jj < n-spots_try_switch || dest ~= m)
                            % assert(G1(ii-1,jj+1) == 0);
                            G1(ii-1,jj+1) = 1;
                            % assert(ii-1 > 0);
                            driver_data(1,kk) = ii-1;
                            driver_data(2,kk) = jj+1;
                            driver_data(4,kk) = t;
                        else
                            % assert(G1(ii,jj) == 0);
                            G1(ii,jj) = 1;
                            driver_data(4, kk) = t;
                        end
                    end
                else
                    % assert(G1(ii,jj) == 0);
                    G1(ii,jj) = 1;
                    driver_data(4,kk) = t;
                end    
            end
        elseif jj == n
            % make sure the driver exited correctly
            if dest ~= 0
                % assert(ii == dest);
            end
            G1(ii,jj) = 0;
            driver_data(2,kk) = jj+1;
            driver_data(4,kk) = t;
        end
    end
    
    for kk = car_vec
        ii = driver_data(1,kk);
        jj = driver_data(2,kk);
        
        if driver_data(6,kk) == 0
            % assert(jj == n+1 || G1(ii, jj) == 1);
        end
    end
    
    for kk = car_vec
        ii = driver_data(1,kk);
        jj = driver_data(2,kk);
        dest = driver_data(7,kk);
        
        % if near end and not in dest lane
        if (jj > n-spots_must_switch) && (dest ~= 0) && (dest ~= ii)
            if street(ii,jj) == 0
                continue;
            end
            
            % assert(street(ii,jj) == 1);
            % assert(G1(ii,jj) == 1);
            
            if (ii < dest) && (street(ii+1, jj) == 1) && (G1(ii+1, jj+1) == 0)
                % assert(dest == m);
                G1(ii+1,jj+1) = 1;
                G1(ii,jj) = 0;
                driver_data(1,kk) = ii+1;
                driver_data(2,kk) = jj+1;
            elseif (ii > dest) && (street(ii-1, jj) == 1) && (G1(ii-1, jj+1) == 0)
                % assert(dest == 1);
                G1(ii-1,jj+1) = 1;
                G1(ii,jj) = 0;
                driver_data(1,kk) = ii-1;
                driver_data(2,kk) = jj+1;                
            end
        end
    end
            
    
    % move up each car in queue if leader left
    for kk = car_vec
        jj = driver_data(2, kk);
        queued = (driver_data(6 ,kk) > 0);
        
        if driver_data(6 ,kk) == 1 && any((jj-1) == stoplights(1,:))
            ss = find((jj-1) == stoplights(1,:));
            if queue_exit(ss)
                driver_data(6, kk) = driver_data(6, kk) - 1;
                if (driver_data(6,kk) == 0)
                    % assert(driver_data(1,kk) > 0);
                end
            end
        elseif driver_data(6 ,kk) > 1 && any(jj == stoplights(1,:))
            ss = find(jj == stoplights(1,:));
            if queue_exit(ss)
                driver_data(6, kk) = driver_data(6, kk) - 1;
                if (driver_data(6,kk) == 0)
                    % assert(driver_data(1,kk) > 0);
                end
            end
        elseif driver_data(6,kk) == 0
            ii = driver_data(1, kk);
            % assert(ii > 0);
            % assert(jj == n+1 || G1(ii, jj) == 1);
        end
    end
    
    for light = 1:size(stoplights, 2)
        if queue_exit(light)
            stoplights(7, light) = stoplights(7,light) - 1;
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
    for ii = 1:length(stoplights)
        if(stoplights(7,ii) > 20)
            long_queue(1,ii) = long_queue(1,ii) + 1*dt/60;
        end
    end
    
    
end

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
toc


%{
Still to be done:
1. Find way to generate stoplight start/stop patterns
2. Find way to get car to respond to stoplight state
3. Find way to generate new cars at stoplight locations
%}