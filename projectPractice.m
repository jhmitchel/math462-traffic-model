clear all
%Define vector of cars on street
car_vec = [];
ind = 0; % will number each car entering the road for reference
dt = 0.25; % duration (sec) of each time step
p = 0.025; % probability of car entering road after timestep dt
end_time = 4000; %how many time steps simulation will run for
time_vec = []; %vector of travel times for cars passing thru completely
%Define Grid for Street
num_lanes = 3;
street_length = 100;
street = zeros(num_lanes,street_length);
%Define Locations of Stoplights
SL_locs = zeros(num_lanes,street_length);
SL_locs(:,50) = 1;
%Define State of Stoplights, 0 for green, 1 for red.
SL_state = zeros(num_lanes,street_length);
%Define Matrix for Driver Data
driver_data = [];
%driver_data(1,:) contains current lane of each car
%driver_data(2,:) contains distance along lane of each car
%driver_data(3,:) contains entry time of each car
%driver_data(4,:) contains exit time of each car
for t = 1:end_time
    m = num_lanes;
    n = street_length;
    G1 = zeros(m,n);
    if isempty(car_vec) %initial car inflow
        G1(:,1) = floor(p*ceil(1/p*rand(m,1)));
        for ii = 1:m
            if G1(ii,1) == 1
                ind = ind+1;
                car_vec = [car_vec ind];
                car_data = [ii;1;t;t];
                driver_data = [driver_data car_data];
            end
        end
        
    else        %with cars on the road
        for kk = car_vec
            ii = driver_data(1,kk);
            jj = driver_data(2,kk);
            if jj < n
                if street(ii,jj+1) == 0
                    G1(ii,jj+1) = 1;
                    driver_data(2,kk) = jj+1;
                    driver_data(4,kk) = t;
                else
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
                end
            elseif jj == n
                G1(ii,jj) = 0;
                driver_data(2,kk) = jj+1;
                driver_data(4,kk) = t;
            end
        end
        for ii = 1:m %additional entry of new cars into street
            if street(ii,1) == 0
                G1(ii,1) = floor(p*ceil(1/p*rand(1)));
                if G1(ii,1) == 1
                    ind = ind+1;
                    car_vec = [car_vec ind];
                    car_data = [ii;1;t;t];
                    driver_data = [driver_data car_data];
                end
            end
        end
    end
    street = G1;
end

%collect travel time data
for jj = 1:length(driver_data)
    if driver_data(4,jj) < end_time
        drive_time = (driver_data(4,jj) - driver_data(3,jj))*dt;
        time_vec = [time_vec drive_time];
    end
end

%generate figure
figure
hist(time_vec);

%{
Still to be done:
1. Find way to generate stoplight start/stop patterns
2. Find way to get car to respond to stoplight state
3. Find way to generate new cars at stoplight locations
%}