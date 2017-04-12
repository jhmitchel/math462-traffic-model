function stoplights = SL_update_sensor(stoplights, driver_data)

SL_new = stoplights;

if size(driver_data, 2) > 0
    for light = 1:size(SL_new,2)
        light_dist = SL_new(1, light);
        queue_length = SL_new(7, light);
        status_time = SL_new(8, light);
        if any(driver_data(2,:) == light_dist) && (queue_length < 20) && status_time > 240
            SL_new(5, light) = 0;
            SL_new(8, light) = 0;
        elseif queue_length > 0 && status_time > 240
            SL_new(5, light) = 1;
            SL_new(8, light) = 0;
        else
            SL_new(8,light) = status_time + 1;
        end
    end
end

stoplights = SL_new;


