function stoplights = SL_update_sensor(stoplights, street)

SL_new = stoplights;

if size(street, 2) > 0
    for light = 2:size(SL_new,2)
        light_dist = SL_new(1, light);
        queue_length = SL_new(7, light);
        status_time = SL_new(8, light);
        ready_change = status_time > 240;        
        sitting_at_light = (sum(street(:,light_dist)) > 0);
        
        if sitting_at_light && (queue_length < 20) && ready_change
            SL_new(5, light) = 0;
            SL_new(8, light) = 0;
        elseif queue_length > 0 && ready_change
            SL_new(5, light) = 1;
            SL_new(8, light) = 0;
        else
            SL_new(8,light) = status_time + 1;
        end
    end
end

stoplights = SL_new;


