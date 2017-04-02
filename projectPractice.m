grid = zeros(10,10);
event_cararrival = 1;
grid(1,1) = 1;
for t = 1:10
    grid = update_step(grid);
    imshow(grid);
end

