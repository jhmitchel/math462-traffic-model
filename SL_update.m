function stoplights = SL_update(stoplights,t,dt)

SL_new = stoplights;

state_mat = zeros(9,size(SL_new,2));
state_mat(2,:) = -(ceil(SL_new(2,:)*60/dt));
state_mat(2,:) = state_mat(2,:) - t;
state_mat(3,:) = ceil(SL_new(3,:)*60/dt);
state_mat(4,:) = ceil(SL_new(4,:)*60/dt);
state_mat(5,:) = SL_new(5,:);
state_mat(1,:) = state_mat(3,:)+state_mat(4,:);
state_mat(6,:) = mod(state_mat(2,:),state_mat(1,:));
state_mat(7,:) = state_mat(6,:)./state_mat(1,:);
state_mat(8,:) = state_mat(3,:)./state_mat(1,:);
for ii = 1:size(SL_new,2)
    if state_mat(7,ii) > state_mat(8,ii)
        state_mat(9,ii) = 1;
        SL_new(5,ii) = 1;
    else
        SL_new(5,ii) = 0;
    end
end

stoplights = SL_new;


