function obstacle_cell = GenerateDynamicObstacles(input_t)
global xyt_graph_search_
global vehicle_geometrics_
%load DynObs
time = input_t;

Nobs = xyt_graph_search_.num_otherveh; %size(dynamic_obs,2);
obstacle_cell = cell(time, Nobs);
A = vehicle_geometrics_.vehicle_wheelbase*vehicle_geometrics_.vehicle_width;

len_veh = vehicle_geometrics_.vehicle_wheelbase/2; % half length
width_veh = vehicle_geometrics_.vehicle_width/2;

for ii = 1 : Nobs
    dx = xyt_graph_search_.obs_vx(ii);%dynamic_obs{end,ii}.x(1) - dynamic_obs{1,ii}.x(1);
    dy = xyt_graph_search_.obs_vy(ii);%dynamic_obs{end,ii}.y(1) - dynamic_obs{1,ii}.y(1);
    x_pos0 = [xyt_graph_search_.obs_x0(ii)-len_veh xyt_graph_search_.obs_x0(ii)-len_veh xyt_graph_search_.obs_x0(ii)+len_veh xyt_graph_search_.obs_x0(ii)+len_veh]; %xyt_graph_search_.obs_x0(ii)-len_veh];
    y_pos0 = [xyt_graph_search_.obs_y0(ii)-width_veh xyt_graph_search_.obs_y0(ii)-width_veh xyt_graph_search_.obs_y0(ii)+width_veh xyt_graph_search_.obs_y0(ii)+width_veh];% xyt_graph_search_.obs_y0(ii)-width_veh];
    for jj = 1 : time
        temp.x = x_pos0 + dx / xyt_graph_search_.num_nodes_t * (jj - 1);
        temp.y = y_pos0 + dy / xyt_graph_search_.num_nodes_t * (jj - 1);
        temp.A = A;
        obstacle_cell{jj, ii} = temp;
    end
end
end