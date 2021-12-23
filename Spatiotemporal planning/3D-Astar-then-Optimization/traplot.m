function traplot()

global vehicle_geometrics_
global x y theta dynamic_obs
global xyt_graph_search_ 
global environment_scale_

dt = xyt_graph_search_.resolution_t;


len_veh = vehicle_geometrics_.vehicle_wheelbase/2;
width_veh = vehicle_geometrics_.vehicle_width/2;

len = length(x);





%plot(traj_x,traj_y,'--');%画轨迹如果不需要删掉即可

for i = 1: len

    hold on

    rectangle('Position',[x(i)-len_veh y(i)-width_veh 2*len_veh 2*width_veh]);
    for ii = 1:xyt_graph_search_.num_otherveh
        hold on
        rectangle('Position',[dynamic_obs{i,ii}.x(1) dynamic_obs{i,ii}.y(1) 2*len_veh 2*width_veh],'EdgeColor','r');
    end
    axis([environment_scale_.environment_x_min,environment_scale_.environment_x_max,environment_scale_.environment_y_min,environment_scale_.environment_y_max]);
    %axis equal;

    pause(dt);



end

axis equal;

end