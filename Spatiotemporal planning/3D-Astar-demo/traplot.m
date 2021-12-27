function traplot()


global x y params_


dt = params_.dt;


len_veh = 2;
width_veh = 1.5;

len = length(x);





%plot(traj_x,traj_y,'--');%画轨迹如果不需要删掉即可

for i = 1: len

    hold on

    rectangle('Position',[x(i)-len_veh y(i)-width_veh 2*len_veh 2*width_veh]);
    for ii = 1:params_.Nobs
        hold on
        time = i*params_.tf_max/len;
        [obs_x, obs_y, obs_r] = SpecifyObsLocationAtTimeInstance(time, ii);
        rectangle('Position',[obs_x obs_y 2*len_veh 2*width_veh],'EdgeColor','r');
    end
    %axis([environment_scale_.environment_x_min,environment_scale_.environment_x_max,environment_scale_.environment_y_min,environment_scale_.environment_y_max]);
    %axis equal;

    pause(dt);



end

axis equal;



end


function [obs_x, obs_y, obs_r] = SpecifyObsLocationAtTimeInstance(time, jj)
global params_
vec = params_.obs(jj, :);
x0 = vec(1);
y0 = vec(2);
obs_r = vec(5);
xf = vec(3);
yf = vec(4);

rate = time / params_.tf_max;
obs_x = x0 + (xf - x0) * rate;
obs_y = y0 + (yf - y0) * rate;
end