function dynamic_obs = generateotherveh()

global params_

dynamic_obs = zeros(params_.Nobs,5);

for i = 1:params_.Nobs
    dynamic_obs(i,1) = params_.vehx0(i); 
    dynamic_obs(i,2) = params_.vehy0(i);
    vx = params_.vehvx(i);
    vy = params_.vehvy(i);
    dynamic_obs(i,3) = params_.tf_max*vx + dynamic_obs(i,1); % xf
    dynamic_obs(i,4) = params_.tf_max*vy + dynamic_obs(i,2); % xf
    dynamic_obs(i,5) = params_.radius; % same vehcle size
    
end

end

