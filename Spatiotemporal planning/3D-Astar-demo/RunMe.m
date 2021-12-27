% ==============================================================================
% % A simple 3D A star code
% ==============================================================================
clear all; close all; clc;

global params_
params_.radius = 2;
params_.x_min = -5;
params_.x_max = 40;
params_.y_min = -2;
params_.y_max = 10.5;
params_.x_scale = params_.x_max - params_.x_min;
params_.y_scale = params_.y_max - params_.y_min;
params_.tf_max = 6;
params_.NT = 40;
params_.dt = params_.tf_max / (params_.NT - 1);
params_.NX = 20;
params_.NY = 20;
params_.dx = params_.x_scale / (params_.NX - 1);
params_.dy = params_.y_scale / (params_.NY - 1);
params_.weight_for_time = 100.0;
params_.Nring = 1;
params_.max_iter = 2000;
params_.x0 = 0;
params_.y0 = 2.5;
params_.xf = 35;
params_.yf = 5;

params_.Nobs = 2;
params_.Nfe = params_.NT * 3;


params_.vehx0 = [-5,8];
params_.vehy0 = [3,-1.5];
params_.vehvx = [2,4];%[12,3];%[2,4];
params_.vehvy = [0,0.3];

params_.obs = generateotherveh();%GenerateDynamicObstacles();

global x y 

[x, y] = SearchTrajViaAstar();

traplot();
