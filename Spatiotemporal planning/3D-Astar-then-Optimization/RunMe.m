
%  备注：
%  1. 请读者支持正版AMPL
%  2. 基于该部分代码的研究成果须引用以下参考文献：
%  a) B. Li, and Z. Shao, “A unified motion planning method for
%     parking an autonomous vehicle in the presence of irregularly placed
%     obstacles,” Knowledge-Based Systems, vol. 86, pp. 11–20, 2015.
%  b) B. Li, K. Wang, and Z. Shao, “Time-optimal maneuver planning in
%     automatic parallel parking using a simultaneous dynamic optimization
%     approach,” IEEE Transactions on Intelligent Transportation Systems, vol.
%     17, no. 11, pp. 3263–3274, 2016.
%  3. 由于初始解质量一般，数值求解收敛失败的可能性较大.
% ==============================================================================
clear all
close all
clc

% % 参数设置
global vehicle_geometrics_ % 车辆轮廓几何尺寸
vehicle_geometrics_.vehicle_wheelbase = 2.8;
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
global vehicle_kinematics_ % 车辆运动能力参数
vehicle_kinematics_.vehicle_v_max = 25;
vehicle_kinematics_.vehicle_a_max = 3;
vehicle_kinematics_.vehicle_phy_max = 0.7;
vehicle_kinematics_.vehicle_w_max = 0.1;
vehicle_kinematics_.vehicle_kappa_max = tan(vehicle_kinematics_.vehicle_phy_max) / vehicle_geometrics_.vehicle_wheelbase;
vehicle_kinematics_.vehicle_turning_radius_min = 1 / vehicle_kinematics_.vehicle_kappa_max;
global environment_scale_ % 车辆所在环境范围
environment_scale_.environment_x_min = -5;
environment_scale_.environment_x_max = 60;
environment_scale_.environment_y_min = -2;
environment_scale_.environment_y_max = 5;
environment_scale_.x_scale = environment_scale_.environment_x_max - environment_scale_.environment_x_min;
environment_scale_.y_scale = environment_scale_.environment_y_max - environment_scale_.environment_y_min;
% % 用于X-Y-T图搜索的A星算法涉及的参数
global xyt_graph_search_
xyt_graph_search_.max_t = 8;
xyt_graph_search_.num_nodes_t = 21;
xyt_graph_search_.resolution_t = xyt_graph_search_.max_t / (xyt_graph_search_.num_nodes_t - 1);
xyt_graph_search_.resolution_x = xyt_graph_search_.resolution_t * vehicle_kinematics_.vehicle_v_max / (3 * 1.414);
xyt_graph_search_.resolution_y = xyt_graph_search_.resolution_x;
xyt_graph_search_.num_nodes_x = ceil(environment_scale_.x_scale / xyt_graph_search_.resolution_x);
xyt_graph_search_.num_nodes_y = ceil(environment_scale_.y_scale / xyt_graph_search_.resolution_y);
xyt_graph_search_.multiplier_H_for_A_star = 1.0;
xyt_graph_search_.weight_for_time = 2.0;
xyt_graph_search_.max_iter = 20000;
xyt_graph_search_.num_otherveh = 3; % num of vehecles can be ajusted!
xyt_graph_search_.obs_vx = [12,18,15,18];
xyt_graph_search_.obs_vy = [-0.5,-0.1,0.2,-0.5];
xyt_graph_search_.obs_x0 = [10,5,20,20];
xyt_graph_search_.obs_y0 = [5.5,7.5,3.5,3.5];
% % 导入既定算例以及静止障碍物分布情况
global vehicle_TPBV_ dynamic_obs
%load TaskSetup.mat
vehicle_TPBV_.x0 = 0;
vehicle_TPBV_.y0 = 2.5;
vehicle_TPBV_.theta0 = -pi;
vehicle_TPBV_.xtf = 60.0;
vehicle_TPBV_.ytf = 6.5;
vehicle_TPBV_.thetatf = -pi;

dynamic_obs = GenerateDynamicObstacles(xyt_graph_search_.num_nodes_t);

% % for QP solvers and will be modified
global environment_
environment_.num_obs = xyt_graph_search_.num_nodes_t;
environment_.obs_agv_vel = 10;
environment_.road_left_barrier = environment_scale_.environment_y_max;
environment_.road_right_barrier = environment_scale_.environment_y_min;
environment_.obstacles = SLTObstacles();

global param_
param_.Nfe = xyt_graph_search_.num_nodes_t*5;
param_.vehicle_width = vehicle_geometrics_.vehicle_width;
param_.vehicle_rear_hang = vehicle_geometrics_.vehicle_rear_hang;
param_.vehicle_front_hang = vehicle_geometrics_.vehicle_front_hang;
param_.vehicle_wheelbase = vehicle_geometrics_.vehicle_wheelbase;
param_.tf = xyt_graph_search_.max_t;
param_.distance_near = 50;
param_.weight_inside_exp = 2;
param_.weight_for_collision_risks = 5;
param_.weight_for_drastic_long_vel_change = 10;
param_.weight_for_drastic_lat_vel_change = 20;
param_.weight_for_biased_long = 1;
param_.weight_for_biased_lat = 1;
param_.vehicle_v_max_long = 20.0;
param_.vehicle_v_suggested_long = 15.0;

%% % X-Y-T图搜索
start_ind = ConvertConfigToIndex(vehicle_TPBV_.x0, vehicle_TPBV_.y0, 0);
goal_ind = ConvertConfigToIndex(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, xyt_graph_search_.max_t);
tic
global x y theta
[x, y, theta, fitness] = SearchTrajectoryInXYTGraph(start_ind, goal_ind);
disp(['CPU time elapsed for A* search: ',num2str(toc), ' sec.'])

% % 轨迹规划
[x, y, theta, vv, a, phy, w] = FormInitialGuess(x, y, theta);
figure(1);
traplot();
plot(x,y);

%% For QP optimization(Always feasible QP)
traj_t = 0:xyt_graph_search_.resolution_t:xyt_graph_search_.max_t;
global coarse_trajectory precise_trajectory 

coarse_trajectory = zeros(length(x),3);
coarse_trajectory(:,1) = x;
coarse_trajectory(:,2) = y;
coarse_trajectory(:,3) = traj_t;
tic
global terminal_moment  trajectory0 trajectory_matlab
precise_trajectory = OptimizeTrajectory(coarse_trajectory);
disp(['QP OP',num2str(toc),'sec']);
x = precise_trajectory(:,1);
y = precise_trajectory(:,2);

dynamic_obs = GenerateDynamicObstacles(param_.Nfe); % for plot change to more dense
figure(2);
traplot();
plot(x,y);
