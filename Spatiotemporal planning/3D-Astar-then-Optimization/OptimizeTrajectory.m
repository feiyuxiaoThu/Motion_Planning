function trajectory = OptimizeTrajectory(path)
global terminal_moment param_ trajectory0 trajectory_matlab
terminal_moment = path(end, 3);
trajectory0 = Resample3DPath(path);

global Boxes_bound
Boxes_bound = SpecifyBoxRegionsForWaypoints(trajectory0);

% plot boxes

%Boxplot(Boxes_bound);

%trajectory = trajectory0;
%trajectory = QP_matlab(trajectory0,Boxes_bound);

%% QP problems
% contraints
%global AA_global u_s_global l_s_global Aeq_global beq_global

[AA_global,u_s_global,l_s_global,Aeq_global,beq_global] = QP_matlab(trajectory0,Boxes_bound);

% weight matrix


sizen = param_.Nfe; % both l and s
%global Q q
Q = eye(6*sizen);
q = zeros(6*sizen,1);

for i = sizen+1:2*sizen
    Q(i,i) = 2*param_.weight_for_drastic_lat_vel_change; % 
end

for i = 2*sizen+1:3*sizen
    Q(i,i) = 2*param_.weight_for_collision_risks;
end

for i = 4*sizen+1:5*sizen
    Q(i,i) =  2*param_.weight_for_drastic_long_vel_change;
end

for i = 5*sizen+1:6*sizen
    Q(i,i) = 2*param_.weight_for_collision_risks;
end

for i = 1:sizen
    q(i) = -2*trajectory0(i,1);
end

for i = 3*sizen+1:4*sizen
    q(i) = -2*trajectory0(i-3*sizen,2);
end

for i = 4*sizen+1:5*sizen
    q(i) = -2*param_.vehicle_v_suggested_long*param_.weight_inside_exp;
end

%Q = sparse(Q);

A_qp = cat(1,AA_global,-AA_global);
b_qp = cat(1,u_s_global,-l_s_global);

%A_qp = sparse(A_qp);
%Aeq_global = sparse(Aeq_global);

options = qpip('defaults');
%trajectory = quadprog(Q,q,A_qp,b_qp,Aeq_global,beq_global);
[trajectory0,state] = qpip(Q,q,A_qp,b_qp,Aeq_global,beq_global,options);
trajectory_matlab = quadprog(Q,q,A_qp,b_qp);

x_all = trajectory0.x;
trajectory = zeros(sizen,3);

if state == 1
   fprintf("qp ok");
else
    fprintf('qp fail');
end


for i = 1:sizen
    trajectory(i,1) = x_all(3*sizen+i);
    trajectory(i,2) = x_all(i);
    trajectory(i,3) = (i-1)*param_.tf/(sizen-1);
end
%!ampl rr.run
%load x.txt
%load y.txt
%nfe = length(x);
%index = round(linspace(1, nfe, length(trajectory0)));
%trajectory(:,1) = x(index);
%trajectory(:,2) = y(index);
%trajectory(:,3) = linspace(0, terminal_moment, length(trajectory0));
end

function Boxes_mat = SpecifyBoxRegionsForWaypoints(trajectory0)
delete('Boxes');
fid = fopen('Boxes', 'w');
Boxes_mat = zeros(4,length(trajectory0));
for ii = 1 : length(trajectory0)
    [x_min, x_max, y_min, y_max] = GetObstacleBoundsAtTime(trajectory0(ii,1), trajectory0(ii,2), trajectory0(ii,3));
    fprintf(fid,'%g 1 %f \r\n', ii, x_min);
    fprintf(fid,'%g 2 %f \r\n', ii, x_max);
    fprintf(fid,'%g 3 %f \r\n', ii, y_min);
    fprintf(fid,'%g 4 %f \r\n', ii, y_max);
    Boxes_mat(1,ii) = x_min;
    Boxes_mat(2,ii) = x_max;
    Boxes_mat(3,ii) = y_min;
    Boxes_mat(4,ii) = y_max;
    
end
fclose(fid);
end

function [x_min, x_max, y_min, y_max] = GetObstacleBoundsAtTime(cur_x, cur_y, cur_time)
global environment_
counter = 0;
while (counter < 100)
    ds = counter * 0.01;
    if (IsEgoVehiclePositionValid(cur_x, cur_y + ds, cur_time))
        cur_y = cur_y + ds;
        break;
    end
    if (IsEgoVehiclePositionValid(cur_x, cur_y - ds, cur_time))
        cur_y = cur_y - ds;
        break;
    end
    if (IsEgoVehiclePositionValid(cur_x  + ds, cur_y, cur_time))
        cur_x = cur_x + ds;
        break;
    end
    if (IsEgoVehiclePositionValid(cur_x  - ds, cur_y, cur_time))
        cur_x = cur_x - ds;
        break;
    end
    counter = counter + 1;
end

for rate = linspace(2, 0, 10)
    is_cur_rate_valid = 1;
    for ii = 1 : length(environment_.obstacles)
        obs_st = environment_.obstacles{1,ii}.st(cur_time);
        obs_lt = environment_.obstacles{1,ii}.lt(cur_time);
        if (IsV1CollidingWithBoxAtRate(obs_st, obs_lt, cur_x, cur_y, rate))
            is_cur_rate_valid = 0;
            break;
        end
    end
    if (is_cur_rate_valid == 1)
        break;
    end
end
if (~is_cur_rate_valid)
    disp('The coarse trajectory is not collision-free.');
    error 'Algorithm returns with a failure.';
end
x_min = cur_x - rate;
x_max = cur_x + rate;
y_min = cur_y - rate;
y_max = cur_y + rate;
end

function val = IsEgoVehiclePositionValid(cur_x, cur_y, cur_time)
global environment_ param_
val = 1;

y_lb = cur_y - param_.vehicle_width * 0.5;
y_ub = cur_y + param_.vehicle_width * 0.5;
if (y_lb < environment_.road_right_barrier)||(y_ub > environment_.road_left_barrier)
    val = 0;
    return;
end

for ii = 1 : length(environment_.obstacles)
    obs_st = environment_.obstacles{1,ii}.st(cur_time);
    obs_lt = environment_.obstacles{1,ii}.lt(cur_time);
    if (IsV1CollidingWithV2(cur_x, cur_y, obs_st, obs_lt))
        val = 0;
        return;
    end
end
end

function val = IsV1CollidingWithBoxAtRate(obs_st, obs_lt, cur_x, cur_y, rate)
global param_ environment_
vehicle_half_width = param_.vehicle_width * 0.5;
v1.xmin = obs_st - param_.vehicle_rear_hang;
v1.xmax = obs_st + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v1.ymin = obs_lt - vehicle_half_width;
v1.ymax = obs_lt + vehicle_half_width;

v2.xmin = cur_x - rate - param_.vehicle_rear_hang;
v2.xmax = cur_x + rate + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v2.ymin = cur_y - rate - vehicle_half_width;
v2.ymax = cur_y + rate + vehicle_half_width;

if (v2.ymin < environment_.road_right_barrier)||(v2.ymax > environment_.road_left_barrier)
    val = 1;
    return;
end

if ((v2.ymax < v1.ymin) || (v1.ymax < v2.ymin) || (v2.xmax < v1.xmin) || (v1.xmax < v2.xmin))
    val = 0;
else
    val = 1;
end
end

function trajectory = Resample3DPath(path)
global param_
trajectory = [];
for ii = 1 : (size(path,1) - 1)
    node1 = path(ii,:);
    node2 = path(ii+1,:);
    distance = abs(node1(3) - node2(3));
    num_mesh_grids = round(distance * 10000);
    
    mat = [linspace(node1(1), node2(1), num_mesh_grids)', linspace(node1(2), node2(2), num_mesh_grids)', linspace(node1(3), node2(3), num_mesh_grids)'];
    trajectory = [trajectory; mat];
end
index = round(linspace(1, length(trajectory), param_.Nfe));
trajectory = trajectory(index, :);
end

function val = IsV1CollidingWithV2(s_ego, l_ego, s_other, l_other)
global param_
vehicle_half_width = param_.vehicle_width * 0.5;
v1.xmin = s_ego - param_.vehicle_rear_hang;
v1.xmax = s_ego + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v1.ymin = l_ego - vehicle_half_width;
v1.ymax = l_ego + vehicle_half_width;

v2.xmin = s_other - param_.vehicle_rear_hang;
v2.xmax = s_other + param_.vehicle_front_hang + param_.vehicle_wheelbase;
v2.ymin = l_other - vehicle_half_width;
v2.ymax = l_other + vehicle_half_width;

if ((v2.ymax < v1.ymin) || (v1.ymax < v2.ymin) || (v2.xmax < v1.xmin) || (v1.xmax < v2.xmin))
    val = 0;
else
    val = 1;
end
end

function Boxplot(Boxes_mat)
 num_box = size(Boxes_mat,2);
 figure;
 
 for i = 1:num_box
     rectangle('Position',[Boxes_mat(1,i) Boxes_mat(3,i) Boxes_mat(2,i)-Boxes_mat(1,i) Boxes_mat(4,i)-Boxes_mat(3,i)]);
     hold on;
 end
 
 axis([0 100 0 6]);
 
end



