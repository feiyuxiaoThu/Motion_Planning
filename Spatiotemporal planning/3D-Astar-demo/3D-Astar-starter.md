# 三维 A* 搜索算法初探 

此处为对于 SearchTrjViaAstar() 这一函数的剖析
首先是主函数入口，要对于坐标向三维 的离散网格进行投影，再调用 SearhViaAStar() 函数进行 A* 搜索

```matlab
function [x, y] = SearchTrajViaAstar()
global params_
start_ind = ConvertConfigToIndex(params_.x0, params_.y0, 0);
goal_ind = ConvertConfigToIndex(params_.xf, params_.yf, params_.tf_max);
ind_vec = SearchViaAStar(start_ind, goal_ind);
x = params_.x_min + (ind_vec(:,1)' - 1) .* params_.dx;
y = params_.y_min + (ind_vec(:,2)' - 1) .* params_.dy;
end
```
一个简单的将坐标投射到格点内的函数（*hybrid Astar* 算法此处会比较复杂）

```matlab
function ind = ConvertConfigToIndex(x, y, time)
global params_
ind1 = round((x - params_.x_min) / params_.dx) + 1;
ind2 = round((y - params_.y_min) / params_.dy) + 1;
ind3 = round(time / params_.dt) + 1;
ind = [min(max(1, ind1), params_.NX), min(max(1, ind2), params_.NY), min(max(1, ind3), params_.NT)];
end
```

接下来就是主函数 主要需要注意其启发函数设计

其 $h$ 的设计为曼哈顿距离，但是对于时间有一个额外的权项 *weight_for_time* 

```matlab
function ind_vec = SearchViaAStar(start_ind, goal_ind)
global params_ grid_space
grid_space = cell(params_.NX, params_.NY, params_.NT);
init_node.id = start_ind;
init_node.g = 0; 
init_node.h = sum(abs(start_ind(1:2) - goal_ind(1:2))) + params_.weight_for_time * abs(start_ind(3) - goal_ind(3)); % 曼哈顿距离启发函数
init_node.f = init_node.g + init_node.h;
init_node.is_in_openlist = 1;
init_node.is_in_closedlist = 0;
init_node.parent_id = [-999 -999 -999];
init_node.parent_operation = [0 0 0];
global openlist openlist_f
openlist = init_node.id;
openlist_f = init_node.f;

grid_space{init_node.id(1), init_node.id(2), init_node.id(3)} = init_node;
[expansion_pattern, expansion_length] = GeneratePattern(); % ？？？

ready_flag = 0;
cur_best_id = [];
cur_best_val = Inf;
iter = 0;
while ((~isempty(openlist))&&(iter <= params_.max_iter)&&(~ready_flag)) % 终止条件：1. openlist 为空 2. 达到最大迭代次数 3. 满足目标条件（如到达目标点）
    iter = iter + 1;
    cur_id = ExtractNodeFromOpenlist(); % 从 openlist 中选出 f 最小的一个点
    cur_node = grid_space{cur_id(1), cur_id(2), cur_id(3)};
    cur_ind = cur_node.id;
    cur_g = cur_node.g;
    parent_operation = cur_node.parent_operation;
    for ii = 1 : size(expansion_pattern, 1)
        child_node_ind = cur_ind + expansion_pattern(ii, :);
        if ((child_node_ind(1) < 1)||(child_node_ind(2) < 1)||...
                (child_node_ind(3) < 1)||(child_node_ind(1) > params_.NX)||...
                (child_node_ind(2) > params_.NY)||...
                (child_node_ind(3) > params_.NT))
            continue; % out the grid space
        end
        if ((~isempty(grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))...
                &&(grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)}.is_in_closedlist == 1))
            continue;
        end
        child_g = cur_g + expansion_length(ii) + expansion_length(ii) + sum(abs(parent_operation - expansion_pattern(ii, :)));
        child_h = sum(abs(child_node_ind(1:2) - goal_ind(1:2))) + params_.weight_for_time * abs(child_node_ind(3) - goal_ind(3));
        child_f = child_g + child_h;
        child_node.id = child_node_ind;
        child_node.g = child_g;
        child_node.h = child_h;
        child_node.f = child_f;
        child_node.is_in_openlist = 1;
        child_node.is_in_closedlist = 0;
        child_node.parent_id = cur_ind;
        child_node.parent_operation = expansion_pattern(ii, :);
        
        % If the child node has been explored ever before (but not closed yet)
        if (~isempty(grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)}.g > child_g + 0.01)
                DeleteSpecifiedNodeFromOpenlist(child_node_ind);
                grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node;
                openlist_f = [openlist_f, child_node.f];
                openlist = [openlist; child_node.id];
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (IsNodeValid(cur_node, child_node, expansion_length(ii)))
                openlist_f = [openlist_f, child_node.f];
                openlist = [openlist; child_node.id];
                grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node;
                if (~any(child_node_ind - goal_ind))
                    cur_best_id = child_node_ind;
                    ready_flag = 1;
                    break;
                end
                if (child_node.h < cur_best_val)
                    cur_best_id = child_node_ind;
                end
            else % If the child node involves collisons
                child_node.is_in_openlist = 0;
                child_node.is_in_closedlist = 1;
                grid_space{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node;
            end
        end
    end
end
ind_vec = cur_best_id;
parent_ind = grid_space{ind_vec(1), ind_vec(2), ind_vec(3)}.parent_id;
while (parent_ind(3) ~= -999)
    ind_vec = [parent_ind; ind_vec];
    parent_ind = grid_space{parent_ind(1), parent_ind(2), parent_ind(3)}.parent_id;
end
end
```

下面两个函数分别实现的是 决定节点的膨胀方式 和 获得新的节点

```matlab
function [expansion_pattern, expansion_length] = GeneratePattern()
global params_
expansion_pattern = [];
expansion_length = [];
for ii = -params_.Nring : params_.Nring
    for jj = -params_.Nring : params_.Nring
        expansion_pattern = [expansion_pattern; [ii, jj, 1]];
        expansion_length = [expansion_length, hypot(ii, jj)];
    end
end
end

function cur_node_id = ExtractNodeFromOpenlist()
global openlist openlist_f grid_space
ind = find(openlist_f == min(openlist_f));
ind = ind(end);
cur_node_id = openlist(ind, :);
openlist(ind, :) = [];
openlist_f(ind) = [];
grid_space{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_in_closedlist = 1;
grid_space{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_in_openlist = 0;
end
```

一个比较重要的函数是判定节点是否有效的函数 主要是判定当前的节点所在网格是否被障碍物所占据：

由于这里物体和障碍物建模均使用圆形，所以其碰撞检测十分简单（只需要检测其圆心之间的距离即可），注意需要检测从 cur_node 到 child_node 的路径上的所有点是否有碰撞。

```matlab
function is_valid = IsNodeValid(cur_node, child_node, length)
is_valid = 0;
global params_ grid_space
child_id = child_node.id;
cur_node_id = cur_node.id;
parent_node_id = grid_space{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.parent_id;
% if (parent_node_id(3) ~= -999)
%     vx1 = (parent_node_id(1) - cur_node_id(1)) * params_.dx / params_.dt;
%     vx2 = (child_id(1) - parent_node_id(1)) * params_.dx / params_.dt;
%     ax = (vx1 - vx2) / params_.dt;
%     vy1 = (parent_node_id(2) - cur_node_id(2)) * params_.dy / params_.dt;
%     vy2 = (child_id(2) - parent_node_id(2)) * params_.dy / params_.dt;
%     ay = (vy1 - vy2) / params_.dt;
%     if (hypot(ax, ay) > params_.a_max)
%         return;
%     end
% end

N = round(length * 10) + 1;
d1 = linspace(cur_node.id(1), child_node.id(1), N);
d2 = linspace(cur_node.id(2), child_node.id(2), N);
d3 = linspace(cur_node.id(3), child_node.id(3), N);
for ii = 1 : N
    time = (d3 - 1) * params_.dt;
    x = (d1 - 1) * params_.dx + params_.x_min;
    y = (d2 - 1) * params_.dy + params_.y_min;
    for jj = 1 : params_.Nobs
        [obs_x, obs_y, obs_r] = SpecifyObsLocationAtTimeInstance(time, jj);
        if (hypot(x - obs_x, y - obs_y) < 1.2 * (obs_r + params_.radius)) % 碰撞检测
            return;
        end
    end
end
is_valid = 1;
end
```

此处有一个关于动态障碍物的位置估计，一般而言我们可以得到当前时刻测定的障碍物的位置和速度，简单起见可以假定其匀速运动，但是对复杂运动的预测又是一个更加复杂并且十分有趣的话题了，此处不表。

```matlab
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
```

