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

