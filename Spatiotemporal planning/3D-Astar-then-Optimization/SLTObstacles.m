function obs_slt = SLTObstacles()

    global xyt_graph_search_

    veh1.st = @(t)xyt_graph_search_.obs_vx(1)*t + xyt_graph_search_.obs_x0(1);
    veh1.lt = @(t)xyt_graph_search_.obs_vy(1)*t + xyt_graph_search_.obs_y0(1);

    veh2.st = @(t)xyt_graph_search_.obs_vx(2)*t + xyt_graph_search_.obs_x0(2);
    veh2.lt = @(t)xyt_graph_search_.obs_vy(2)*t + xyt_graph_search_.obs_y0(2);

    veh3.st = @(t)xyt_graph_search_.obs_vx(3)*t + xyt_graph_search_.obs_x0(3);
    veh3.lt = @(t)xyt_graph_search_.obs_vy(3)*t + xyt_graph_search_.obs_y0(3);

    obs_slt = {veh1,veh2,veh3};


end