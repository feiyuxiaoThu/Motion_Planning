#include "GlobalSettings.h"
#include "Astar.h"
#include <math.h>



Vehicle_Traj SearchViaAStar( Grid_3D start_ind, Grid_3D end_ind,XYT_Graph_Search_ xyt_graph_search_, Environment_Scale_ environment_scale_,Vehicle_TPBV_ vehicle_TPBV_){

    Vehicle_Traj ind_vec;

    qp_real grid_space_3D[15][Num_x_nodes_cons][Num_y_nodes_cons][Num_t_nodes_cons];
    /*
    % Information of each element in each node:
    %  Dim # |  Variable
    %  0-2      index of current node
    %  3        f
    %  4        g
    %  5        h
    %  6        is_in_openlist use 1 and -1 
    %  7        is_in_closedlist use 1 and -1
    %  8-10     index of parent node
    %  11-13    parent node's expansion vector
    %  14       orientation angle
    */

   qp_real init_node[15];
   qp_real* cur_best_node;

   //initilize
   init_node[0] = start_ind.x;
   init_node[1] = start_ind.y;
   init_node[2] = start_ind.t;
   init_node[4] = 0;
   init_node[5] = abs(start_ind.x - end_ind.x) + abs(start_ind.y - end_ind.y) + xyt_graph_search_.weight_for_time * abs(start_ind.t - end_ind.t);
   init_node[3] = xyt_graph_search_.multiplier_H_for_A_star * init_node[5];
   init_node[6] = 1;
   init_node[7] = -1;
   init_node[8] = start_ind.x;
   init_node[9] = start_ind.y;
   init_node[10] = -999;
   init_node[11] = 0;
   init_node[12] = 0;
   init_node[13] = 0;
   init_node[14] = vehicle_TPBV_.theta0;





	return ind_vec;
}
