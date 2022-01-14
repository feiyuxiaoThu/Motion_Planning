#include "GlobalSettings.h"
#include "Astar.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <limits.h>


Vehicle_Traj SearchViaAStar( Grid_3D start_ind, Grid_3D end_ind,XYT_Graph_Search_ xyt_graph_search_, Environment_Scale_ environment_scale_,Vehicle_TPBV_ vehicle_TPBV_){

    Vehicle_Traj ind_vec;

    Astar_Node grid_space_3D[Num_x_nodes_cons][Num_y_nodes_cons][Num_t_nodes_cons];
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

    Astar_Node init_node;
    Astar_Node* cur_best_node;

    Grid_3D open_set[MAX_NODE_COUNT];
    int open_set_count = 0; // important

    //initilize
    init_node.cur_index = start_ind;
    init_node.g = 0;
    init_node.h = abs(start_ind.x - end_ind.x) + abs(start_ind.y - end_ind.y) + xyt_graph_search_.weight_for_time * abs(start_ind.t - end_ind.t);
    init_node.f = xyt_graph_search_.multiplier_H_for_A_star * init_node.h;
    init_node.is_in_openlist = 1;
    init_node.is_in_closedlist = 0;
    init_node.parent_index = start_ind;
    init_node.expansion_vec[0] = 0;
    init_node.expansion_vec[1] = 0;
    init_node.expansion_vec[2] = 0;
    init_node.theta = vehicle_TPBV_.theta0;

    add_to_open_set(init_node.cur_index, &open_set_count, open_set);

    grid_space_3D[init_node.cur_index.x][init_node.cur_index.y][init_node.cur_index.t] = init_node;

    cur_best_node = &init_node; // point

    qp_int expansion_pattern[6][3] = {{1,1,1},{1,0,1},{1,-1,1},{0,1,1},{0,0,1},{0,-1,1}};

    qp_real length_type_1 = sqrt(2+xyt_graph_search_.weight_for_time*xyt_graph_search_.weight_for_time);

    qp_real length_type_2 = sqrt(1+xyt_graph_search_.weight_for_time*xyt_graph_search_.weight_for_time);

    qp_real length_type_3 = abs(xyt_graph_search_.weight_for_time);

    qp_real expansion_length[6] = {length_type_1,length_type_2,length_type_1,length_type_2,length_type_3,length_type_2};

    bool completeness_flag = 0;

    int iter = 0;

    while((open_set_count > 0) && (iter <= xyt_graph_search_.max_iter) && (!completeness_flag)){
        iter = iter + 1;
        


    }



    return ind_vec;
}


bool point_eq(Grid_3D a, Grid_3D b){
    return a.x == b.x && a.y == b.y && a.t == b.t;
}

void add_to_open_set(Grid_3D p, int* open_set_count, Grid_3D *open_set){
    for(int i = 0; i < *open_set_count; i++) {
        Grid_3D q = open_set[i];
        if(p.x == q.x && p.y == q.y && p.t == q.t) {
            return; // The point is already in the set.
        }
    }
    open_set[*open_set_count++] = p;
}

void remove_from_open_set(Grid_3D p, int* open_set_count, Grid_3D *open_set){
    for(int i = 0; i < *open_set_count; i++) {
        Grid_3D q = open_set[i];
        if(p.x == q.x && p.y == q.y && p.t == q.t) {
            // Found it, replace with last item in array
            open_set[i] = open_set[*open_set_count - 1];
            open_set_count--;
            return;
        }
    }
    printf("Failed to remove point (%d, %d) from open set.\n", p.x, p.y);
    abort();
}

Grid_3D get_point_with_lowest_f_score(int* open_set_count, Grid_3D *open_set,Astar_Node grid_space_3D[][Num_y_nodes_cons][Num_t_nodes_cons]){
    int lowest_f_score = INT_MAX;
    Grid_3D best = { -1, -1 ,-1};
    for(int i = 0; i < *open_set_count; i++) {
        Grid_3D q = open_set[i];
        Astar_Node n = grid_space_3D[q.x][q.y][q.t];
        if(n.f < lowest_f_score) {
            lowest_f_score = n.f;
            best = q;
        }
    }
    assert(best.x > -1 && best.y > -1 && best.t > -1); // Must find something.
    return best;
}