#include "GlobalSettings.h"
#pragma once

typedef struct
{
    Grid_3D cur_index;
    qp_real f;
    qp_real g;
    qp_real h;
    bool is_in_openlist;
    bool is_in_closedlist;
    Grid_3D parent_index;
    qp_real expansion_vec[3];
    qp_real theta;
/*
% Information of each element in each node:
%  Dim # |  Variable
%  1-3      index of current node
%  4        f
%  5        g
%  6        h
%  7        is_in_openlist
%  8        is_in_closedlist
%  9-11     index of parent node
%  12-14    parent node's expansion vector
%  15       orientation angle
*/ 

}Astar_Node;

bool point_eq(Grid_3D a, Grid_3D b);

void add_to_open_set(Grid_3D p, int* open_set_count, Grid_3D *open_set);

void remove_from_open_set(Grid_3D p, int* open_set_count, Grid_3D *open_set);

Grid_3D get_point_with_lowest_f_score(int* open_set_count, Grid_3D *open_set,Astar_Node grid_space_3D[][Num_y_nodes_cons][Num_t_nodes_cons]);
