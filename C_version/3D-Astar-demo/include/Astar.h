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

bool cmp(Astar_Node x, Astar_Node y); // for sort 
