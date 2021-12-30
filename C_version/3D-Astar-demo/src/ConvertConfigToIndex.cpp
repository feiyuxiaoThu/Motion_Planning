#include "GlobalSettings.h"
#include "Astar.h"
#include <math.h>

Grid_3D ConvertConfigToIndex(qp_real x, qp_real y, qp_real t,Environment_Scale_ environment_scale_,XYT_Graph_Search_ xyt_graph_search_){
    
    qp_int ind1 = round((x - environment_scale_.environment_x_min) / xyt_graph_search_.resolution_x) + 1;

    qp_int ind2 = round((y - environment_scale_.environment_y_min) / xyt_graph_search_.resolution_y) + 1;

    qp_int ind3 = round(t / xyt_graph_search_.resolution_t) + 1;

    if(ind1 < 1){
        ind1 = 1;
    }else if (ind1 > xyt_graph_search_.num_nodes_x)
    {
        ind1 = xyt_graph_search_.num_nodes_x;
    }

    if(ind2 < 1){
        ind2 = 1;
    }else if (ind2 > xyt_graph_search_.num_nodes_y)
    {
        ind2 = xyt_graph_search_.num_nodes_y;
    }

    if(ind3 < 1){
        ind3 = 1;
    }else if (ind3 > xyt_graph_search_.num_nodes_t)
    {
        ind3 = xyt_graph_search_.num_nodes_t;
    }

    Grid_3D out_ind = {ind1,ind2,ind3};
    

    return out_ind;
}