#pragma once

#define qp_real double
#define qp_int int
#define Num_obs 4 // max 4 vehicles
#define Num_t_nodes_cons 31
#define Num_x_nodes_cons 56 // !!!
#define Num_y_nodes_cons 6
#define MAX_NODE_COUNT (Num_t_nodes_cons*Num_x_nodes_cons*Num_y_nodes_cons)
#define to_be_deter 0 // to be changed

typedef struct{
    qp_real vehicle_wheelbase;
    qp_real vehicle_front_hang;
    qp_real vehicle_rear_hang;
    qp_real vehicle_width;
}Vehicle_Geometrics_;

typedef struct{
    qp_real vehicle_v_max;
    qp_real vehicle_a_max;
    qp_real vehicle_phy_max;
    qp_real vehicle_w_max;
    qp_real vehicle_kappa_max; //= tan(vehicle_phy_max / vehicle_wheelbase;
    qp_real vehicle_turning_radius_min; //= 1 / vehicle_kinematics_.vehicle_kappa_max;
}Vehicle_Kinematics;
    
typedef struct{
    qp_real environment_x_min;
    qp_real environment_x_max;
    qp_real environment_y_min;
    qp_real environment_y_max;
    qp_real x_scale; //= environment_scale_.environment_x_max - environment_scale_.environment_x_min;
    qp_real y_scale; //= environment_scale_.environment_y_max - environment_scale_.environment_y_min; 
}Environment_Scale_;

typedef struct{
    qp_real max_t;
    qp_real num_nodes_t;
    qp_real resolution_t; //= xyt_graph_search_.max_t / (xyt_graph_search_.num_nodes_t - 1);
    qp_real resolution_x; //= xyt_graph_search_.resolution_t * vehicle_kinematics_.vehicle_v_max / (3 * 1.414);
    qp_real resolution_y; //= xyt_graph_search_.resolution_x;
    qp_real num_nodes_x; // = ceil(environment_scale_.x_scale / xyt_graph_search_.resolution_x);
    qp_real num_nodes_y;// = ceil(environment_scale_.y_scale / xyt_graph_search_.resolution_y);
    qp_real multiplier_H_for_A_star;
    qp_real weight_for_time;
    qp_real max_iter;
    qp_real num_otherveh;
    qp_real obs_vx[Num_obs]; // max 4 vehicles
    qp_real obs_vy[Num_obs];
    qp_real obs_x0[Num_obs];
    qp_real obs_y0[Num_obs];
}XYT_Graph_Search_;

typedef struct{
    qp_real x0;
    qp_real y0;
    qp_real theta0;
    qp_real xtf;
    qp_real ytf;
    qp_real thetaf;
}Vehicle_TPBV_;

typedef struct{
    qp_int x;
    qp_int y;
    qp_int t;
}Grid_3D;

typedef struct{
    qp_int ind_vec[Num_t_nodes_cons][3];
    qp_real theta[Num_t_nodes_cons];
    qp_real fitness;
}Vehicle_Traj;

//functions
Vehicle_Traj SearchViaAStar( Grid_3D start_ind, Grid_3D end_ind,XYT_Graph_Search_ xyt_graph_search_, Environment_Scale_ environment_scale_,Vehicle_TPBV_ vehicle_TPBV_);

