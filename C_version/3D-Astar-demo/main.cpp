#include <stdio.h>
#include <math.h>
//#include "GlobalSettings.h"
#include "Astar.h"


#define PI acos(-1)

#include <iostream> // for output and test

using namespace std; // delete at end

Grid_3D ConvertConfigToIndex(qp_real x, qp_real y, qp_real t,Environment_Scale_ environment_scale_,XYT_Graph_Search_ xyt_graph_search_);


int main(){

    //initilize the parameters
    Vehicle_Geometrics_ vehicle_geometrics_ = {2.8,0.96,0.929,1.942};

    Vehicle_Kinematics vehicle_kinematics_= {25,3,0.7,0.1,to_be_deter,to_be_deter};
    vehicle_kinematics_.vehicle_kappa_max = tan(vehicle_kinematics_.vehicle_phy_max) / vehicle_geometrics_.vehicle_wheelbase;
    vehicle_kinematics_.vehicle_turning_radius_min = 1.0 / vehicle_kinematics_.vehicle_kappa_max;

    Environment_Scale_ environment_scale_ = {-5,60,-2,5,to_be_deter,to_be_deter};

    environment_scale_.x_scale = environment_scale_.environment_x_max - environment_scale_.environment_x_min;

    environment_scale_.y_scale = environment_scale_.environment_y_max - environment_scale_.environment_y_min;


    XYT_Graph_Search_ xyt_graph_search_ = {6,Num_t_nodes_cons,to_be_deter,to_be_deter,to_be_deter,to_be_deter,to_be_deter,1.0,2.0,20000,4,0,0,0,0};

    xyt_graph_search_.resolution_t= xyt_graph_search_.max_t / (xyt_graph_search_.num_nodes_t - 1);
    xyt_graph_search_.resolution_x= xyt_graph_search_.resolution_t * vehicle_kinematics_.vehicle_v_max / (3 * 1.414);
    xyt_graph_search_.resolution_y= xyt_graph_search_.resolution_x;
    xyt_graph_search_.num_nodes_x = ceil(environment_scale_.x_scale / xyt_graph_search_.resolution_x);
    xyt_graph_search_.num_nodes_y = ceil(environment_scale_.y_scale / xyt_graph_search_.resolution_y);

    // initilize the array

    xyt_graph_search_.obs_vx[0] = 12.0;
    xyt_graph_search_.obs_vx[1] = 18.0;
    xyt_graph_search_.obs_vx[2] = 12.0;
    xyt_graph_search_.obs_vx[3] = 18.0;

    xyt_graph_search_.obs_vy[0] = -0.5;
    xyt_graph_search_.obs_vy[1] = -0.2;
    xyt_graph_search_.obs_vy[2] = -0.2;
    xyt_graph_search_.obs_vy[3] = -0.5;

    xyt_graph_search_.obs_x0[0] = 20.0;
    xyt_graph_search_.obs_x0[1] = 5.0;
    xyt_graph_search_.obs_x0[2] = 5.0;
    xyt_graph_search_.obs_x0[3] = 20.0;

    xyt_graph_search_.obs_y0[0] = 0.0;
    xyt_graph_search_.obs_y0[1] = 4.0;
    xyt_graph_search_.obs_y0[2] = 1.0;
    xyt_graph_search_.obs_y0[3] = 3.5;
    
    Vehicle_TPBV_ vehicle_TPBV_ = {0,1.5,-PI,40,4.5,-PI};

    // Generating dynamic_obs matrix --- begin
    qp_real obs_x[Num_t_nodes_cons][4][Num_obs];
    qp_real obs_y[Num_t_nodes_cons][4][Num_obs];
    
    qp_real len_veh = vehicle_geometrics_.vehicle_wheelbase/2;
    qp_real width_veh = vehicle_geometrics_.vehicle_width/2;

    for (int ii=0; ii<Num_obs; ii++ ){
        qp_real dx = xyt_graph_search_.obs_vx[ii];
        qp_real dy = xyt_graph_search_.obs_vy[ii];
        qp_real x_pos0[4] = {xyt_graph_search_.obs_x0[ii]-len_veh,xyt_graph_search_.obs_x0[ii]-len_veh,xyt_graph_search_.obs_x0[ii]+len_veh,xyt_graph_search_.obs_x0[ii]+len_veh};
        qp_real y_pos0[4] = {xyt_graph_search_.obs_y0[ii]-width_veh,xyt_graph_search_.obs_y0[ii]-width_veh,xyt_graph_search_.obs_y0[ii]+width_veh,xyt_graph_search_.obs_y0[ii]+width_veh};

        for(int jj=0; jj<Num_t_nodes_cons; jj++){
            obs_x[jj][0][ii] = x_pos0[0] + dx / xyt_graph_search_.num_nodes_t * (jj - 1);
            obs_x[jj][1][ii] = x_pos0[1] + dx / xyt_graph_search_.num_nodes_t * (jj - 1);
            obs_x[jj][2][ii] = x_pos0[2] + dx / xyt_graph_search_.num_nodes_t * (jj - 1);
            obs_x[jj][3][ii] = x_pos0[3] + dx / xyt_graph_search_.num_nodes_t * (jj - 1);

            obs_y[jj][0][ii] = y_pos0[0] + dy / xyt_graph_search_.num_nodes_t * (jj - 1);

            obs_y[jj][1][ii] = y_pos0[1] + dy / xyt_graph_search_.num_nodes_t * (jj - 1);

            obs_y[jj][2][ii] = y_pos0[2] + dy / xyt_graph_search_.num_nodes_t * (jj - 1);

            obs_y[jj][3][ii] = y_pos0[3] + dy / xyt_graph_search_.num_nodes_t * (jj - 1);

        }
    }

    // --- finish

    Grid_3D start_ind = ConvertConfigToIndex(vehicle_TPBV_.x0, vehicle_TPBV_.y0, 0,environment_scale_,xyt_graph_search_);
    Grid_3D goal_ind = ConvertConfigToIndex(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, xyt_graph_search_.max_t,environment_scale_,xyt_graph_search_);

    Vehicle_Traj Grid_res = SearchViaAStar(start_ind, goal_ind,xyt_graph_search_,environment_scale_,vehicle_TPBV_);

    qp_real xx[Num_t_nodes_cons];
    qp_real yy[Num_t_nodes_cons];

    for (int i=0; i< Num_t_nodes_cons; i++){
        xx[i] = environment_scale_.environment_x_min + (Grid_res.ind_vec[i][0] - 1) * xyt_graph_search_.resolution_x;

        yy[i] = environment_scale_.environment_y_min + (Grid_res.ind_vec[i][1] - 1) * xyt_graph_search_.resolution_y;
    }


    // print test
    
    //cout<< "goal_end" << goal_ind.x << goal_ind.y << goal_ind.t << endl;
    cout<< "start_end" << start_ind.x << start_ind.y << start_ind.t << endl;
    

    return 0;
}

