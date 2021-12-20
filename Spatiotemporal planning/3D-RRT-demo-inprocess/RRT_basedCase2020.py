# 3D search
import random
import numpy as np
import math
#from RRTstar import *

class Problem_para:

    
    # Geometric size of the ego vehicle and the surrouding social vehicles (assumed to be identical)
    # Vehicle kinematics
    def __init__(self,wheelbase,front_hang,rear_hang,width,v_max_long,v_suggested_long,timefinish,max_iteration,s_start,l_start,l_timefinish,L_norminal,velocity_start,acceleration_start,late_v_start,late_a_start,lat_sample_seeds,prob_long,prob_lat,dis_near,weight_inside_exp,weight_for_collision_risks,weight_for_drastic_long_vel_change,weight_for_drastic_lat_vel_change,weight_for_biased_long,weight_for_biased_lat,Nfe,num_obs,agv_velocity,road_left_bound,road_right_bound,st_relation_list,lt_relation_list):
        self.vehicle_wheelbase = wheelbase
        self.vehicle_front_hang = front_hang
        self.vehicle_rear_hang = rear_hang
        self.vehicle_width = width
        self.vehicle_length = self.vehicle_wheelbase + self.vehicle_front_hang + self.vehicle_rear_hang
        self.vehicle_v_max_long = v_max_long
        self.vehicle_v_suggested_long = v_suggested_long

    # Problem 

        self.tf = timefinish
        self.Nmax_iter = max_iteration
        self.s0 = s_start
        self.l0 = l_start
        self.ltf = l_timefinish
        self.L_norm = L_norminal
        self.s_max = self.s0 + self.vehicle_v_max_long * self.tf
        self.ds0 = velocity_start
        self.dds0 = acceleration_start
        self.dl0 = late_v_start
        self.ddl0 = late_a_start
        self.Nlat_samples = lat_sample_seeds
        self.prob_long = prob_long # ?
        self.prob_lat = prob_lat #?
        self.distance_near = dis_near
    
    # QP settings
        self.w_inside_exp = weight_inside_exp
        self.w_collision_risk = weight_for_collision_risks
        self.w_long_vel_change = weight_for_drastic_long_vel_change
        self.w_lat_vel_change = weight_for_drastic_lat_vel_change
        self.w_biased_long = weight_for_biased_long
        self.w_biased_lat = weight_for_biased_lat
        self.Nfe = Nfe
    # Obstacles
        self.num = num_obs
        self.obs_agv_vel = agv_velocity
        self.road_lb = road_left_bound
        self.road_rb = road_right_bound
        self.ob_st = st_relation_list
        self.ob_lt = lt_relation_list # eg:s(t) = 18*t + 12;
        self.center_right_bar = self.road_lb - self.vehicle_width*0.5
        self.center_left_bar = self.road_rb + self.vehicle_width*0.5

        # rrt nodes
        #self.node_list_slt = np.array([])
        #self.node_list = [] # a list
        self.node_num = 0


    def SpecifyLongValueRandomly(self,time):
        rand_prob1 = random.uniform(0,1.0)
        if rand_prob1 < self.prob_long:
            rand_prob2 =  random.uniform(0,1.0)
            v_avg = self.vehicle_v_max_long * rand_prob2
        else:
            v_avg = self.vehicle_v_suggested_long
        
        s = v_avg*time

        return s

    def IsV1CollidingWithV2(self,s_ego,l_ego,s_other,l_other):
        veh_half_width = self.vehicle_width*0.5
        v1_xmin = s_ego - self.vehicle_rear_hang
        v1_xmax = s_ego + self.vehicle_front_hang + self.vehicle_wheelbase
        v1_ymin = l_ego - veh_half_width
        v1_ymax = l_ego + veh_half_width

        v2_xmin = s_other - self.vehicle_rear_hang
        v2_xmax = s_other + self.vehicle_front_hang + self.vehicle_wheelbase
        v2_ymin = l_other - veh_half_width
        v2_ymax = l_other + veh_half_width

        if ((v2_ymax < v1_ymin) or (v1_ymax < v2_ymin) or (v2_xmax < v1_xmin) or (v1_xmax < v2_xmin)):
            val = 0
        else:
            val = 1

        return val

    def SpecifyLatValueRandomly(self,s_ego,time):
        #l_val = -999
        obs_ss = []
        obs_ll = []
        
        for i in range(0,self.num):
            obs_s = self.ob_st[i][0]*time + self.ob_st[i][1]

            if (abs(s_ego-obs_s) > self.vehicle_length):
                continue

            obs_l = self.ob_lt[i][0]*time + self.ob_lt[i][1]

            obs_ss.append(obs_s)
            obs_ll.append(obs_l)

        for i in range(0,self.Nlat_samples):
            rand_prob1 = random.uniform(0,1.0)
            if (rand_prob1 < self.prob_lat):
                l_val = self.L_norm
            else:
                rand_prob2 = random.uniform(0,1.0)
                l_val = self.center_right_bar + (self.center_left_bar - self.center_right_bar) * rand_prob2

            is_ready = 1

            for ii in range(0,len(obs_ss)):
                if (Problem_para.IsV1CollidingWithV2(self,s_ego,l_val,obs_ss[ii],obs_ll[ii])):
                    is_ready = 0
                    break

            if(is_ready == 1):
                return l_val


        l_val = -999
        return l_val

    def FindParentAndChild(self,q_new_slt,node_list):
        num_nodes = len(node_list)
        node_list_slt_here = []
        for i in range(0,num_nodes):
            node_list_slt_here.append(node_list[i]['slt'])

        #self.node_list_slt = node_list_slt_here
        node_list_slt_here = np.array(node_list_slt_here)

        mat2 = []
        for ii in range(0,num_nodes):

            mat = np.square(node_list_slt_here[ii] - q_new_slt)
            mat2.append(sum(mat)) 
            #mat2.append(np.sum(mat))
            #mat2 = np.square(mat)
            #error_vec_old = np.transpose(np.sum(np.transpose(mat2),axis = -1))  
        #mat2 = np.array(mat2)
        #print('Q',len(mat2))
        error_vec_old = np.array(mat2)#np.sum(mat2,axis = 1)#np.array(mat2)
        #print('error',len(error_vec_old))
        error_vec = np.sort(error_vec_old,axis = -1) # ?
        #print('error_new',len(error_vec))
        index_sequence = np.argsort(error_vec_old)   
        q_nearest_underlying_parent_id = []
        q_nearest_underlying_child_id = []
        
        cur_time = q_new_slt[2]
        threshold = self.distance_near**2

        for iii in range(0,num_nodes):
            if (error_vec[iii] > threshold):
                return [q_nearest_underlying_parent_id,q_nearest_underlying_child_id]
            ind = index_sequence[iii]
            cur_node = node_list[ind]
            if ((cur_node['slt'][2] > cur_time) and (cur_node['slt'][0] > q_new_slt[0])):
                q_nearest_underlying_child_id.append(ind)
            if ((cur_node['slt'][2] < cur_time) and (cur_node['slt'][0] < q_new_slt[0])):
                q_nearest_underlying_parent_id.append(ind)
        
        return [q_nearest_underlying_parent_id,q_nearest_underlying_child_id]

    def ComputeDistance(self,nodea,nodeb):
        init_velocity_long = nodea['ds']
        init_velocity_lat = nodea['dl']

        lon_dis = nodeb['slt'][0] - nodea['slt'][0]
        lat_dis = nodeb['slt'][1] - nodea['slt'][1]

        dt = nodeb['slt'][2] - nodea['slt'][2] 

        avg_velocity_long = lon_dis/dt
        avg_velocity_lat = lat_dis/dt

        num_obs = self.num
        num_samples = round(dt/0.2) + 2 # ?
        timeline = np.linspace(nodea['slt'][2],nodeb['slt'][2],num_samples)
        sline = np.linspace(nodea['slt'][0],nodeb['slt'][0],num_samples)
        lline = np.linspace(nodea['slt'][1],nodeb['slt'][1],num_samples)

        cost_for_collision = 0

        for i in range(0,num_obs):
            obs_st = self.ob_st[i]
            obs_lt = self.ob_lt[i]
            for j in range(0,num_samples):
                time = timeline[j]
                s_ego = sline[j]
                l_ego = lline[j]
                s_other = obs_st[0]*time + obs_st[1]
                l_other = obs_lt[0]*time + obs_lt[1]
                distance = math.hypot(s_ego-s_other,l_ego-l_other)
                cost_for_collision = cost_for_collision + math.exp(-self.w_inside_exp * (distance**2))
        cost_for_collision = cost_for_collision/(num_obs * num_samples)

        out_res = self.w_collision_risk * cost_for_collision + self.w_long_vel_change *(abs(init_velocity_long-avg_velocity_long)/dt) + self.w_lat_vel_change*(abs(init_velocity_lat-avg_velocity_lat)/dt) + self.w_biased_long*abs(avg_velocity_long-self.vehicle_v_suggested_long) + self.w_biased_lat*abs(0.5*(nodeb['slt'][1]+nodea['slt'][1]) - self.L_norm)/(self.center_left_bar - self.L_norm)

        return out_res

    def IsSteerCollisionFree(self,a, b):
        is_valid = 0
        lat_dis = b[1] - a[1]
        lon_dis = b[0] - a[0]
        dt = b[2] - a[2]

        avg_velocity_long = lon_dis / dt
        avg_velocity_lat = lat_dis / dt

        if(math.hypot(avg_velocity_long,avg_velocity_lat) > self.vehicle_v_max_long):
            return is_valid
        
        if(abs(avg_velocity_lat/avg_velocity_long)>0.5):
            return is_valid
        
        num_samples = round(abs(b[2]-a[2])/0.1) + 2
        sline = np.linspace(a[0],b[0],num_samples)
        lline = np.linspace(a[1],b[1],num_samples)
        timeline = np.linspace(a[2],b[2],num_samples)

        for i in range(0,self.num):
            obs_st = self.ob_st[i]
            obs_lt = self.ob_lt[i]
            for j in range(0,num_samples):
                time = timeline[j]
                s_ego = sline[j]
                l_ego = lline[j]
                s_other = obs_st[0]*time + obs_st[1]
                l_other = obs_lt[0]*time + obs_lt[1]
                if (abs(s_other-s_ego) >= self.vehicle_length):
                    continue
                else:
                    if(Problem_para.IsV1CollidingWithV2(self,s_ego,l_ego,s_other,l_other)):
                        return is_valid
        is_valid = 1
        return is_valid

    def MeasureDistancePure(self,nodea, nodeb):
        lon_dis = nodeb['slt'][0] - nodeb['slt'][1]
        lat_dis = nodeb['slt'][1] - nodea['slt'][1]
        time_dis = nodeb['slt'][2] - nodea['slt'][2]

        dis3dim = (abs(lon_dis)/(self.s_max - self.s0)) + (abs(time_dis)/self.tf) + (abs(lat_dis - self.L_norm)/(self.road_lb - self.road_rb))

        return dis3dim

    def SearchCoarseTrajectoryViaRRTStar(self):
        # main function
        #q_init = [{'slt':np.array([self.s0,self.l0,0.0]),'cost':0.0,'parentid':-999,'ds':self.ds0,'dl':self.dl0}] # cost is cost to come
        # -999 represents NULL


        node_list_slt = np.array([self.s0,self.l0,0.0])
        node_list = [{'slt':np.array([self.s0,self.l0,0.0]),'cost':0.0,'parentid':-999,'ds':self.ds0,'dl':self.dl0}]
        self.node_num = 1 # initial

        #q_goal_slt = [self.s_max,self.ltf,self.tf]
        # This is the nominal goal node, which is not necessarily reachable.
        q_best_cost = np.Inf
        

        q_goal = {'slt':np.array([self.s_max,self.ltf,self.tf]),'cost':0.0,'parentid':-999,'ds':self.ds0,'dl':self.dl0}
        #q_goal['slt'] = [self.s_max,self.ltf,self.tf]
        #q_new_slt = q_init['slt']

        time_sampling_val_index = np.random.permutation(self.Nmax_iter) 

        q_best_id = -999

        #q_new = {'slt':np.array([0.0,0.0,0.0]),'cost':0.0,'parentid':-999,'ds':0,'dl':0}

        for iter in range(0,self.Nmax_iter):
            
            q_new = {'slt':np.array([0.0,0.0,0.0]),'cost':0.0,'parentid':-999,'ds':0,'dl':0}
            
            q_new['slt'][2] = self.tf*time_sampling_val_index[iter]/self.Nmax_iter
            q_new['slt'][0] = Problem_para.SpecifyLongValueRandomly(self,q_new['slt'][2])
            q_new['slt'][1] = Problem_para.SpecifyLatValueRandomly(self,q_new['slt'][0],q_new['slt'][2])

            if(q_new['slt'][1] == -999):
                continue
            #print('q_new',q_new['slt'])
            [q_nearest_underlying_parent_id, q_nearest_underlying_child_id] = Problem_para.FindParentAndChild(self,q_new['slt'],node_list) # something is wrong! always zero

            

            parent_current_best_cost_value = np.Inf
            

            for k in range(0,len(q_nearest_underlying_parent_id)):
                current_candidate_parent = node_list[q_nearest_underlying_parent_id[k]]

                candidate_cost = current_candidate_parent['cost'] + Problem_para.ComputeDistance(self,current_candidate_parent, q_new)

                if ((candidate_cost < parent_current_best_cost_value) and (Problem_para.IsSteerCollisionFree(self,current_candidate_parent['slt'],q_new['slt']))):
                    parent_current_best_cost_value = candidate_cost
                    q_new['parentid'] = q_nearest_underlying_parent_id[k]
                    q_new['cost'] = parent_current_best_cost_value
                
            if(q_new['parentid'] == -999):
                continue
            
            dt = q_new['slt'][2] - node_list[q_new['parentid']]['slt'][2]

            q_new['ds'] = (q_new['slt'][0] - node_list[q_new['parentid']]['slt'][0])/dt

            q_new['dl'] = (q_new['slt'][1] - node_list[q_new['parentid']]['slt'][1])/dt
            #print('check q_new',q_new)
            node_list.append(q_new)
            self.node_num = self.node_num + 1
            node_list_slt = np.append(node_list_slt,q_new['slt'])

            #Rewire

            for kk in range(0,len(q_nearest_underlying_child_id)):
                id = q_nearest_underlying_child_id[kk]
                current_candidate_child = node_list[id]

                candidate_cost = q_new['cost'] + Problem_para.ComputeDistance(self,q_new,current_candidate_child)

                if (candidate_cost < current_candidate_child['cost']) and (Problem_para.IsSteerCollisionFree(self,q_new['slt'],current_candidate_child['slt'])):
                    node_list[id]['parentid'] = len(node_list) # Since q_new is the last one added in the node_list, the length is also its ID.

                    node_list[id]['cost'] =  candidate_cost

                    dtt = node_list[id]['slt'][2] - q_new['slt'][2]

                    node_list[id]['ds'] = (node_list[id]['slt'][0] - q_new['slt'][0])/dtt

                    node_list[id]['dl'] = (node_list[id]['slt'][1] - q_new['slt'][1])/dtt
                
            # update current best node
            q_best_cost_candidate = Problem_para.MeasureDistancePure(self,q_new,q_goal)

            if (q_best_cost_candidate < q_best_cost):
                q_best_cost = q_best_cost_candidate
                q_best_id = len(node_list)
            
        path = []

        parent_id_here = q_best_id
        while(parent_id_here != -999):
            print('ok')
            print('add this',node_list[parent_id_here]['slt'])
            print('len nodelist',len(node_list))
            path.append(node_list[parent_id_here]['slt'])
            parent_id_here = node_list[parent_id_here]['parentid']
        
        return path


problem_para = Problem_para(2.8,0.96,0.929,1.942,20.0,15.0,8.0,500,0,1.75,1.75,1.75,10,0,0.1,0,50,0.8,0.2,50,2,5,10,20,1,1,200,3,10,7,0,[[15.0,-15.0],[12.0,10.0],[18.0,12.0]],[[5.0,0.0],[0.8,-1.6],[1.2,0.0]])


path_coarse = problem_para.SearchCoarseTrajectoryViaRRTStar()

print(path_coarse)

#print(dir(problem_para))

#print(vars(obs_env))

# Coarse trajectory planning via spatio-temporal RRT*
#global coarse_trajectory precise_trajectory
#coarse_trajectory = SearchCoarseTrajectoryViaRRTStar();
#precise_trajectory = OptimizeTrajectory(coarse_trajectory);

#coarse_trajectory = SearchCoarseTrajectoryViaRRTStar(param_vehicle)
