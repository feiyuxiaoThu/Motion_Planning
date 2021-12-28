% QP_matlab
function [x1,x2,x3,x4,x5] = QP_matlab(coarse_trejectory,Boxes)
global param_

Nstep = param_.Nfe;

[x1,x2,x3,x4,x5] = constraint_gen(Nstep,Boxes);

%QP_trajectory = coarse_trejectory;
end

function [A_out,u_s_out,l_s_out,Aeq_out,beq_out] = constraint_gen(nstep,Boxes)
global param_
% parameters

dt = param_.tf/(nstep-1);
v_lat0 = 0;
v_lon0 = 15;
a_maxlon = 2;
a_maxlat = 1;

lat_jerk_limit = 0.5*dt;
lon_jerk_limit = 0.5*dt;

%% l lateral

Eye_s = cat(2,eye(nstep),zeros(nstep,2*nstep));
Eye_ss = cat(2,zeros(nstep,2*nstep),eye(nstep));

A_2line = zeros(nstep-1,nstep);

for i = 1: nstep-1
    A_2line(i,i) = -1;
    A_2line(i,i+1) = 1;
end

A1 = cat(2,zeros(nstep-1,2*nstep),A_2line);

A_t1 = zeros(nstep-1,nstep);

for i = 1:nstep-1
    A_t1(i,i) = -dt/2;
    A_t1(i,i+1) = -dt/2;
end

A2 = cat(2,zeros(nstep-1,nstep),A_2line,A_t1);

A_t2 = zeros(nstep-1,nstep);

for i = 1:nstep-1
    A_t2(i,i) = -dt*dt/3;
    A_t2(i,i+1) = -dt*dt/6;
end

A3 = cat(2,A_2line,-dt*eye(nstep-1,nstep),A_t2);

A_ini = zeros(3,3*nstep);

A_ini(1,1) = 1;
A_ini(2,nstep+1) = 1;
A_ini(3,2*nstep+1) = 1;

A = cat(1,Eye_s,Eye_ss,A1);
Aeq = cat(1,A2,A3,A_ini);

% generate u and l for l


size_A = size(A,1);
u_s = zeros(size_A,1);
l_s = zeros(size_A,1);

for i = 1:nstep
    % s.t. attraction_x {i in {1..Nw}}:
    %Boxes[i,1] <= x[3*i-2] <= Boxes[i,2];
    u_s(i) = Boxes(4,i);
    l_s(i) = Boxes(3,i);
end

for i = nstep+1:2*nstep % acceleration limit
    u_s(i) = a_maxlat;
    l_s(i) = -a_maxlat;
end


for i = 2*nstep+1:3*nstep-1 % jlat erk limit
    u_s(i) = lat_jerk_limit;
    l_s(i) = -lat_jerk_limit;
end

size_Aeq = size(Aeq,1);
beq = zeros(size_Aeq,1);
beq(size_Aeq-1) = v_lat0;
beq(size_Aeq-2) = 1.75;





%% s longtual
% same A and Aeq
uu_s = zeros(size_A,1);
ll_s = zeros(size_A,1);

for i = 1:nstep
    %s.t. attraction_y {i in {1..Nw}}:
    %Boxes[i,3] <= y[3*i-2] <= Boxes[i,4];
    uu_s(i) = Boxes(2,i);
    ll_s(i) = Boxes(1,i);
end

for i = nstep+1:2*nstep % acceleration limit
    uu_s(i) = a_maxlon;
    ll_s(i) = -a_maxlon;
end


for i = 2*nstep+1:3*nstep-1 % jlat erk limit
    uu_s(i) = lon_jerk_limit;
    ll_s(i) = -lon_jerk_limit;
end

%size_Aeq = size(Aeq,1);
bbeq = zeros(size_Aeq,1);
bbeq(size_Aeq-1) = v_lon0;

A_out = blkdiag(A,A);
Aeq_out = blkdiag(Aeq,Aeq);
u_s_out = cat(1,u_s,uu_s);
l_s_out = cat(1,l_s,ll_s);
beq_out = cat(1,beq,bbeq);



end
