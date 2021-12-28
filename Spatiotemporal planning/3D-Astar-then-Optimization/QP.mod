param NFE0{i in {1}};
param Nw = NFE0[1];
param Nfe = 3 * Nw;
param Waypoints{i in {1..Nw}, j in {1..2}};
param Boxes{i in {1..Nw}, j in {1..4}};

var x{i in {1..Nfe}};
var dx{i in {1..Nfe}};
var ddx{i in {1..Nfe}};
var dddx{i in {1..Nfe}};
var y{i in {1..Nfe}};
var dy{i in {1..Nfe}};
var ddy{i in {1..Nfe}};
var dddy{i in {1..Nfe}};

minimize cost_function:
sum{i in {1..Nfe}}(dy[i]^2 + dx[i]^2 + ddy[i]^2 + ddx[i]^2 + 0*dddy[i]^2 + 0*dddx[i]^2) + 0.01 * sum{i in {1..Nw}}((x[3*i-2] - Waypoints[i,1])^2 + (y[3*i-2] - Waypoints[i,2])^2);

## Constraints ##
s.t. intgrate_dddx {i in {2..Nfe}}:
ddx[i] = ddx[i-1] + dddx[i];

s.t. intgrate_dddy {i in {2..Nfe}}:
ddy[i] = ddy[i-1] + dddy[i];

s.t. intgrate_ddx {i in {2..Nfe}}:
dx[i] = dx[i-1] + ddx[i-1] + 0.5 * dddx[i];

s.t. intgrate_ddy {i in {2..Nfe}}:
dy[i] = dy[i-1] + ddy[i-1] + 0.5 * dddy[i];

s.t. intgrate_dx {i in {2..Nfe}}:
x[i] = x[i-1] + dx[i-1] + 0.5 * ddx[i-1] + dddx[i] / 6;

s.t. intgrate_dy {i in {2..Nfe}}:
y[i] = y[i-1] + dy[i-1] + 0.5 * ddy[i-1] + dddy[i] / 6;

s.t. attraction_x {i in {1..Nw}}:
Boxes[i,1] <= x[3*i-2] <= Boxes[i,2];

s.t. attraction_y {i in {1..Nw}}:
Boxes[i,3] <= y[3*i-2] <= Boxes[i,4];

s.t. init_x:
x[1] = 0;

s.t. init_y:
y[1] = 1.75;

s.t. init_dx:
dx[1] = 0;

s.t. init_dy:
dy[1] = 0;

s.t. init_ddx:
ddx[1] = 0;

s.t. init_ddy:
ddy[1] = 0;


data;
param NFE0:= include NFE0;
param Waypoints:= include Waypoints;
param Boxes:= include Boxes;