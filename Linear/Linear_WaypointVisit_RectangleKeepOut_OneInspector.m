yalmip('clear');
clear all
clc

%% Build discrete state space matricies 
SampleTime = 60; % (secs)
n = 7.291e-5; % (rad/sec)
m = 1000;

A = [0,     0,      0,      1,      0,      0;
     0,     0,      0,      0,      1,      0;
     0,     0,      0,      0,      0,      1;
    3*n^2,  0,      0,      0,      2*n,    0;
     0,     0,      0,      -2*n,   0,      0;
     0,     0,      -n^2,   0,      0,      0];
 
 B = [0 0 0;
      0 0 0;
      0 0 0;
      1 0 0;
      0 1 0;
      0 0 1];
 
C = [1, 1,  1,  1,  1,  1];
 
sys = ss(A,B,C,0);
 
d_sys = c2d(sys,SampleTime);

A = d_sys.A;
B = d_sys.B;


%% Define decision variables
T = 1*60*60/SampleTime; % number of discrete time steps
nx = 6; % number of states
nu = 3; % number of control inputs

u = sdpvar(nu,T-1); % state decision variables
x = sdpvar(nx,T); % control decision variables
h = binvar(T-1,3); % 5 integer decision variables
c = binvar(T-1,6);

%% Set bounds and build scaling matrix
Mp = 10*1000;
Mv = 1*1000;
Mu = 1*1000;

Sx = [Mp     0     0       0       0       0;
      0      Mp    0       0       0       0;
      0      0     Mp      0       0       0;
      0      0     0       Mv      0       0;
      0      0     0       0       Mv      0;
      0      0     0       0       0       Mv];
  
Su = [Mu     0       0;
      0      Mu      0;
      0      0       Mu];
 

%% Waypoint locations
xw(1:6,1) = [500;500;500;0;0;0];
xw(1:6,2) = [-500;-500;-500;0;0;0];
xw(1:6,3) = [500;500;-500;0;0;0];

%% Keep out zone corners
xmin = -450;
xmax = 450;
ymin = -450;
ymax = 450;
zmin = -450;
zmax = 450;

%% Constrain initial and final location of inspector
constraints = [x(1,1) == 0
               x(2,1) == 5*1000/Mp
               x(3,1) == 0
               x(4,1) == 0
               x(5,1) == 0
               x(6,1) == 0
               x(1,T) == 0
               x(2,T) == 5*1000/Mp
               x(3,T) == 0
               x(4,T) == 0
               x(5,T) == 0
               x(6,T) == 0];
           

%% Define all other constraints
constraints = [constraints, -1 <= x(1:3,:) <= 1];
constraints = [constraints, -1 <= x(4:6,:) <= 1];
constraints = [constraints, -1 <= u(1:3,:) <= 1];

objective = 0;
for i = 1:T-1
    
    constraints = [constraints, Sx*x(1:6,i+1) == Sx*A*x(1:6,i) + B*u(1:3,i)];
             
    constraints = [constraints
                   implies(h(i,1), Sx*x(1:6,i) == xw(1:6,1))
                   implies(h(i,2), Sx*x(1:6,i) == xw(1:6,2))
                   implies(h(i,3), Sx*x(1:6,i) == xw(1:6,3))];
               
    constraints = [constraints
                   Mp*x(1,i) <= xmin + c(i,1)*Mp
                   -Mp*x(1,i) <= -xmax + c(i,2)*Mp
                   Mp*x(2,i) <= ymin + c(i,3)*Mp
                   -Mp*x(2,i) <= -ymax + c(i,4)*Mp
                   Mp*x(3,i) <= zmin + c(i,5)*Mp
                   -Mp*x(3,i) <= -zmax + c(i,6)*Mp];
               
    constraints = [constraints, sum(c(i,:)) <= 5];

    constraints = [constraints, sum(h(i,:)) <= 1];
    
    objective = objective + sum(abs(u(1:3,i)));
                                   
end

constraints = [constraints, sum(h(:,1)) == 1];
constraints = [constraints, sum(h(:,2)) == 1];
constraints = [constraints, sum(h(:,3)) == 1];


%% Find solution
% options = sdpsettings('solver','cplex','cplex.mip.strategy.branch',1,'cplex.mip.tolerances.mipgap',0.2);
options = sdpsettings('solver','glpk');
optimize(constraints,objective,options)

%% Plot solution
for i = 1:T
    sol(1:6,i) = Sx*value(x(1:6,i));
end

for i = 1:T-1
    control(1:3,i) = value(u(1:3,i));
    hval(i,:) = value(h(i,:));
end

figure(1)
plot3(sol(2,:),sol(1,:),sol(3,:),'-ok')
axis equal
hold on

P = [0,0,0] ;   % you center point 
L = [900,900,900] ;  % your cube dimensions 
C = P-L/2 ;       % Get the origin of cube so that P is at center 
plotcube(L,C,.5,[1 0 0]);   % use function plotcube

figure(2)
plot(1:T,sol(1,:),1:T,sol(2,:))

figure(3)
scatter(2:T,control(1,:))
hold on
scatter(2:T,control(2,:))
hold on
scatter(2:T,control(3,:))

