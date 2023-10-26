clc; clear; close all;

f   = @(t,x) [-x(2); x(1) + (x(1)^2-1)*x(2)];

plotpp(f,'xlim',[-3,3],'ylim',[-3,3],'linecolor',0.3*[1 1 1]); hold on;

% linearize the dynamics around the origin to get xdot = Ax.
A   = [0 -1; 1 -1];
Q   = eye(2);

% Solve the Lyapunov equation to get the PSD matrix P
% and the Lyapunov function V(x) = x'Px for the linearized system
P   = lyap(A',Q);

% A conservatuive approximation of ROA can be obtained using the
% Lyapunov function V(x) derived in the linear analysis
% We consider the level sets L(rho) = {x: V(x) <= rho}, and we look
% for the maximum value of rho such that Vdot(x) = 2x'Pf(x)<0,
% for all x in L(rho)\{0}
rho_max     = 2.3;
plot_V(rho_max,P);
plot_Vdot(P);

%% Method 1: Line-Search on rho
rho         = 0;
rho_step    = .01;
while(is_verified(rho,P,f))
    rho     = rho+rho_step;
end
rho_method_1 = rho-rho_step

%% Method 2: Single-Shot SOS Program

% Initialize SOS program
prog    = spotsosprog;
x       = msspoly('x',2);
prog    = prog.withIndeterminate(x);

% Lyapunov function and its derivative
V       = x'*P*x;
xdot    = f(0,x);
Vdot    = 2*x'*P*xdot;

% degree of the polynomial lambda(x)
l_deg       = 4;
[prog,l]    = prog.newSOSPoly(monomials(x,0:l_deg));
[prog,rho]  = prog.newFree(1);

% SOS condition
constr      = (x'*x)*(V-rho)-l*Vdot;
prog        = prog.withSOS(constr);

options_spot            = spot_sdp_default_options();
options_spot.verbose    = 0;
sol                     = prog.minimize(-rho,@spot_mosek,options_spot);
if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    disp('Primal infeasible');
end
rho_method_2    = double(sol.eval(rho))
clear prog

%% Method 3: Smarter Single-Shot SOS Program
% Initialize SOS program
prog    = spotsosprog;
x       = msspoly('x',2);
prog    = prog.withIndeterminate(x);

% Lyapunov function and its derivative
V       = x'*P*x;
xdot    = f(0,x);
Vdot    = 2*x'*P*xdot;

% degree of the polynomial lambda(x)
l_deg       = 4;
[prog,l]    = prog.newFreePoly(monomials(x,0:l_deg));
[prog,rho]  = prog.newFree(1);

% SOS condition
constr      = (x'*x)*(V-rho)-l*Vdot;
prog        = prog.withSOS(constr);

options_spot            = spot_sdp_default_options();
options_spot.verbose    = 0;
sol                     = prog.minimize(-rho,@spot_mosek,options_spot);
if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    disp('Primal infeasible');
end
rho_method_3    = double(sol.eval(rho))
clear prog



function is_success = is_verified(rho,P,f)
% Initialize SOS program
prog    = spotsosprog;
x       = msspoly('x',2);
prog    = prog.withIndeterminate(x);

% Lyapunov function and its derivative
V       = x'*P*x;
xdot    = f(0,x);
Vdot    = 2*x'*P*xdot;

% degree of the polynomial lambda(x)
l_deg       = 4;
[prog,l]    = prog.newFreePoly(monomials(x,0:l_deg));
[prog,slack]= prog.newPos(1);

% SOS condition
eps         = 1e-3;
constr      = -Vdot - l*(rho-V) - eps*x'*x;
prog        = prog.withSOS(constr);

options_spot            = spot_sdp_default_options();
options_spot.verbose    = 0;
sol                     = prog.minimize(slack,@spot_mosek,options_spot);
if ~strcmp(sol.status,'STATUS_PRIMAL_AND_DUAL_FEASIBLE')
    is_success = 0;
else
    is_success = 1;
end
end


 
function plot_V(rho,P)
x1          = linspace(-3,3,50);
x2          = linspace(-3,3,50);
[X1,X2]     = meshgrid(x1,x2);
V           = P(1,1)*X1.^2 + P(1,2)*X1.*X2 + P(2,1)*X1.*X2 + P(2,2)*X2.^2;
contour(X1,X2,V,rho*[1 1],'red','LineWidth',3);
% legend(gca,'${x: V(x)=\rho}$','Interpreter','latex')
end

function plot_Vdot(P)
x1          = linspace(-3,3,50);
x2          = linspace(-3,3,50);
[X1,X2]     = meshgrid(x1,x2);
X1dot       = -X2;
X2dot       = X1 + (X1.^2-1).*X2;
p11         = P(1,1);
p12         = P(1,2);
p21         = P(2,1);
p22         = P(2,2);
Vdot        = 2*(p11*X1.*X1dot + p12*X1.*X2dot + p21*X1dot.*X2 + p22*X2.*X2dot);
contour(X1,X2,Vdot,-10:10:40,'b-','LineWidth',1.5,'ShowText','on');
end