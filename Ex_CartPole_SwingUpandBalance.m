%%====== Underactuated Robotics at MIT ======
% https://underactuated.csail.mit.edu/pend.html
% Edited by Yi-Hsuan Chen (yhchen91@umd.edu) 10/26/2023

clc; clear; close all;

% Simplified dynamics (Set all constants 1)
paras.mc    = 1;        % mass of cart
paras.mp    = 1;        % mass of pole
paras.l     = 1;        % length of pole
paras.g     = 1;

paras.w     = 1.6;      % width of cart (for animation)
paras.h     = 0.8;      % height of cart (for animation)
cartpole    = CartPole(paras);
dynamics    = @cartpole.dynamics;
linearize   = @cartpole.linearize;
xeq         = [0;pi;0;0];
ueq         = 0;
paras.xeq   = xeq;

% SwingUp energy shaping controller parameters
paras.k     = 8;      
paras.kp    = 4;
paras.kd    = 0.1;

% Balancing using LQR
[A,B]       = linearize(xeq,ueq);
Q           = diag([10,10,1,1]);
R           = 1;
[K,S]       = lqr(A,B,Q,R);
paras.K     = K;
paras.S     = S;

u           = @SwingUpAndBalanceController;
fcl         = @(t,x) dynamics(t,x,u(t,x,paras));
tspan       = [0 15];

nSim        = 5;
for i = 1:nSim
x0          = randn(4,1);
% x0          = [1; pi-pi/4; 0.2 ; 0];
[t,X]       = ode45(fcl,tspan,x0);
xPos        = X(:,1);
theta       = wrapTo2Pi(X(:,2));
theta(end)
% Visualization and animation
title(sprintf('Simulation: %d',i),'Interpreter','latex','FontSize',18);
animateCartPole(t,xPos,theta,paras)
end
close all;


function u = SwingUpAndBalanceController(t,X,paras)
K       = paras.K;
S       = paras.S;
k       = paras.k;
kp      = paras.kp;
kd      = paras.kd;

x       = X(1);
theta   = X(2);
xdot    = X(3);
dtheta  = X(4);

xbar    = X;
xbar(2) = wrapTo2Pi(xbar(2))-pi;

% If x'Sx <= ??, then use the LQR controller
if xbar'*S*xbar <= 15 || abs(xbar(2)) < pi/10
    u   = -K*xbar;
else
    cT      = cos(theta);
    sT      = sin(theta);
    E_des   = 1;
    E       = .5*dtheta^2-cT;
    E_tilde = E-E_des;
    ddx_des = k*dtheta*cT*E_tilde-kp*x-kd*xdot;
    u       = (2-cT^2)*ddx_des - sT*cT - dtheta^2*sT;
end

end