%%====== Underactuated Robotics at MIT ======
% https://underactuated.csail.mit.edu/pend.html
% Edited by Yi-Hsuan Chen (yhchen91@umd.edu) 10/26/2023

clc; clear; close all;

% Set up system dynamics
paras.mc    = 2;        % mass of cart
paras.mp    = 0.5;      % mass of pole
paras.l     = 2.2;      % length of pole
paras.g     = 9.81;
paras.w     = 1.6;      % width of cart (for animation)
paras.h     = 0.8;      % height of cart (for animation)
cartpole    = CartPole(paras);
dynamics    = @cartpole.dynamics;
linearize   = @cartpole.linearize;
xeq         = [0;pi;0;0];
ueq         = 0;
[A,B]       = linearize(xeq,ueq);

% Balancing using LQR
Q           = diag([10,10,1,1]);
R           = 1;
[K,S]       = lqr(A,B,Q,R);
controller  = @(x) -K*(x-xeq);
fcl         = @(t,x) dynamics(t,x,controller(x));
tspan       = [0 8];

nSim        = 5;
paras.div   = 100;
for i = 1:nSim
x0          = xeq+0.4*randn(4,1);
[t,X]       = rk4(fcl,tspan,x0);
xPos        = X(:,1);
theta       = X(:,2);
% Visualization and animation
title(sprintf('Simulation: %d',i),'Interpreter','latex','FontSize',18);
animateCartPole(t,xPos,theta,paras)
end
close all;