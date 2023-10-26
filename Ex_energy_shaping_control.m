%%====== Underactuated Robotics at MIT ======
% https://underactuated.csail.mit.edu/pend.html
clc; clear; close all;

global m g l
m       = 1;
g    	= 9.81;
l       = 1;
paras.k = 0.1;

%% Energy-shaping control of the simple pendulum
plotpp(@(t,x) pendulum(t,x,0),'xlim',[-pi,3*pi],'ylim',[-10,10],'linecolor',0.8*[1 1 1]);
ax(1) = gca; hold on;

nSim    = 5;
for i = 1:nSim
x0          = randn(2,1);
tspan       = [0 10];
u           = @energyShapingController;
[t,x]       = rk4(@(t,x) pendulum(t,x,u(t,x,paras)),tspan,x0);
plot(x(:,1),x(:,2),'b','LineWidth',2);
plot(x0(1),x0(2),'gs','MarkerSize',12,'LineWidth',3);
plot(x(end,1),x(end,2),'ko','MarkerSize',10,'LineWidth',3);
end
hold off;
xlabel('$\theta$'); ylabel('$\dot{\theta}$');
set(ax(1),'XTick',-pi:pi:3*pi,'XTickLabel',{'$-\pi$','0','$\pi$','$2\pi$','$3\pi$'});

%% Swing-up and stabilize control
xeq     = [pi;0];
[A,B]   = linearize(xeq);
Q       = diag([10 1]);
R       = 1;
[K,S]   = lqr(A,B,Q,R);
paras.K = K;
paras.S = S;

plotpp(@(t,x) pendulum(t,x,0),'xlim',[-pi,3*pi],'ylim',[-10,10],'linecolor',0.8*[1 1 1]);
ax(2) = gca; hold on;
xlabel('$\theta$'); ylabel('$\dot{\theta}$');

for i = 1:nSim
x0          = randn(2,1);
tspan       = [0 10];
u           = @SwingUpAndBalanceController;
[t,x]       = rk4(@(t,x) pendulum(t,x,u(t,x,paras)),tspan,x0);
theta       = x(:,1);
dtheta      = x(:,2);
xPos        = l*sin(theta);
yPos        = -l*cos(theta);
plot(ax(2),theta,dtheta,'b','LineWidth',2);
plot(ax(2),theta(1),dtheta(2),'gs','MarkerSize',12,'LineWidth',3);
plot(ax(2),theta(end),dtheta(end),'ko','MarkerSize',10,'LineWidth',3);
animateSimplePendulum;
end

set(ax,'XGrid','on','YGrid','on');
set(ax,'ticklabelinterpreter','latex');

for i = 1:length(ax)
    set(ax(i).XLabel,'interpreter','latex','FontSize',18);
    set(ax(i).YLabel,'interpreter','latex','FontSize',18);
    set(ax(i).XAxis,'FontSize',12);set(ax(i).YAxis,'FontSize',12);
    RemovePlotWhiteArea(ax(i));
end


function u = SwingUpAndBalanceController(t,x,paras)
K       = paras.K;
S       = paras.S;

xbar    = x;
xbar(1) = wrapTo2Pi(xbar(1))-pi;

% If x'Sx <= 2, then use the LQR controller
if xbar'*S*xbar < 2
    u   = -K*xbar;
else
    u   = energyShapingController(t,x,paras);
end

end


function [A,B] = linearize(xeq)
global m g l

syms x1 x2 u
x       = [x1;x2];
dx      = [x2;1/(m*l^2)*(u-m*g*l*sin(x1))];
A       = double(subs(jacobian(dx,x),x,xeq));
B       = double(subs(jacobian(dx,u),x,xeq));
end

function xdot = pendulum(t,x,u)
global m g l

theta   = x(1);
thetadot= x(2);

xdot    = zeros(2,1);
xdot(1) = thetadot;
xdot(2) = 1/(m*l^2)*(u-m*g*l*sin(theta));
end


function u = energyShapingController(t,x,paras)
global m g l
k       = paras.k;
theta   = x(1);
thetadot= x(2);

T       = .5*m*l^2*thetadot^2;
U       = -m*g*l*cos(theta);
E_des   = m*g*l;
E_tilde = (T+U)-E_des;

u       = -k*thetadot*E_tilde;
end