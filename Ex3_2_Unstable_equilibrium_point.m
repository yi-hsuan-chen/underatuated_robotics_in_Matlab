%%====== Underactuated Robotics at MIT ======
% https://underactuated.csail.mit.edu/pend.html
clc; clear; close all;

%% Example 2.3: Unstable equilibrium point that attracts all trajectories
plotpp(@dynamics,'xlim', [-1.5,1.5],'ylim', [-1.5,1.5],'linecolor',[0,0,1],...
       'xPlotNum',10,'yPlotNum',10);
ax = gca; xlabel('$x_1$'); ylabel('$x_2$');
set(ax.XLabel,'interpreter','latex','FontSize',18);
set(ax.YLabel,'interpreter','latex','FontSize',18);
set(ax,'DataAspectRatio',[1 1 1],'ticklabelinterpreter','latex');
set(ax.XAxis,'FontSize',12);
set(ax.YAxis,'FontSize',12)
RemovePlotWhiteArea(ax)

%% The damped pendulum
paras.m     = 1;
paras.g     = 9.81;
paras.l     = 1;
paras.b     = 0.1;
paras.u0    = 0.5;
ybar        = paras.m*paras.g*paras.l/paras.b;
pendulum    = @(t,x,paras) [x(2); ...
    1/(paras.m*paras.l^2)*(paras.u0-paras.b*x(2)-paras.m*paras.g*paras.l*sin(x(1)))];
plotpp(@(t,x) pendulum(t,x,paras),'xlim',[-2*pi,2*pi],'linecolor',[0,0,1],...
    'plotNonSaddleTrajectory',false,'arrowSize',12,'axisMarginRatio', 0.5);

function xdot = dynamics(t,x)
r           = sqrt(x(1)^2+x(2)^2);
theta       = atan2(x(2),x(1));
rdot        = r*(1-r);
thetadot    = sin(theta/2)^2;

xdot        = zeros(2,1);
xdot(1)     = rdot*cos(theta)-r*thetadot*sin(theta);
xdot(2)     = rdot*sin(theta)+r*thetadot*cos(theta);
end