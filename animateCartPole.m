function animateCartPole(t,xPos,theta,paras)
w       = paras.w;
h       = paras.h;
xc      = xPos(1);
yc      = h/2;

% plot the cart using four vertices
xvert   = [xc-w/2 xc+w/2 xc+w/2 xc-w/2];
yvert   = [yc-h/2 yc-h/2 yc+h/2 yc+h/2];

gcf; ax(1) = gca; axis equal; ylim([-2 4]);
if max(xPos) > 5 || min(xPos) < -5
    xbuffer     = 1;
    xlim([min(xPos)-xbuffer max(xPos)+xbuffer]); 
else
    xlim([-5 5]); 
end

cart        = patch(xvert,yvert,'blue','FaceAlpha',.8); hold on;
% plot the pole
a           = 0:0.01:paras.l;
pole        = plot(zeros(1,length(a)),a,'c','linewidth',4);
theta0      = theta(1);
x           = xc+a*sin(theta0);
y           = yc-a*cos(theta0);
locateX     = x(end);                   % x-Joint location of 1st-arm;
locateY     = y(end);                   % y-Joint location of 1st-arm;
k           = plot(locateX,locateY,'ro','LineStyle','none','MarkerSize',10,'MarkerFace','r'); 
traj        = plot(locateX,locateY,'m.','LineStyle','none');

set(pole,'xdata',x,'ydata',y);
xlabel('$x$  (m)'); ylabel('$y$  (m)');
set(ax,'Fontsize',12,'XMinorGrid','on','YMinorGrid','on','TickLabelInterpreter','latex');

for i = 1:2:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
end

% v_handle = VideoWriter('cartPoleBalancingLQR','MPEG-4');
% open(v_handle);
divider     = paras.div;
for i = 1:divider:length(t)
    x           = xPos(i)+a*sin(theta(i));
    y           = yc-a*cos(theta(i));
    locateX(i)  = x(length(x));
    locateY(i)  = y(length(y));
    
    set(cart,'XData',[xPos(i)-w/2 xPos(i)+w/2 xPos(i)+w/2 xPos(i)-w/2]);
    set(pole,'XData',x,'YData',y);
    set(k,'XData',locateX(i),'YData',locateY(i));
    set(traj,'XData',locateX(1:divider:i),'YData',locateY(1:divider:i));

    drawnow;
    if max(xPos) > 5 || min(xPos) < -5
        xbuffer     = 1;
        xlim([min(xPos)-xbuffer max(xPos)+xbuffer]);
    else
        xlim([-5 5]);
    end

%     frame = getframe(fig(1));
%     writeVideo(v_handle,frame);
end
pause(0.5);
hold off; clf;
% close(v_handle);
end