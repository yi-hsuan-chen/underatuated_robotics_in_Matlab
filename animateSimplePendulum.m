if i == 1
    fig(3)  = figure(); ax(3) = gca;
else
    figure(fig(3)); ax(3) = gca;
end

p(1)    = plot(ax(3),[0 xPos(1)],[0 yPos(1)],'b','Linewidth',4); hold on; grid on;
p(2)    = plot(ax(3),xPos(1),yPos(1),'LineStyle','none');
p(3)    = plot(ax(3),xPos(1),yPos(1),'Color','magenta','LineStyle','--','Linewidth',1.5);
p(4)    = plot(ax(3),0,0,'LineStyle','none');
set(p(2),'Marker','o','MarkerSize',14,'MarkerEdgeColor','r','MarkerFaceColor','r')
set(p(4),'Marker','^','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','k')
xlabel('$x$-position  (m)'); ylabel('$y$-position  (m)');
title(sprintf('Simulation: %d',i),'Interpreter','latex','FontSize',18);
set(ax(3).XLabel,'Interpreter','latex','FontSize',18);
set(ax(3).YLabel,'Interpreter','latex','FontSize',18);
set(ax(3),'ticklabelinterpreter','latex');

Ns      = 150;
for i = 1:Ns:length(t)
    set(p(1),'XData',[0 xPos(i)],'YData',[0 yPos(i)]);
    set(p(2),'XData',xPos(i),'YData',yPos(i))
    set(p(3),'XData',xPos(1:i),'YData',yPos(1:i))
    xlim([-1.5 1.5]); ylim([-1.2 1.2]); axis equal;
    set(ax(3).XAxis,'FontSize',12); set(ax(3).YAxis,'FontSize',12);
    drawnow();
end
hold off;