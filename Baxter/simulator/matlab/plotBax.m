function plotBax(P)

P = [[0 0 0]; P];

plot3(P(:,1),P(:,2),P(:,3),'o-k','markerfacecolor','r');
hold on
plot3(P(1,1),P(1,2),P(1,3),'ok','markerfacecolor','y','markersize',12);

plot3(P(1,1),P(1,2),P(1,3),'ok','markerfacecolor','y','markersize',12);
hold off

grid on
xlabel('x');
ylabel('y');
zlabel('z');
