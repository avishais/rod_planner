
M = load('../data/milestones.txt');
SM = load('../data/sub_milestones.txt');
DG = load('../data/dis_graph.txt');
PG = load('../data/parent_graph.txt'); 
PG(PG~=-1) = PG(PG~=-1) + 1;
FP = load('../data/full_path.txt'); 

[N,D] = load_nearest_data();

%%
figure(1)
clf

% obs
% patch(2*cos(0:0.1:2*pi),2*sin(0:0.1:2*pi),'y');

hold on

% Edges
for i = 1:size(N,1)
    for j = 1:size(N{i},2)
        if i>=size(N,1)-1 || N{i}(j)>=size(N,1)-1
            plot([M(i,1) M(N{i}(j),1)],[M(i,2) M(N{i}(j),2)],':m');
        else
            plot([M(i,1) M(N{i}(j),1)],[M(i,2) M(N{i}(j),2)],'k');
        end
    end
end


% Milestones
plot(M(1:end-1,1),M(1:end-1,2),'ok','MarkerSize',10,'MarkerFaceColor','r');
for i = 1:size(M,1)-1
    text(M(i,1)+0.5,M(i,2),num2str(i),'FontSize',12,'Color','r');
end

% Start and goal nodes
plot(M(end-1:end,1),M(end-1:end,2),'ok','MarkerSize',10,'MarkerFaceColor','m');
for i = size(M,1)-1:size(M,1)
    text(M(i,1)+0.5,M(i,2),num2str(i),'FontSize',12,'Color','r');
end

% Sub_milestones
plot(SM(:,1),SM(:,2),'ok','MarkerSize',6,'MarkerFaceColor','b');
for i = 1:size(SM,1)
    text(SM(i,1)+0.1,SM(i,2),num2str(i),'FontSize',8,'Color','b');
end

% Shortest path
% P = shortest_path(PG, 542, 1001);
% for i = 2:length(P)
%     plot([M(P(i),1) M(P(i-1),1)],[M(P(i),2) M(P(i-1),2)],':y','LineWidth',2.5);
% end

% Full path with sub_milestones
if (1)
    for i = 2:size(FP,1)
        plot([FP(i,1) FP(i-1,1)],[FP(i,2) FP(i-1,2)],':r','MarkerSize',7,'MarkerFaceColor','k','LineWidth',2.5);
    end
    plot(FP(:,1),FP(:,2),'or','MarkerSize',7,'MarkerFaceColor','g');
    for i = 1:size(FP,1)
        text(FP(i,1)+0.2,FP(i,2)+0.2,num2str(i-1),'FontSize',10,'Color','k');
    end
end

axis equal
axis([-10 10 -10 10]);

hold off
