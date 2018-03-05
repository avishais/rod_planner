% Benchmarking the pole scene of the ABB robots

clear all
clc

%%

D = load('Benchmark_poleScene_ODEanalysis.txt');

D = D(:,[1 2 13 15 16]);

%%

ms = unique(D(:,1));

for i = 1:length(ms)
    t(i) = mean(D(D(:,1)==ms(i), 4));    
    ts(i) = std(D(D(:,1)==ms(i), 4))/size(D(:,1)==ms(i),1);  
    n(i) = mean(D(D(:,1)==ms(i), 3));
    ns(i) = std(D(D(:,1)==ms(i), 3))/size(D(:,1)==ms(i),1);  
end

h = figure(1);
clf
[ax, h1, h2] = plotyy(ms, t, ms, n);
set(h1,'linestyle','-','color','k','linewidth',2.5,'marker','o','markerfacecolor','k');
set(h2,'linestyle',':','color','k','linewidth',2.5,'marker','^','markerfacecolor','k');
set(ax(1),'YColor',[0 0 0]);
set(ax(2),'YColor',[0 0 0]);

% hold(ax(1), 'on');
% hold(ax(2), 'on');

% errorbar(ax(1), ms, t, ts, '-k','linewidth',2.5);
xlim(ax(1), [0 1050]);
% set(h1,'linestyle','-','color','black');

% errorbar(ax(2), ms, n, ns, ':k','linewidth',2.5);
xlim(ax(2), [0 1050]);

legend('time','comp.');

% hold(ax(1), 'off');
% hold(ax(2), 'off');

xlabel('number of milestones - m');
ylabel(ax(1),'time (sec)');
ylabel(ax(2),'number of computations');
set(gca, 'fontsize', 13);

set(h, 'Position', [100, 100, 800, 300]);
% print odeconnections.eps -depsc -r200










