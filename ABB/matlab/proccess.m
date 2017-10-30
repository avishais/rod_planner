% from info_doc.txt

clear all

k = 2:6;
m = [100 500 1000];
subms = [8718 30071 52033;
    14997 49830 67108;
    21095 70629 119148;
    27324 91126 155476;
    34420 114967 -1];
comp_time = [797 3003 5963
    857.582 3927 6771
    1392.9 5230 8132
    1482 5227 9545
    1826 6321 -1];
file_size = [107 376 659
    184 617 1100
    259 871 1500
    335 1100 1900
    421 1400 -1];

%%
color = 'rbk';

figure(8)
clf

subplot(2,2,[1 3])
hold on
for i = 1:size(comp_time,2)
    ct = comp_time(:,i)~=-1;
    plot(k(ct), comp_time(ct,i)/60,'ok','MarkerFaceColor',color(i),'MarkerSize',8);
end
hold off
set(gca,'XTick',2:1:6);
xlabel('k');
ylabel('time [min]');
title('Roadmap generation time');
legend('MS100','MS500','MS1000');

subplot(2,2,2)
hold on
for i = 1:size(subms,2)
    ct = subms(:,i)~=-1;
    plot(k(ct), subms(ct,i)/1000,'ok','MarkerFaceColor',color(i),'MarkerSize',8);
end
hold off
set(gca,'XTick',2:1:6);
xlabel('k');
ylabel('# * 1000');
title('Number of sub-milestones');
legend('MS100','MS500','MS1000');

subplot(2,2,4)
hold on
for i = 1:size(file_size,2)
    ct = file_size(:,i)~=-1;
    plot(k(ct), file_size(ct,i),'ok','MarkerFaceColor',color(i),'MarkerSize',8);
end
hold off
set(gca,'XTick',2:1:6);
xlabel('k');
ylabel('[MB]');
title('File Size');
legend('MS100','MS500','MS1000');

%% For paper
color = 'rbk';
mrk = 'o^s';
fs = 18;

h = figure(1)
clf
hold on
for i = 1:size(comp_time,2)
    ct = comp_time(:,i)~=-1;
    plot(k(ct), comp_time(ct,i)/60,'.k','Marker',mrk(i),'MarkerFaceColor',color(i),'MarkerSize',12);
end
hold off
set(gca,'XTick',2:1:6);
xlabel('k - nearest neighbors');
ylabel('time [min]');
legend('m=100','m=500','m=1000','location','northwest');
set(gca,'fontsize',fs);
set(h, 'Position', [100, 100, 800, 400]);
% print('roadmap_gen_ms', '-dpng','-r200');

%%
h = figure(2)
clf
hold on
for i = 1:size(subms,2)
    ct = subms(:,i)~=-1;
    plot(k(ct), subms(ct,i),'.k','Marker',mrk(i),'MarkerFaceColor',color(i),'MarkerSize',12);
end
hold off
set(gca,'XTick',2:1:6);
ylabel('number of sub-milestones');
xlabel('k - nearest neighbors');
legend('m=100','m=500','m=1000','location','northwest');
ylim([0 1.6e5]);
set(gca,'fontsize',fs);

set(h, 'Position', [100, 100, 800, 400]);
% print('roadmap_gen_subms', '-dpng','-r200');

%%
h = figure(3)
clf
hold on
for i = 1:size(file_size,2)
    ct = file_size(:,i)~=-1;
    plot(k(ct), file_size(ct,i),'.k','Marker',mrk(i),'MarkerFaceColor',color(i),'MarkerSize',12);
end
hold off
set(gca,'XTick',2:1:6);
ylabel('file size [MB]');
xlabel('k - nearest neighbors');
legend('m=100','m=500','m=1000','location','northwest');
set(gca,'fontsize',fs);

set(h, 'Position', [100, 100, 800, 400]);
% print('roadmap_gen_file', '-dpng','-r200');

