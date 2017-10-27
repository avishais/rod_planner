% Benchmarking the pole scene of the ABB robots

clear all
clc

%%

D = load('Benchmark_poleScene.txt');

% D(D(:,1)==500 & D(:,2)==3 & D(:,3)==0,:) = [];
% D(D(:,1)==500 & D(:,2)==2 & D(:,3)==0,:) = [];
% D(D(:,1)==500 & D(:,2)==6 & D(:,3)==0,:) = [];
% 
% dlmwrite('Benchmark_poleScene.txt', D, ' ');

%%

ms = unique(D(:,1));
knn = unique(D(:,2));
T = zeros(length(ms),length(knn));
S = zeros(length(ms),length(knn));

for i = 1:length(ms)
    M = D(D(:,1)==ms(i), 2:end);
    for j = 1:length(knn)
        K = M(M(:,1)==knn(j),2:end);
        S(i,j) = sum(K(:,1))/size(K,1);
        T(i,j) = mean(K(K(:,1)==1,3));
        T_std(i,j) = std(K(K(:,1)==1,3))/sqrt(size(K,1));
        N(i,j) = size(K,1);
    end
end

%%

for j = 1:length(ms)
    K = D(D(:,1)==ms(j) & D(:,2)==4,3:end);
    
    t = K(K(:,1)==1,3);
    maxT = max(t);
    tt = linspace(0,maxT,20);
    tt = tt(2:end);
    for i = 1:length(tt)
        s = t < tt(i);
        m(i) = 1-sum(s)/length(t);
    end
    
    Ty(j,:) = [0 tt];
    Ky(j,:) = [1 m];    
end

%% PLOTS
%% Runtime for each map and density

colors = 'kbr';


figure(1)
clf
hold on
for i = 1:length(ms)
    %plot(knn, T(i,:),'o','color',colors(i),'markerfacecolor',colors(i));
    errorbar(knn, T(i,:),T_std(i,:),'o','color',colors(i),'markerfacecolor',colors(i));
end
hold off
xlabel('k');
ylabel('time (sec)');
legend('ms100','ms500','ms1000');

%% Success rate for each map and density

figure(2)
clf
bar(knn,S'*100);
xlabel('k');
ylabel('success rate (%)');
legend('ms100','ms500','ms1000');

%% Runtime failure rate for k=4

h = figure(3);
clf
plot(Ty(1,:),Ky(1,:)*100,'-k','linewidth',2);
hold on
plot(Ty(2,:),Ky(2,:)*100,'--k','linewidth',2);
plot(Ty(3,:),Ky(3,:)*100,':k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
xlim([0 max(Ty(1:end))]);
legend('m=100','m=500','m=1000');
set(gca,'fontsize',12);
set(h, 'Position', [100, 100, 800, 400]);
% print success_abb_pre.eps -depsc -r200

%% Runtime break

M = D(D(:,1)==500 & D(:,2)==4, 3:end);
M = M(M(:,13)<3 & M(:,13)>0, :);
v = mean(M(:,:));

t1(1) = v(5); % IK time
t1(2) = v(7); % Collision time
t1(3) = v(13); % ODE
t1(4) = v(3); % Total runtime

n1(1) = v(4);
n1(2) = v(6);
n1(3) = v(11);






