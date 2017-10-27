% Test what is the success rate for the 100 random queries in "./data/feasible_queries.txt" (known to
% have a solution) with regards to the size of the map and number of
% connections.

clear all
% clc

%%

D = load('Benchmark_rand_sg_noObs_b4.txt');

%% Check

n = unique(D(:,3));
for i = 1:length(n)
    s = D(D(:,3)==n(i),6);
    if ~any(s)
        disp(['Failed in ' num2str(n(i))]);
    end
end

%% Proccess

ms = unique(D(:,1));
knn = unique(D(:,2));
T = zeros(length(ms), length(knn));
S = zeros(length(ms), length(knn));

for i = 1:length(ms)
    M = D(D(:,1)==ms(i), 2:end);
    for j = 1:length(knn)
        K = M(M(:,1)==knn(j),2:end);
        S(i,j) = sum(K(:,4))/size(K,1);
        T(i,j) = mean(K(K(:,4)==1,6));
    end
end

%%
figure(1)
subplot(211)
bar3(T);
set(gca,'XTickLabel',2:6);
set(gca,'YTickLabel',{'100','500','1000'});
zlabel('time (sec)');

subplot(212)
bar3(S*100);
set(gca,'XTickLabel',2:6);
set(gca,'YTickLabel',{'100','500','1000'});
zlabel('success rate');

%%

figure(2)
[hAx,hLine1,hLine2] = plotyy(knn, S(2,:)*100, knn, T(2,:));
legend('success rate','runtime (sec)');
xlabel('k nearest neighbors')

ylabel(hAx(1),'success rate (%)') % left y-axis
ylabel(hAx(2),'runtime (sec)') % right y-axis