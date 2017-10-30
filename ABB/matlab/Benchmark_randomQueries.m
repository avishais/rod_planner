% Test what is the success rate for the 100 random queries in "./data/feasible_queries.txt" (known to
% have a solution) with regards to the size of the map and number of
% connections.

clear all
% clc

%%

D = load('Benchmark_rand_sg_noObs.txt');
D = D(D(:,3) <= 99, :);

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

%%
Y = D(D(:,1)==500 ,[2 3 6 8]); %& D(:,3)==0
knn = unique(Y(:,1));
n = unique(Y(:,2));
E = zeros(length(n), length(knn));
R = zeros(length(knn), 1);

for i = 1:length(n)
    M = Y(Y(:,2)==n(i),:);
    M = sortrows(M,1);
    [minT, im] = min(M(:,4));
    for j = 1:size(M,1)
        if ~M(j,3)
            continue;
        end
        E(i,M(j,1)-1) = (M(j,4) - minT)/minT;
    end
    
    im = M(im,1) - 1;
    R(im) = R(im) + 1;
end

v = mean(E(1:end,:));

figure(3)
subplot(211)
plot(knn, v);
subplot(212)
plot(knn, R);