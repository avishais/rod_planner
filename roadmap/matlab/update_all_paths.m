function [dg, pg] = update_all_paths(dg, pg, neighbors, dist)

n = size(dg,1);

dg = [dg 10000*ones(n,1)];
dg = [dg; [10000*ones(1,n) 0]];

pg = [pg -1*ones(n,1)];
pg = [pg; -1*ones(1,n+1)];
 


j = n + 1;

for i = 1:n
    for k = 1:length(neighbors)
        newD = dg(i,neighbors(k))+dist(k);
        if newD < dg(i,j)
            dg(i,j) = newD;
            pg(i,j) = neighbors(k);
        end   
    end
    dg(j,i) = dg(i,j);
end

