function [dg, pg] = update_all_paths2(dg, pg, v)

dg = [dg v];
dg = [dg; [v' 0]];

j = size(dg, 2);
n = size(dg,1)-1;

pg = [pg zeros(n,1)];
pg = [pg; -1*ones(1,n+1)];
for i = 1:n
    if (i==j || dg(i,j) > 5000)
        pg(i,j) = -1;
    else
        pg(i,j) = i;
        pg(j,i) = j;
    end
end


for k = 1:n
    for i = 1:n
        newD = dg(i,k)+dg(k,j);
        if newD < dg(i,j)
            dg(i,j) = newD;
            pg(i,j) = pg(k,j);
        end
        newD = dg(j,k)+dg(k,i);
        if newD < dg(j,i)
            dg(j,i) = newD;
            pg(j,i) = pg(k,i);
        end
    end
end

