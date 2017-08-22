function [dg, pg] = all_paths(dg)
n = size(dg,1);

pg = zeros(n,n);
for i = 1:n
    for j = 1:n
        if (i==j || dg(i,j) > 5000)
            pg(i,j) = -1;
        else
            pg(i,j) = i;
        end
    end
end



for k = 1:n
    for i = 1:n 
        for j = 1:n 
            newD = dg(i,k)+dg(k,j);
            if newD < dg(i,j)
                dg(i,j) = newD;
                pg(i,j) = pg(k,j);
            end
        end
    end
end


