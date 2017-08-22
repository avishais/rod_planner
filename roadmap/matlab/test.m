n = size(N,1);
dg = 10000*ones(n,n);
pg = -1*ones(n,n);

for i = 1:n
    for j = 1:length(N{i})
        dg(i,N{i}(j)) = D{i}(j);
    end
    dg(i,i) = 0;
end

dg(:,6) = [];
dg(6,:) = [];

[dg, pg] = all_paths(dg);

[dg1, pg1] = update_all_paths(dg, pg, N{6}, D{6});