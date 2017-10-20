D = load('feasible_queries.txt');
D = D(:,1:2);

for i = 1:size(D,1)-1
    for j = i+1:size(D,1)
        if all(D(i,:)==D(j,:)) || all(D(i,:)==fliplr(D(j,:)))
            disp([i j]);
        end
    end
end