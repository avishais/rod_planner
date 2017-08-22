function P = shortest_path(PG, s, g)

P = s;

found = 0;
while ~found
    nx = PG(g, s);
    
    if nx==-1
        disp('No solution');
        return;
    end
    %disp(nx);
    P = [P nx];
    
    if nx==g
        found = 1;
    else
        s = nx;
    end
end

