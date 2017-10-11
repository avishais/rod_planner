function P = FK(q)

PhyPro;

T{1} = Tz(q(1)) * Tt([l2a; 0; l2b]);
T{2} = Ty(q(2)) * Tt([l3; 0; 0]);
T{3} = Tx(q(3)) * Tt([l4b; 0; -l4a]);
T{4} = Ty(q(4)) * Ty(pi/2) * Tt([l5; 0; 0]);
T{5} = Tx(q(5)) * Tt([l6b; 0; -l6a]);
T{6} = Ty(q(6)) * Tt([l7; 0; 0]);
T{7} = Tx(q(7)) * Tt([lEE; 0; 0]);

Q = eye(4);
for i = 1:7
    Q = Q * T{i}; 
    P(i,:) = Q * [0;0;0;1];
end

P = P(:,1:3);


end

function T = Tz(x)

T = [cos(x) -sin(x) 0 0; sin(x) cos(x) 0 0; 0 0 1 0; 0 0 0 1];

end

function T = Ty(x)

T = [cos(x) 0 sin(x) 0; 0 1 0 0; -sin(x) 0 cos(x) 0; 0 0 0 1];

end

function T = Tx(x)

T = [1 0 0 0; 0 cos(x) -sin(x) 0; 0 sin(x) cos(x) 0; 0 0 0 1];

end

function T = Tt(v)

T = [[eye(3) v]; 0 0 0 1];

end