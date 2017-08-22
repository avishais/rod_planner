% This function contains the differential equations that govern the shape
% of the rod
function[dX]=eqsA(~,X,c)

% Extract variables q, mu, M, and J from vector X
q=[reshape(X(1:12),4,3)'; 0 0 0 1];
mu=X(13:18);

% Compute variables u
k=1./c;
u=mu(1:3)'.*k;

% System of differential equations for q and mu
dq=q*[0 -u(3) u(2) 1; u(3) 0 -u(1) 0; -u(2) u(1) 0 0; 0 0 0 0];

dmu=[u(3)*mu(2)-u(2)*mu(3)...
     mu(6)+u(1)*mu(3)-u(3)*mu(1)...
     -mu(5)+u(2)*mu(1)-u(1)*mu(2)...
     u(3)*mu(5)-u(2)*mu(6)...
     u(1)*mu(6)-u(3)*mu(4)...
     u(2)*mu(4)-u(1)*mu(5)];

% Collect derivatives in vector dX
dX=[reshape(dq(1:3,1:4)',1,12) dmu]';

end