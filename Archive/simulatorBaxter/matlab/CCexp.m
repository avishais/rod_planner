function C = CC(qf, TL)

b = 0;
l1x= 0.024645e3+0.055695e3-0.02e3; l1y=-0.25e3; l1z=0.118588e3+0.011038e3;
l2a=0.073e3;
l2b=0.245e3;
l3=0.102e3;
l4a=0.069e3;
l4b=0.26242e3-0.015e3;
l5=0.1e3;
l6a=0.01e3;
l6b=0.2707e3;
l7=0.16e3;
lEE=0.15e3;

% r11 = T(1,1); r12 = T(1,2); r13 = T(1,3); t1 = T(1,4);
% r21 = T(2,1); r22 = T(2,2); r23 = T(2,3); t2 = T(2,4);
% r31 = T(3,1); r32 = T(3,2); r33 = T(3,3); t3 = T(3,4);

% Cons;

% R1
q = qf(1:7);
T01 = Tz(q(1));
T02 = T01*Tt([l2a;0;l2b])*Ty(q(2))*Tx(-pi/2);
T03 = T02*Tt([l3;0;0])*Tx(q(3)+pi/2)*Ty(pi/2);
T04 = T03*Tt([l4a;0;l4b])*Ty(q(4))*Tx(pi/2);
T05 = T04*Tt([l5;0;0])*Tx(q(5)-pi/2)*Ty(pi/2);
T06 = T05*Tt([l6a;0;l6b])*Ty(q(6)-pi/2)*Tx(pi/2);
T07 = T06*Tt([l7;0;0])*Tx(q(7)+pi/2)*Ty(pi/2);
T0EE1 = T07 * Tt([0;0;lEE]);
Tt1 = Tt([0;0;b])*Tt([l1x;l1y;l1z]); % position of shoulder pf R1 relative to center of torso (world frame)
T1 = Tt1*T0EE1;

% R2
q = qf(8:14);
T01 = Tz(q(1));
T02 = T01*Tt([l2a;0;l2b])*Ty(q(2))*Tx(-pi/2);
T03 = T02*Tt([l3;0;0])*Tx(q(3)+pi/2)*Ty(pi/2);
T04 = T03*Tt([l4a;0;l4b])*Ty(q(4))*Tx(pi/2);
T05 = T04*Tt([l5;0;0])*Tx(q(5)-pi/2)*Ty(pi/2);
T06 = T05*Tt([l6a;0;l6b])*Ty(q(6)-pi/2)*Tx(pi/2);
T07 = T06*Tt([l7;0;0])*Tx(q(7)+pi/2)*Ty(pi/2);
T0EE2 = T07 * Tt([0;0;lEE]);
Tt2 = Tt([0;0;b])*Tt([l1x;-l1y;l1z]); % position of shoulder of R2 relative to center of torso (world frame)
T2 = Tt2*T0EE2;

To = [-1, 0, 0, 0;
    0, -1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
C = T1*TL*To-T2;
C = C(1:3,1:4);
C = C(1:12);


end

%% Tz Tx Ty
function T = Tz( x)

T = [cos(x) -sin(x) 0 0; sin(x) cos(x) 0 0; 0 0 1 0; 0 0 0 1];

end

function T = Ty( x)

T = [cos(x) 0 sin(x) 0; 0 1 0 0; -sin(x) 0 cos(x) 0; 0 0 0 1];

end

function T = Tx( x)

T = [1 0 0 0; 0 cos(x) -sin(x) 0; 0 sin(x) cos(x) 0; 0 0 0 1];

end

function T = Tt( v)

T = [[eye(3) v]; 0 0 0 1];

end