% One pose animation

D = 1085.85; % Distance between the two robots along x
RobotArm1 = RobotArm_N1([-D/2 0 0],'k','coarse'); %'none' % Generate Robot 1
RobotArm2 = RobotArm_N1([D/2 0 180],'k','coarse'); % Generate Robot 2
c = [1 1 1];
L = 1;
q0 = eye(4);

%%
% C = [2.17, -4.2, 2, 1.9, 9.6, -2.4, 0.0873, 0.1745,  0, 0, 0.1745, 0, 0.972532, 0.309214, 0.505241, 0.848896, -1.59516, 0.0524666];
% C = [0, -4, 0, -5, 0, 3, 1.9, 0.6726, 0.1745, -1.5707, 0.7, -2.6, -0.9826, 1.1060, 0.4769, -0.5696, -1.5875, -0.9615];

% C = [1.13317 -4.08401 2.74606 3.77001 6.46315 -2.83533 -0.0718585 0.736297 -0.795402 -0.355589 0.476945 1.02925 0.767703 0.801507 -0.122032 0.700162 -1.54235 -0.761076 ];
%     0.795402 0.355589 0.476945 0.02925 0.767703 0.801507 0.122032 0.700162 1.54235 0.761076 0.0925301 0.747572 ];

% Half circle to helix shape
% C = [0, 3.2, 0, 0, 0, 0, 0, -0.50,0.440898,-0,-1.57,0,0,-0.000695474163865839,-0.00798116494726853,0,-1.56222150355732,0];
C = [0, -4, 0, -5, 0, 3,-1.4,0.419512,0.064,1.2,1.0,-2.6,-0.588374911336135,-0.277496816097365,-0.384651761569099,-2.16861949996107,-1.72941957387009,3.04121873510677];


Joints_Robot1 = C(7:12);
Joints_Robot2 = C(13:18);
a = C(1:6);
a = [0.113813 -3.72774 -0.0284137 -4.79099 -0.00683304 2.70805];
%% Rod
X0 = [reshape(q0(1:3,:)',1,12) a];
opt = odeset('RelTol',10^-6,'AbsTol',10^-6);
[T,sol] = ode45(@(t,X) eqsA(t,X,c),linspace(0,L,500),X0,opt);

P = [sol(:,4) sol(:,8) sol(:,12)]*1e3;
q=[sol(end,1:12) zeros(1,3) 1];
Q=permute(reshape(q',4,4,1),[2 1 3]);
Q(1:3,4) = Q(1:3,4)*1e3;

% disp(IsNodeFeasibleA(a));
%% Arm
% Define motion of T1
[T1, RC1] = RobotArm1.FK(Joints_Robot1,1);
T_E1_W = T1;
% P = N;
Pe = (T_E1_W * [P ones(size(P,1),1)]')';

% Compute required T2 motion
T2 = T_E1_W * Q; % In robot 1's CF
T2(1:3,1:3) = T2(1:3,1:3) * [-1 0 0; 0 -1 0; 0 0 1];

%%
s = 2;
[Q2, f1] = RobotArm2.IKP(T2, 1, s); % We send the T2 in the world coordinate frame.
if f1
    disp('Failed.');
    %         continue;
end

Q2 = Joints_Robot2';
[~,RC2] = RobotArm2.FK(Q2,1);
Q2 = RobotArm1.fix_joints_4_6(Q2);

%% Animate

clf
RobotArm1.Robot2Fig(1, RC1);
RobotArm2.Robot2Fig(1, RC2);

patch([-700 -700 1200 1200], [-500 500 500 -500], [0.8 0.8 0.8]);
hold on
plot3(Pe(:,1),Pe(:,2),Pe(:,3),'k','LineWidth',5);

hold off
axis equal
xlabel('x');
ylabel('y');
zlabel('z');
grid on
%     axis([-750 1200 -500 800 0 1050]);
view(-171,18)
disp(s)
drawnow;
