FP = load('../data/full_path.txt'); 
n = size(FP,1);
N = 500;

max(abs(diff(FP)))
%%
figure(1)
subplot(121)
plot(FP,'.-');
legend('a_1','a_2','a_3','a_4','a_5','a_6');


%%
FPR = load('../data/full_path_rod.txt'); 
c = [1 1 1];
L = 1;
q0 = eye(4);

% figure(1)
subplot(122)
ax = [min(FPR) max(FPR)]*1.2;
for i = 1:n
    P = FPR((i-1)*N+1:i*N,:);
    
    a = FP(i,:);
    X0 = [reshape(q0(1:3,:)',1,12) a];
    opt = odeset('RelTol',10^-6,'AbsTol',10^-6);
    [T,sol] = ode45(@(t,X) eqsA(t,X,c),linspace(0,L,500),X0,opt);
    Pode = [sol(:,4) sol(:,8) sol(:,12)]*1e3;
    q=[sol(end,1:12) zeros(1,3) 1];
    Q=permute(reshape(q',4,4,1),[2 1 3]);
    Q(1:3,4) = Q(1:3,4)*1e3;
    
%     clf;
    plot3(P(:,1),P(:,2),P(:,3),'-k');
    hold on
    plot3(P(end,1),P(end,2),P(end,3),'ok');
    plot3(Pode(:,1),Pode(:,2),Pode(:,3),'--r');
    plot3(Pode(end,1),Pode(end,2),Pode(end,3),'xr');
    plot3(0,0,0,'ok','MarkerFaceColor','k');
    hold off
    axis equal
    grid on
    xlim([ax(1) ax(4)]);
    ylim([ax(2) ax(5)]);
    zlim([ax(3) ax(6)]);
    drawnow;
end

