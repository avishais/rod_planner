clear all
clc

% D = load('../robot_paths.txt');
D = dlmread('./path.txt',' ',1,0);
D = D(:,1:14);
A = dlmread('./afile.txt',' ',0,0);

figure(1)
clf
subplot(121)
hold on
for i = 1:14
    %     plot(D(:,1),D(:,i),'.-k');
    plot(rad2deg(D(:,i)),'.-k');
    
    %     plot(Do(:,i),'x--r');
    plot(xlim,180*[1 1],':k','linewidth',1.5);
    plot(xlim,-180*[1 1],':k','linewidth',1.5);
    
    grid on
    %     pause(1)
end
hold off
ylabel('angles [^o]');
title('Angles');

subplot(122)
hold on
for i = 1:6
    plot(A(:,i),'.-k');
    
    plot(xlim,30*[1 1],':k','linewidth',1.5);
    plot(xlim,-30*[1 1],':k','linewidth',1.5);
    
    grid on
end
hold off
ylabel('a');
title('A');


