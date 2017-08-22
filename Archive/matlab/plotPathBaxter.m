A = load('../path/afile_Bx.txt');
Q = dlmread('../path/robot_paths_Bx.txt',' ',1,0);

%% 

figure(1)
subplot(311)
plot(A(:,:),'.-');
legend('a_1','a_2','a_3','a_4','a_5','a_6');

%% 
subplot(312)
plot(rad2deg(Q(:,1:6)),'.-');
legend('q^{(1)}_1','q^{(1)}_2','q^{(1)}_3','q^{(1)}_4','q^{(1)}_5','q^{(1)}_6');

subplot(313)
plot(rad2deg(Q(:,7:12)),'.-');
legend('q^{(2)}_1','q^{(2)}_2','q^{(2)}_3','q^{(2)}_4','q^{(2)}_5','q^{(2)}_6');
