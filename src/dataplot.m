clear;
close all;
%t[1],pos[2-4],pos_d[5-7],q[8-11],q_d[12-15],T[16],M[17-19],thetav1[20],thetav2[21-25]
%thetaw1[26-28],thetaw2[29-31],thetaw3[32-37]
sim_data_norm = readmatrix('D:/VSCode/Zoneshi/RotorCraft/out/build/x64-Debug/simple_data.dat');
sim_data_adap = readmatrix('D:/VSCode/Zoneshi/RotorCraft/out/build/x64-Debug/adaptive_data.dat');
sim_data_fix = readmatrix('D:/VSCode/Zoneshi/RotorCraft/out/build/x64-Debug/fixed_data.dat');

marker_nums = 10;
marker_step = floor(size(sim_data_adap,1)/marker_nums);
marker_index = 1:marker_step:size(sim_data_adap,1);
%%
figure("Name","XYZ3D","Position",[100,100,900,500]);
plot3(sim_data_norm(:,2),sim_data_norm(:,3),-sim_data_norm(:,4),'--s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot3(sim_data_adap(:,2),sim_data_adap(:,3),-sim_data_adap(:,4),'-.h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot3(sim_data_fix(:,2),sim_data_fix(:,3),-sim_data_fix(:,4),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot3(sim_data_adap(:,5),sim_data_adap(:,6),-sim_data_adap(:,7),'--d',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);

grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$x\rm{[m]}$','Interpreter','latex','FontSize',20);
ylabel('$y\rm{[m]}$','Interpreter','latex','FontSize',20);
zlabel('$z\rm{[m]}$','Interpreter','latex','FontSize',20);

lgd = legend('Normal','Proposed','Fixed','Command');
lgd.FontSize = 16;

%%
figure("Name","Thrust","Position",[100,100,900,500]);
plot(sim_data_norm(:,1),sim_data_norm(:,16),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,16),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot(sim_data_fix(:,1),sim_data_fix(:,16),'--h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
grid on;

ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$T\rm{[N]}$','Interpreter','latex','FontSize',20);
lgd = legend('Normal','Proposed','Fixed');
lgd.FontSize = 20;
%%
figure("Name","Torque","Position",[100,100,900,900]);
subplot(3,1,1);
plot(sim_data_norm(:,1),sim_data_norm(:,17),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,17),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot(sim_data_fix(:,1),sim_data_fix(:,17),'--h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$M_{Tx}\rm{[N\cdot m]}$','Interpreter','latex','FontSize',20);
ylim([-10,10]);
lgd = legend('Normal','Proposed','Fixed');
lgd.FontSize = 20;
lgd.NumColumns = 3;


subplot(3,1,2);
plot(sim_data_norm(:,1),sim_data_norm(:,18),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,18),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot(sim_data_fix(:,1),sim_data_fix(:,18),'--h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$M_{Ty}\rm{[N\cdot m]}$','Interpreter','latex','FontSize',20);
ylim([-8,8]);


subplot(3,1,3);
plot(sim_data_norm(:,1),sim_data_norm(:,19),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,19),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot(sim_data_fix(:,1),sim_data_fix(:,19),'--h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$M_{Tz}\rm{[N\cdot m]}$','Interpreter','latex','FontSize',20);
ylim([-8,8]);
%%
figure("Name","Thetav1","Position",[100,100,900,400]);
plot(sim_data_norm(:,1),sim_data_norm(:,20),'-.s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,20),'-h',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.9,-0.9],'--h',"LineWidth",4);
grid on;

ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
lgd = legend('Normal','Proposed','Nominal');
lgd.FontSize = 20;
ylim([-0.94,-0.86]);
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$\theta_{v1}$','Interpreter','latex','FontSize',20);
%%
figure("Name","Thetav2","Position",[100,100,900,900]);
subplot(3,1,1);
plot(sim_data_norm(:,1),sim_data_norm(:,21),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,21),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.05,-0.05],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{v21}$','Interpreter','latex','FontSize',20);
ylim([-0.1,0.05]);
lgd = legend('Normal','Proposed','Nominal');
lgd.FontSize = 20;
lgd.NumColumns = 3;


subplot(3,1,2);
plot(sim_data_norm(:,1),sim_data_norm(:,22),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,22),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.1,-0.1],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{v22}$','Interpreter','latex','FontSize',20);


subplot(3,1,3);
plot(sim_data_norm(:,1),sim_data_norm(:,23),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,23),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.1,-0.1],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$\theta_{v23}$','Interpreter','latex','FontSize',20);

%%
figure("Name","Thetaw1","Position",[100,100,900,900]);
subplot(3,1,1);
plot(sim_data_norm(:,1),sim_data_norm(:,26),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,26),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.5,-0.5],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega 11}$','Interpreter','latex','FontSize',20);

lgd = legend('Normal','Proposed','Nominal');
lgd.FontSize = 20;
lgd.NumColumns = 3;


subplot(3,1,2);
plot(sim_data_norm(:,1),sim_data_norm(:,27),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,27),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.75,-0.75],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega12}$','Interpreter','latex','FontSize',20);


subplot(3,1,3);
plot(sim_data_norm(:,1),sim_data_norm(:,28),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,28),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.75,-0.75],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$\theta_{\omega13}$','Interpreter','latex','FontSize',20);

%%
figure("Name","Thetaw2","Position",[100,100,900,900]);
subplot(3,1,1);
plot(sim_data_norm(:,1),sim_data_norm(:,29),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,29),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[0.0,0.0],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega 21}$','Interpreter','latex','FontSize',20);
ylim([-0.5,0.5]);
lgd = legend('Normal','Proposed','Nominal');
lgd.FontSize = 20;
lgd.NumColumns = 3;


subplot(3,1,2);
plot(sim_data_norm(:,1),sim_data_norm(:,30),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,30),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[0.5,0.5],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega22}$','Interpreter','latex','FontSize',20);


subplot(3,1,3);
plot(sim_data_norm(:,1),sim_data_norm(:,31),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,31),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.5,-0.5],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$\theta_{\omega23}$','Interpreter','latex','FontSize',20);

%%
figure("Name","Thetaw3","Position",[100,100,900,1200]);
subplot(5,1,1);
plot(sim_data_norm(:,1),sim_data_norm(:,32),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,32),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.05,-0.05],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega 31}$','Interpreter','latex','FontSize',20);
lgd = legend('Normal','Proposed','Nominal');
lgd.FontSize = 20;
lgd.NumColumns = 3;


subplot(5,1,2);
plot(sim_data_norm(:,1),sim_data_norm(:,33),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,33),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.05,-0.05],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega32}$','Interpreter','latex','FontSize',20);


subplot(5,1,3);
plot(sim_data_norm(:,1),sim_data_norm(:,34),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,34),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.025,-0.025],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylim([-0.075,0.05]);
ylabel('$\theta_{\omega33}$','Interpreter','latex','FontSize',20);

subplot(5,1,4);
plot(sim_data_norm(:,1),sim_data_norm(:,35),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,35),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.05,-0.05],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylabel('$\theta_{\omega34}$','Interpreter','latex','FontSize',20);


subplot(5,1,5);
plot(sim_data_norm(:,1),sim_data_norm(:,36),'-.*',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
hold on;
plot(sim_data_adap(:,1),sim_data_adap(:,36),'-s',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
plot([sim_data_adap(1,1),sim_data_adap(end,1)],[-0.025,-0.025],'--h',"LineWidth",4);
grid on;
ax = gca;
ax.TickLabelInterpreter = "latex";
ax.FontSize = 20;
ylim([-0.075,0.05]);
xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
ylabel('$\theta_{\omega35}$','Interpreter','latex','FontSize',20);
