%% System validation
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('Datos_Prueba1.mat')


%% REFERENCE SIGNALS
ul_ref = vf_ref;
um_ref = vl_ref;
un_ref = ve_ref;
w_ref = w_ref;

%% REAL SYSTEM VELICITIES
ul = vf(1,1:length(ul_ref));
um = vl(1,1:length(um_ref))+eps*randn(size(ul));
un = ve(1,1:length(un_ref))+eps*randn(size(ul));
w = w(1,1:length(w_ref))+eps*randn(size(ul));

%% REAL SYSTEM ACCELERATIONS
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

v = [ul; um; un; w];
vp = [ulp; ump; unp; wp];
vref = [ul_ref; um_ref; un_ref; w_ref];

v_estimate1 = v(:,1);

A = [0.9655   -0.0033   -0.0762    0.0199;...
    0.0042    0.9430   -0.0189    0.0037;...
    0.0012    0.0007    0.6632    0.0087;...
   -0.0024   -0.0011   -0.0445    0.146];

B = [0.0945   -0.0095    0.0750   -0.0269;...
    0.0020    0.0925    0.0256   -0.0016;...
    0.0007   -0.0020    0.3397   -0.0106;...
    0.0033    0.0044    0.0644    0.8715];

for k=1:length(t)
    v_estimate1(:, k+1) = A*v_estimate1(:,k)+B*vref(:,k);

end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul_ref)),ul_ref,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,ul,'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{lref}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t(1:length(ul_ref)),um_ref,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{mref}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,3)
plot(t(1:length(ul_ref)),un_ref,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{nref}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,4)
plot(t(1:length(ul_ref)),w_ref,'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\omega_{ref}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng Data_validation
print -depsc Data_validation

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t,v_estimate1(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,ul,'--','Color',[226,76,44]/255,'linewidth',1); hold on
%plot(t(1:length(ul_ref)),ul_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{lm}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Dynamic Model Identification}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t,v_estimate1(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
%plot(t(1:length(ul_ref)),um_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{mm}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,3)
plot(t,v_estimate1(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
%plot(t(1:length(ul_ref)),un_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{nm}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,4)
plot(t,v_estimate1(4,1:length(t)),'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
%plot(t(1:length(ul_ref)),w_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\omega_{m}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng Data_model1
print -depsc Data_model1