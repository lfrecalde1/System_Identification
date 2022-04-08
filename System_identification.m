%% ESTIMATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('Datos_Prueba9.mat')
clear tf;
polyorder = 2;    % Library terms polynomial order
usesine   = 0;    % Using sine functions in library
n = 4;

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
xTrain = [v;vref];
uTrain = vref;

tic 
[sysmodel_DMDc] = DMDc(xTrain(1:4,:),uTrain,ts);
toc


v_estimate1 = v(:,1);
% v_estimate2 = v(:,1);
X_k = [v(:,1:end-1);vref(:,1:end-1)];
X_k_aux = v(:,1:end-1);
Y_k = v(:,2:end);
P_k = inv(X_k*X_k');
G_k = Y_k*X_k'*inv(X_k*X_k');
A_k = G_k(:,1:4);
B_k = G_k(:,5:end);

for k=1:length(t)-1
    tic
    norma_A(k) = norm(A_k,2);
    norma_B(k) = norm(B_k,2);
    v_estimate1(:, k+1) = A_k*v_estimate1(:,k)+ B_k*vref(:,k);
    estados = [v_estimate1(:,k+1);vref(:,k)];
    
%     %% Matrix Model Matrices
%     factor = 1/(1+estados'*P_k*estados);
%     P_k =  P_k-factor*P_k*(estados*estados')*P_k;
%     G_k = G_k + factor*(v(:,k+1)-G_k*estados)*estados'*P_k;
%     A_k = G_k(:,1:4);
%     B_k = G_k(:,5:end);
toc
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

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(norma_A)),norma_A,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1,1:length(norma_A)),norma_B,'--','Color',[226,76,44]/255,'linewidth',1); hold on
%plot(t(1:length(ul_ref)),ul_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$norm_A$','$norm_B$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Dynamic Model Identification}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
% 
% subplot(4,1,2)
% plot(t,v_estimate2(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t,um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
% %plot(t(1:length(ul_ref)),um_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\mu_{mm}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
% 
% subplot(4,1,3)
% plot(t,v_estimate2(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t,un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
% %plot(t(1:length(ul_ref)),un_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\mu_{nm}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
% 
% subplot(4,1,4)
% plot(t,v_estimate2(4,1:length(t)),'Color',[83,57,217]/255,'linewidth',1); hold on
% plot(t,w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
% %plot(t(1:length(ul_ref)),w_ref,'Color',[100,100,100]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\omega_{m}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
% xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
% print -dpng Data_model2
% print -depsc Data_model2