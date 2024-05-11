tbl = readtable('Verification.xlsx'); % Verification
tbl1 = readtable('Identification.xlsx'); % Identification
clc
t = (tbl.time - 9)/1000; 
u = tbl.speedu;
target = tbl.target;
actual = tbl.actual;
t_new = linspace(min(t), max(t), length(t));
u_new = interp1(t, u, t_new, 'linear');

u1 = tbl1.speedu;
target1 = tbl1.target;
actual1 = tbl1.actual;
t1 = (tbl1.time-10)/1000;

num = 44.05;
den = [1 65.45 0.00012];
tf1 = tf(num,den);

idres = lsim(u_new,tf1,t_new);
% figure;
% subplot(2,1,1);
% plot(t1, u1, 'LineWidth', 2);
% grid on; 
% ylabel('Controller Effort u', 'FontSize', 12); 
% xlabel('Time (s)', 'FontSize', 12); 
% title('Controller Effort u', 'FontSize', 12); 
% ylim([-10 350]);
% 
% subplot(2,1,2); 
% plot(t1, actual1, 'LineWidth', 2); 
% grid on; 
% ylabel('Number of Steps', 'FontSize', 12); 
% xlabel('Time (s)', 'FontSize', 12); 
% title('Extractor Position In Steps', 'FontSize', 12); 

% figure;
% plot(t,target,'black','LineWidth',2);
% hold on;
% plot(t,actual,'blue','LineWidth',2);
% hold on;
% plot(t_new,idres,'red--','LineWidth',2);
% hold on;
% legend('Target Steps','Response of Actual Build','Response of Identified Transfer Function, 93.87% Fit','fontsize',10,'location','best')
% 
% ylabel('Number of Steps','fontsize',12);
% xlabel('Time (s)','fontsize',12);
% title('Responses of Actual and Mathematical Model','fontsize',12);
% grid on 

figure;
sta = readtable('stall.xlsx');
time = sta.time/1000;
actt = sta.actual;
plot(time,actt,'black','LineWidth',2);
grid on
ylabel('Number of Steps', 'FontSize', 12); 
xlabel('Time (s)', 'FontSize', 12); 
title('Open Loop Response When Stalled', 'FontSize', 12); 