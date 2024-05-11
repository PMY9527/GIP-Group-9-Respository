clc
tbl = readtable('Verification.xlsx'); % kp = 10
t = (tbl.time - 9)/1000; 
u = tbl.speedu;

target = tbl.target;
actual = tbl.actual;
statex2 = tbl.rateu;

fc = 50; % Cut-off frequency (in Hz)
fs = 2000; % Sampling frequency (in Hz)
order = 2; % Filter order

%Low-pass Butterworth filter
[b, a] = butter(order, fc / (fs / 2), 'low');

% Apply the filter to statex2
filtered_statex2 = filtfilt(b, a, statex2);

figure;
plot(t,filtered_statex2,'blue','LineWidth',2);
hold on;
plot(out.x2.time,out.x2.signals(1).values,'green:','LineWidth',2); % Generated from simulink
hold on;
plot(out.x2.time,out.x2.signals(2).values(:,2),'black--','LineWidth',2);
xlim([0 4]);
plot(t,actual*1.5,'red-.','linewidth',2);
legend('Actual Velocity Using Finite Differences with Low-pass Filter','Controller Effort u','Theoretical Velocity From Linear State-space Model','Actual Position, Scaled by 1.5','Location','best','fontsize',14,'Location', 'southoutside')
xlabel('Time (s)','FontSize',14);
ylabel('Number of Steps','FontSize',14)
title('Velocity of Nonlinear Actual System and Linear State-space Model','FontSize',14);
grid on
