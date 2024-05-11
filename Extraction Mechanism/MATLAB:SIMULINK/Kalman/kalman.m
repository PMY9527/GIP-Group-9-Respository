noKalman = readtable('WithOutKalmanFilter.xlsx');
time = noKalman.t/1000;
actual = noKalman.actual;
u = noKalman.u;

plot(time,u,'LineWidth',2);
hold on 
plot(time,actual*15,'LineWidth',2); % *15

xlim([0 5]);
grid on
legend('Scaled Position Response','Controller Effort','fontsize',12)
xlabel('Time (s)','fontsize',12);
ylabel('Number of Steps','fontsize',12);
title('Step Response Without Kalman Filter','fontsize',12)

yesKalman = readtable('withKalman.xlsx');
time = yesKalman.time/1000;
tar = yesKalman.target;
act = yesKalman.tot;
fil = yesKalman.fil;
figure;
plot(time,tar,'--','LineWidth',3);
hold on 
plot(time,act,':','LineWidth',3);
hold on
%plot(time,fil,'LineWidth',2);
xlim([0 3])
xlabel('Time (s)','fontsize',12);
ylabel('Number of Steps','fontsize',12);
title('Step Response With Kalman Filter','fontsize',12)
legend('Setpoint','Actual Response','Estimated Response','fontsize',12,'location','best')
grid on
