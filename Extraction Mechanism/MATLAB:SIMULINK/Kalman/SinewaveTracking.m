psine = readtable('ff_sinewave.xlsx');
time = psine.time/1000;
tar = psine.target;
act = psine.tot;
fil = psine.fil;
u = psine.u;
plot(time,tar,'--','LineWidth',3);
hold on 
plot(time,act,':','LineWidth',3);
hold on
plot(time,fil,'LineWidth',2);
xlim([0 8])
ylim([-22.5 22.5])
xlabel('Time (s)','fontsize',12);
ylabel('Number of Steps','fontsize',12);
title('Sine Wave Tracking With Full State Feedback and Kalman Filter ','fontsize',12)
legend('Setpoint','Actual Response','Estimated Response','fontsize',12,'location','best')
grid on
