clear;
clc;

A = [0 1;-0.00012 -65.45];
B = [0 ; 44.05];
C = [1 0];
D = 0;

sysol  = ss(A,B,C,D);

%%%%%%% POLE PLACEMENT %%%%%%% 
p  = [-65.45, -15.5];
K = place(A,B,p);
syscl = ss(A-B*K,B,C,D);
stepin = stepinfo(syscl);

Kr = (1/stepin.Peak);

%%%%%%% LQR %%%%%%% 
Q = diag([1,0.0001]); % PENALISE Q11(POSITIONING ERROR) Q22 (SPEED)
R = 0.01; % PENALISE CONTROLLER EFFORT
Klqr = lqr(sysol,Q,R);
syslqr = ss(A-B*Klqr,B,C,D);
stepinn = stepinfo(syslqr);
Krlqr = (1/stepinn.Peak);
