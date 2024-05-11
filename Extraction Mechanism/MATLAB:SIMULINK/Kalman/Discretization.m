clear
clc
A = [0 1;-0.00012 -65.45];
B = [0 ; 44.05];
C = [1 0];
D = 0;
sysol  = ss(A,B,C,D);
Ts = 0.003; % 0.004 for actual system
sysd = c2d(sysol, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
