close all;
rho = 1.22495238; %air density kg / (m^3)
D = 0.066; % prop diameter 66 mm
Ct = 0.075; %Coefficient of thrust from graph, lecture 8
g = 9.81;
m = 0.075;
b = 0.047625;
Jxx = 0.0000582857;
Jyy= 0.0000716914;
Jzz = 0.0001;
w_hover = sqrt((m*g)/(4*Ct*rho*D^4));

A = [       0    1.0000         0         0         0         0         0         0         0         0         0         0;
            0         0         0         0         0         0         0         0         0         0         0         0;
            0         0         0    1.0000         0         0         0         0         0         0         0         0;
            0         0         0         0         0         0         0         0         0         0         0         0;
            0         0         0         0         0    1.0000         0         0         0         0         0         0;
            0         0         0         0         0         0         0         0         0         0         0         0;
            0         0         0         0         0         0         0    1.0000         0         0         0         0;
            0         0         0         0         0         0         0         0         0         0         0         0;
            0         0         0         0         0         0         0         0         0    1.0000         0         0;
            0         0   -9.8100         0         0         0         0         0         0         0         0         0;
            0         0         0         0         0         0         0         0         0         0         0    1.0000;
            0         0         0         0    9.8100         0         0         0         0         0         0         0];

BT = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
                            -1/m 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0];
 BE = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
                            0 0 0 0; 
                             0 0 0 0; 
                             0 b/Jyy 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0];
BA = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
                            0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 b/Jxx 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0];
BR = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
                            0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 -b/Jzz; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0];
                         
C = eye(12);

sys_T = ss(A,BT,C,[],'InputName',{'T'});
sys_E = ss(A,BE,C,[],'InputName',{'E'});
sys_A = ss(A,BA,C,[],'InputName',{'A'});
sys_R = ss(A,BR,C,[],'InputName',{'R'});


s = tf('s');
low_pass_tf = 1/(1 + s/60);

sys_filter = append(low_pass_tf);

filt_freqT = sys_filter*sys_T;
filt_freqE = sys_filter*sys_E;
filt_freqA = sys_filter*sys_A;
filt_freqR = sys_filter*sys_R;
% bode(sys_T)
% hold all;
% bode(sys_E)
% hold all;
% bode(sys_A)
% hold all;
% bode(sys_R)
% hold all;
% bode(filt_freqT)
% hold all;
% bode(filt_freqE)
% hold all;
% bode(filt_freqA)
% hold all;
% bode(filt_freqR)
% hold all;

figure;
nyquist(sys_T); hold all; nyquist(filt_freqT)
xlim([-2 2]);
ylim([-2,2]);
hold all; 
nyquist(sys_E); hold all; nyquist(filt_freqE)
hold all; 
nyquist(sys_A); hold all; nyquist(filt_freqA)
hold all; 
nyquist(sys_R); hold all; nyquist(filt_freqR)








