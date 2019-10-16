close all;

Ti = 10;
Td = 2;
N = 3;
K=1;

% sampling time
dT = 0.005;

% transfer function of low pass filter with cut off frequency 60Hz 
H_of_s = @(s) 1/(1 + s/60)

% transfer function of high pass filter with cut off frequency 50Hz 
G_of_s = @(s) s/(1 + s/50)

% define s
s = tf('s');
z = tf('z', dT);

% Continuous version
continuous_version = H_of_s(s);

% Tustin's transform 
s = 2*(1-z^-1)/dT/(1+z^-1);
tf_low_pass = H_of_s(s)
[A,B,C,D] = tf2ss([0.3 0.3],[2.3 -1.7])

tf_high_pass = G_of_s(s);

%tf_high_pass*tf_low_pass



 


