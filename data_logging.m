load('RSdata')
close all

Fs = 200;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1600;             % Length of signal
t = (0:L-1)*T;        % Time vector

wdot = rt_accel_z.signals.values;
wdot_filtered = rt_accel_z1.signals.values;

w = rt_w.signals.values;
w_filtered = rt_w1.signals.values;

p = rt_p.signals.values;
p_filtered = rt_p1.signals.values;

phi = rt_phi.signals.values;
phi_filtered = rt_phi1.signals.values;

q = rt_q.signals.values;
q_filtered = rt_q1.signals.values;

theta = rt_theta.signals.values;
theta_filtered = rt_theta1.signals.values;

r = rt_r.signals.values;
r_filtered = rt_r1.signals.values;

psi = rt_psi.signals.values;
psi_filtered = rt_psi1.signals.values;

udot = rt_accel_x.signals.values;
udot_filtered = rt_accel_x1.signals.values;

u = rt_u.signals.values;
u_filtered = rt_u1.signals.values;

vdot = rt_accel_y.signals.values;
vdot_filtered = rt_accel_y1.signals.values;

v = rt_v.signals.values;
v_filtered = rt_v1.signals.values;



states = [w wdot p phi q theta r psi u udot v vdot ...
    w_filtered wdot_filtered p_filtered phi_filtered q_filtered theta_filtered ...
    r_filtered psi_filtered u_filtered udot_filtered v_filtered vdot_filtered];
names = {'w', 'wdot', 'p', 'phi', 'q', 'theta', 'r', 'psi', 'u', 'udot', 'v', 'vdot', ...
    'w_filtered', 'wdot_filtered', 'p_filtered', 'phi_filtered', 'q_filtered', 'theta_filtered', ... 
    'r_filtered', 'psi_filtered', 'u_filtered', 'udot_filtered', 'v_filtered vdot_filtered'};

% for i = 1:12
%     figure(i);
%     state = states(:,i);
%     plot(rt_tout, state, 'b');
%     xlabel('Time');
%     ylabel('Signal Value');
%     hold on
%     
%     state_filtered = states(:,i + 12);
%     plot(rt_tout, state_filtered,'r');
%     title(names{i});
%     legend('original','filtered')
%     saveas(gcf, names{i}, 'png');
% 
% end

%Fast Fourier Transform 
% used to determine cut off frequency for low pass filter

for j = 1:12
    state = states(:,j);
    Y = fft(state);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);

    f = Fs*(0:(L/2))/L;
    figure(12 + j)
    plot(f,P1,'b') 
    title(names{j})
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
    hold on
    state_filtered = states(:,j + 12);
    
    Y = fft(state_filtered);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);

    f = Fs*(0:(L/2))/L;
    plot(f,P1,'r') 
    title(names{j});
    legend('original','filtered')
    saveas(gcf, names{j}, 'png');
end