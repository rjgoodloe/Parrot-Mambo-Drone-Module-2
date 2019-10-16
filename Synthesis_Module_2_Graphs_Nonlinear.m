close all;
Zned = 1;
w = 2;
theta = 3;
q = 4;
phi = 5;
p = 6;
psi = 7;
r = 8;
Xned = 9;
u = 10;
Yned = 11;
v = 12;

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [-1,0,0,0,0,0,0,0,0,0,0,0];

[t_out, s_traj] = ode45(@(t,s) nonlinear_dynamics_2(t,s),tspan,x_0)


% plots
figure(1)
plot(t_out,s_traj(:,Zned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in the z-axis: Zned');
title('Nonlinear Trajectory')
axis([0 6 -2 0])
saveas(gcf,'Zned_nonlinear_ODE45_rudder', 'png')

figure(2)
plot(t_out,s_traj(:,w),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame Z axis: w');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'w_nonlinear_ODE45_rudder', 'png')

figure(3)
plot(t_out,s_traj(:,theta),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Pitch Angle: Theta');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'theta_nonlinear_ODE45_rudder', 'png')

figure(4)
plot(t_out,s_traj(:,q),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Pitch: q');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'q_nonlinear_ODE45_rudder', 'png')

figure(5)
plot(t_out,s_traj(:,phi),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Roll Angle: phi');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'phi_nonlinear_ODE45_rudder', 'png')

figure(6)
plot(t_out,s_traj(:,p),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Roll: p');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'p_nonlinear_ODE45_rudder', 'png')

figure(7)
plot(t_out,s_traj(:,psi),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Yaw Angle: psi');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'psi_nonlinear_ODE45_rudder', 'png')

figure(8)
plot(t_out,s_traj(:,r),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Yaw: r');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'r_nonlinear_ODE45_rudder', 'png')

figure(9)
plot(t_out,s_traj(:,Xned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in x axis: Xned');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'Xned_nonlinear_ODE45_rudder', 'png')

figure(10)
plot(t_out,s_traj(:,u),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame x axis: u');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'u_nonlinear_ODE45_rudder', 'png')

figure(11)
plot(t_out,s_traj(:,Yned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in the y-axis: Yned');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'Yned_nonlinear_ODE45_rudder', 'png')

figure(12)
plot(t_out,s_traj(:,v),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame Y axis: v');
title('Nonlinear Trajectory')
axis([0 6 -1 1])
saveas(gcf,'v_nonlinear_ODE45_rudder', 'png')


%Output
% t_out, 
% s_traj
% 
% m = 0.075
% s_traj(:,)
% Accels = (F_T + F_a)/m


