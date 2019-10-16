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

[t_out, s_traj] = ode45(@(t,s) linear_dynamics(t,s),tspan,x_0);


% plots
figure(1)
plot(t_out,s_traj(:,Zned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in the z-axis: Zned');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'Zned_Linear_ODE45_aileron', 'png')

figure(2)
plot(t_out,s_traj(:,w),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame Z axis: w');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'w_Linear_ODE45_aileron', 'png')

figure(3)
plot(t_out,s_traj(:,theta),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Pitch Angle: Theta');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'theta_Linear_ODE45_aileron', 'png')

figure(4)
plot(t_out,s_traj(:,q),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Pitch: q');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'q_Linear_ODE45_aileron', 'png')

figure(5)
plot(t_out,s_traj(:,phi),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Roll Angle: phi');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'phi_Linear_ODE45_aileron', 'png')

figure(6)
plot(t_out,s_traj(:,p),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Roll: p');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'p_Linear_ODE45_aileron', 'png')

figure(7)
plot(t_out,s_traj(:,psi),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Yaw Angle: psi');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'psi_Linear_ODE45_aileron', 'png')

figure(8)
plot(t_out,s_traj(:,r),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Yaw: r');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'r_Linear_ODE45_aileron', 'png')

figure(9)
plot(t_out,s_traj(:,Xned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in x axis: Xned');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'Xned_Linear_ODE45_aileron', 'png')

figure(10)
plot(t_out,s_traj(:,u),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame x axis: u');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'u_Linear_ODE45_aileron', 'png')

figure(11)
plot(t_out,s_traj(:,Yned),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Position in the y-axis: Yned');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'Yned_Linear_ODE45_aileron', 'png')

figure(12)
plot(t_out,s_traj(:,v),'LineWidth',2);  
hold on;
shg;
drawnow;
grid on
xlabel('Time');
ylabel('Velocity in the body reference frame Y axis: v');
title('Linear Trajectory')
axis([0 6 -5 5])
saveas(gcf,'v_Linear_ODE45_aileron', 'png')




