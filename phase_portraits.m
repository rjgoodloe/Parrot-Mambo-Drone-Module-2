


close all;





%Throttle
% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [-1,0];

[t_out, s_traj] = ode45(@(t,s) linear_dynamics_throttle(t,s),tspan,x_0);

Zned = 1;
w = 2;

% plots
figure(1)
plot(s_traj(:,Zned),s_traj(:,w),'LineWidth',2);
hold on;
shg;
drawnow;
grid on
xlabel('Position in the z-axis: Zned');
ylabel('Velocity in the body reference frame Z axis: w');
title('Linear Trajectory')
axis equal tight 
saveas(gcf,'phase_portrait_throttle', 'png')

%Elevator%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [0,0,0,0];

[t_out, s_traj] = ode45(@(t,s) linear_dynamics_elevator(t,s),tspan,x_0);

theta = 1;
q = 2;
Xned = 3;
u = 4

% plots
figure(2)
plot(s_traj(:,theta),s_traj(:,q),'LineWidth',2);
hold on;
shg;
drawnow;
grid on
xlabel('Pitch Angle: Theta');
ylabel('Pitch: q');
title('Linear Trajectory')
axis equal tight 
saveas(gcf,'phase_portrait_elevator', 'png')

%Aileron%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [0,0,0,0];

[t_out, s_traj] = ode45(@(t,s) linear_dynamics_aileron(t,s),tspan,x_0);

phi = 1;
p = 2;
Yned = 3;
v = 4;

% plots
figure(3)
plot(s_traj(:,phi),s_traj(:,p),'LineWidth',2);
hold on;
shg;
drawnow;
grid on
xlabel('Roll Angle: phi');
ylabel('Roll: p');
title('Linear Trajectory')
axis equal tight 
saveas(gcf,'phase_portrait_aileron', 'png')

%Rudder%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [0, 0];

[t_out, s_traj] = ode45(@(t,s) linear_dynamics_rudder(t,s),tspan,x_0);

psi = 1;
r = 2;

% plots
figure(4)
plot(s_traj(:,psi),s_traj(:,r),'LineWidth',2);
hold on;
shg;
drawnow;
grid on
xlabel('Yaw Angle: psi');
ylabel('Yaw: r');
title('Linear Trajectory')
axis equal tight 
saveas(gcf,'phase_portrait_rudder', 'png')
