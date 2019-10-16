
close all;
syms m g Ct rho D b Jxx Jyy Jzz 
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

B = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
                            -1/m 0 0 0; 
                             0 0 0 0; 
                             0 b/Jyy 0 0; 
                             0 0 0 0; 
                             0 0 b/Jxx 0; 
                             0 0 0 0; 
                             0 0 0 -b/Jzz; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0; 
                             0 0 0 0];
                         

                            
C = [  -1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0];


D = -(8*Ct*D^4*rho*(w_hover))/m*[   0 0 0 0;
                                    0 0 0 0;
                                    0 0 0 0;
                                    1 0 0 0]
   
                                
                                

A_throttle = A(1:2,1:2);
           
B_throttle = B(1:2,1);

C_throttle = [-C(1,1:2); 0 1; C(1,1:2); C(4,1:2)];

D_throttle = [0; 0; 0; D(4,1)];

% eienvectors and eigenvalues
[V,J] = jordan(A_throttle)

% state transition matrix
syms t
matrix_exp = expm(J*t)
% modes
mode1 = matrix_exp * V(:,1)
mode2 = matrix_exp * V(:,2)

% controllability
W = [ B_throttle A_throttle*B_throttle ]
if rank(W) == 2
    fprintf('Controllable')
else
    fprintf('Not Controllable')
end

% controllability
Q = [ C_throttle; C_throttle*A_throttle ]
if rank(Q) == 2
    fprintf('Observable')
else
    fprintf('Not Observable')
end
rank(Q)
% time step\\dT = 0.1;

% time span
tspan = 0:dT:6;

x0_vec = -10:1:10;
y0_vec = -10:1:10;

for ii = 1 : length(x0_vec)
    for jj = 1 : length(y0_vec)
        
        x_0 = [x0_vec(ii) ; y0_vec(jj)];

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
    end
end
saveas(gcf,'phase_portrait_throttle', 'png')