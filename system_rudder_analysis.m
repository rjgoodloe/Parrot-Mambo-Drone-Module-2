
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
                                    1 0 0 0];
   
                                
                                

A_rudder = A(7:8,7:8);
           
B_rudder = B(7:8,4);

C_rudder = eye(2);

D_rudder = [0; 0];

% eienvectors and eigenvalues
[V,J] = jordan(A_rudder)

% state transition matrix
syms t
matrix_exp = expm(J*t)
% modes
mode1 = matrix_exp * V(:,1)
mode2 = matrix_exp * V(:,2)

% controllability
W = [ B_rudder A_rudder*B_rudder ]
if rank(W) == 2
    fprintf('Controllable')
else
    fprintf('nNot Controllable')
end

% controllability
Q= [ C_rudder; C_rudder*A_rudder ]
if rank(Q) == 2
    fprintf('Observable')
else
    fprintf('Not observable')
end

%%Rudder%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;


x0_vec = -10:1:10;
y0_vec = -10:1:10;

for ii = 1 : length(x0_vec)
    for jj = 1 : length(y0_vec)
        
        x_0 = [x0_vec(ii) ; y0_vec(jj)];
        [t_out, s_traj] = ode45(@(t,s) linear_dynamics_rudder(t,s),tspan,x_0);

        psi = 1;
        r = 2;

        % plots
        figure(1)
        plot(s_traj(:,psi),s_traj(:,r),'LineWidth',2);
        hold on;
        shg;
        drawnow;
        grid on
        xlabel('Yaw Angle: psi');
        ylabel('Yaw: r');
        title('Linear Trajectory')        
    end
end

saveas(gcf,'phase_portrait_rudder', 'png')
