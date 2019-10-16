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
   
                                
                                
A_aileron = [A(5:6,5:6) A(5:6,11:12); A(11:12,5:6) A(11:12,11:12)];
           
B_aileron = [B(5:6,3) ; B(11:12,3)];

C_aileron = [eye(4); 0 0 0 0];

D_aileron = [0 0 0 D(4,1)];

% eienvectors and eigenvalues
[V,J] = jordan(A_aileron);

% state transition matrix
syms t;
matrix_exp = expm(J*t);

% modes
mode1 = matrix_exp * V(:,1)
mode2 = matrix_exp * V(:,2)
mode3 = matrix_exp * V(:,3)
mode4 = matrix_exp * V(:,4)


% controllability
W = [ B_aileron A_aileron*B_aileron A_aileron^2*B_aileron A_aileron^3*B_aileron ]
if rank(W) == 4
    fprintf('Controllable')
else
    fprintf('Not Controllable')
end


% controllability
Q = [ C_aileron; C_aileron*A_aileron; C_aileron*A_aileron^2; C_aileron*A_aileron^3 ]
if rank(Q) == 4
    fprintf('Observable')
else
    fprintf('Not Observable')
end

%Aileron%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [0,0,0,0];

phi = 1;
p = 2;
Yned = 3;
v = 4;

x0_vec = -10:1:10;
y0_vec = -10:1:10;

for ii = 1 : length(x0_vec)
    for jj = 1 : length(y0_vec)

        x_0 = [x0_vec(ii) ; y0_vec(jj); x0_vec(ii) ; y0_vec(jj)];
        [t_out, s_traj] = ode45(@(t,s) linear_dynamics_aileron(t,s),tspan,x_0);

        % plots
        figure(1)
        plot(s_traj(:,phi),s_traj(:,p),'LineWidth',2);
        hold on;
        shg;
        drawnow;
        grid on
        xlabel('Roll Angle: phi');
        ylabel('Roll: p');
        title('Linear Trajectory')
    end
end
saveas(gcf,'phase_portrait_aileron', 'png')