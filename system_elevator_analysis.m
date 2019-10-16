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
   
                                
                                

A_elevator = [A(3:4,3:4)  A(3:4,9:10) ; A(9:10,3:4) A(9:10,9:10)]
           
B_elevator = [B(3:4,2) ; B(9:10,2)]

C_elevator = [eye(4); 0 0 0 0]

D_elevator = [0 0 0 0]

% eienvectors and eigenvalues
[V,J] = jordan(A_elevator)

% state transition matrix
syms t
matrix_exp = expm(J*t)

% modes
mode1 = matrix_exp * V(:,1)
mode2 = matrix_exp * V(:,2)
mode3 = matrix_exp * V(:,3)
mode4 = matrix_exp * V(:,4)

% controllability
W = [ B_elevator A_elevator*B_elevator A_elevator^2*B_elevator A_elevator^3*B_elevator ]
if rank(W) == 4
    fprintf('Controllable')
else
    fprintf('Not Controllable')
end


% controllability
Q = [ C_elevator; C_elevator*A_elevator; C_elevator*A_elevator^2; C_elevator*A_elevator^3]
if rank(Q) == 4
    fprintf('Observable')
else
    fprintf('Not Observable')
end

%Elevator%

% time step
dT = 0.1;

% time span
tspan = 0:dT:6;

x_0 = [0,0,0,0];


x0_vec = -10:1:10;
y0_vec = -10:1:10;

for ii = 1 : length(x0_vec)
    for jj = 1 : length(y0_vec)
        
        x_0 = [x0_vec(ii) ; y0_vec(jj); x0_vec(ii) ; y0_vec(jj)];
        [t_out, s_traj] = ode45(@(t,s) linear_dynamics_elevator(t,s),tspan,x_0);

        theta = 1;
        q = 2;
        Xned = 3;
        u = 4

        % plots
        figure(1)
        plot(s_traj(:,theta),s_traj(:,q),'LineWidth',2);
        hold on;
        shg;
        drawnow;
        grid on
        xlabel('Pitch Angle: Theta');
        ylabel('Pitch: q');
        title('Linear Trajectory')
    end
end
saveas(gcf,'phase_portrait_elevator', 'png')