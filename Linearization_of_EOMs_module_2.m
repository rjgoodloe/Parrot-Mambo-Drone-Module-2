close all;
clear;

syms phi theta psi p q r u v w x y z 
syms Xned Yned Zned %state variables
syms w1 w2 w3 w4  % motor speeds
syms m g Jxx Jyy Jzz b rho D Ct Cn d L_T F_T M_T N_A%parameters

% moments of inertia kgm^2
Jxy = 0;
Jxz = 0;
Jyx = 0;
Jyz = 0;
Jzx = 0;
Jzy = 0;

w_hover = sqrt((m*g)/(4*Ct*rho*D^4));

F_T(w) = Ct*rho*(w^2)*D^4
% force of thrust is negative because z defined as down
F_Ti = -[ 0;
          0; 
          F_T(w1) + F_T(w2) + F_T(w3) + F_T(w4)];


L_T(w) = Ct*rho*(w^2)*D^4*b;
M_T(w) = Ct*rho*(w^2)*D^4*b;
N_A(w) = Cn*rho*(w^2)*D^4*d;

Mt =  [  (L_T(w1) - L_T(w2) - L_T(w3) + L_T(w4)); 
         (M_T(w1) + M_T(w2) - M_T(w3) - M_T(w4));
        (-N_A(w1) + N_A(w2) - N_A(w3) + N_A(w4))]
        
F_g = m*g*[ -sin(theta); 
            cos(theta)*sin(phi);
            cos(phi)*cos(theta)];
        
J = [ Jxx Jxy Jxz; Jyx Jyy Jyz; Jzx Jzy Jzz];

Rphi = [ 1 0 0; 
        0 cos(phi) sin(phi); 
        0 -sin(phi) cos(phi)];

Rtheta = [  cos(theta) 0 -sin(theta); 
            0 1 0;
            sin(theta) 0 cos(theta)];

Rpsi = [cos(psi) sin(psi) 0; 
        -sin(psi) cos(psi) 0; 
        0 0 1];

R =(Rpsi.')*(Rtheta.')*(Rphi.')*[u; v; w];

Xned = R(1);
Yned = R(2);
Zned = R(3);

vdots = (F_g/m + F_Ti/m - cross([ p; q; r], [u; v; w]));

udot = vdots(1);
vdot = vdots(2);
wdot = vdots(3);

wdots = J\(Mt-cross([ p; q; r], J*[p; q; r]));

pdot = wdots(1);
qdot = wdots(2);
rdot = wdots(3);

phidot = p + tan(theta)*(q*sin(phi) + r*cos(phi));

thetadot = q*cos(phi) - r*sin(phi);

psidot = (q*sin(phi) + r*cos(phi))/cos(theta);



A_nonlinear = [Zned;
               wdot;
               thetadot;
               qdot;
               phidot
               pdot;
               psidot;
               rdot;
               Xned;
               udot;
               Yned;
               vdot]
    
% linearization via jacobian
linearized_sys = jacobian(A_nonlinear, [z w theta q phi p psi r x u y v]);

A_lin = eval(subs(linearized_sys,[z, w, theta, q, phi, p, psi, r, x, u, y, v],[1,0,0,0,0,0,0,0,0,0,0,0]))

B_lin = jacobian(A_nonlinear, [ w1 w2 w3 w4 ])

MotorMix = [     1  1  1  -1;
                 1  1 -1   1;
                 1 -1 -1  -1;
                 1 -1  1   1];
B_lin = B_lin*MotorMix
B_lin = eval(subs(B_lin,[ w1, w2, w3, w4], [w_hover, w_hover, w_hover, w_hover]))

output_matrix = [udot;vdot;wdot] + cross([p;q;r],[u;v;w]) - F_g/m;
output_matrix = [-z; output_matrix(1); output_matrix(2); output_matrix(3)];

linearized_C_matrix = jacobian(output_matrix, [z w theta q phi p psi r x u y v w1 w2 w3 w4]);
C_lin = eval(subs(linearized_C_matrix,[z, w, theta, q, phi, p, psi, r, x, u, y, v, w1, w2, w3, w4],[1,0,0,0,0,0,0,0,0,0,0,0, w_hover, w_hover, w_hover, w_hover]));

D_lin = C_lin(:,13:16)
C_lin = C_lin(:,1:12);

D_lin = D_lin*MotorMix

A_lin 
B_lin
C_lin
D_lin

