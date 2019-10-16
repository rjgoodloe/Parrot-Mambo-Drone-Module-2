function ddt_states = linear_dynamics(t,s)
  
    
    rho = 1.22495238; %air density kg / (m^3)
    D = 0.066; % prop diameter 66 mm
    Ct = 0.075; %Coefficient of thrust from graph, lecture 8
    g = 9.81;
    m = 0.075;
    b = 0.047625;
    Jyy= 0.0000716914;
    Jxx = 0.0000582857;
    w_hover = sqrt((m*g)/(4*Ct*rho*D^4))
    

    A_lin = [       0    1.0000         0         0         0         0         0         0         0         0         0         0;
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
                

    MotorMix = [    1  1  1  1;
                    1  1 -1 -1;
                    1 -1 -1  1;
                    1 -1  1 -1];

B_lin = w_hover*(8*Ct*rho*D^4)*[ 0 0 0 0; 
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
                             0 0 0 0]

    
    T = 0;
    E = 0;
    A = 0;
    R = 0;
    u = [ T; E; A; R];
    input = B_lin*M 
    if t > 1 && t < 5
        A = 0.1;
        u = [ T; E; A; R];
        input = B_lin*M 
    end
  

        ddt_states = A_lin*[s(1);s(2);s(3);s(4);s(5);s(6);s(7);s(8);s(9);s(10);s(11);s(12)] + input;


    end