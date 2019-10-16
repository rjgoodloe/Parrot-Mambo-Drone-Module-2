function ddt_states = linear_dynamics_throttle(t,s)
  
    
    rho = 1.22495238; %air density kg / (m^3)
    D = 0.066; % prop diameter 66 mm
    Ct = 0.075; %Coefficient of thrust from graph, lecture 8
    g = 9.81;
    m = 0.075;
    b = 0.047625;
    Jxx = 0.0000582857;
    Jyy= 0.0000716914;
    Jzz = 0.0001;
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


    A_throttle = A(1:2,1:2);

    B_throttle = B(1:2,1);

    
    T = 20;
    
    

    ddt_states = A_throttle*[s(1);s(2)] + B_throttle*T;


    end