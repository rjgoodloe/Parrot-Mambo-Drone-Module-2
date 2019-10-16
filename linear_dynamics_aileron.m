function ddt_states = linear_dynamics_aileron(t,s)
  
    
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


    A_aileron = [A(5:6,5:6) A(5:6,11:12); A(11:12,5:6) A(11:12,11:12)];
           
    B_aileron = [B(5:6,3) ; B(11:12,3)];

    
    A = 0.1;
    
    

    ddt_states = A_aileron*[s(1);s(2);s(3);s(4)] + B_aileron*A;


    end