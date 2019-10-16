function ddt_states = linear_dynamics_elevator(t,s)
  
    
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


    A_elevator = [A(3:4,3:4)  A(3:4,9:10) ; A(9:10,3:4) A(9:10,9:10)];
           
    B_elevator = [B(3:4,2) ; B(9:10,2)];

    
    E = 0.1;
    
    
    ddt_states = A_elevator*[s(1);s(2);s(3);s(4)] + B_elevator*E;


    end