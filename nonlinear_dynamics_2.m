function ddt_states = nonlinear_dynamics_2(t,s)
    w = s(2);
    theta = s(3);
    q = s(4);
    phi = s(5);
    p = s(6);
    psi = s(7);
    r = s(8);
    u = s(10);
    v = s(12);
    
    Jxx = 0.0000582857; 
    Jyy = 0.0000716914;
    Jzz = 0.0001;
    Jxy = 0;
    Jxz = 0;
    Jyx = 0;
    Jyz = 0;
    Jzx = 0;
    Jzy = 0;

    b = 0.047625; % distance from CoM to center of rotor, y-axis in meters
    d = sqrt(2)*b; % perpendiculer distance from CoM to center of rotor in meters
    rho = 1.22495238; %air density kg / (m^3)
    D = 0.066; % prop diameter 66 mm
    Ct = 0.075; %Coefficient of thrust from graph, lecture 8
    Cn = 0.0072;%7.0322e-04; % Coefficient of yawing moment
    g = 9.81;
    m = 0.075;
    w_hover = sqrt((m*g)/(4*Ct*rho*D^4));

    w1 = w_hover;
    w2 = w_hover;
    w3 = w_hover;
    w4 = w_hover;
  
    if t > 1.0 && t < 5.0
        w1 = w_hover - 0.25*t + 0.25;
        w2 = w_hover + 0.25*t - 0.25;
        w3 = w_hover - 0.25*t + 0.25;
        w4 = w_hover + 0.25*t - 0.25;
    end
    
   % force of thrust is negative because z defined as down
    F_Ti = -[ 0;
              0; 
              Ct*rho*(w1^2)*D^4 + Ct*rho*(w2^2)*D^4 + Ct*rho*(w3^2)*D^4 + Ct*rho*(w4^2)*D^4];

    Mt =   [  b*(Ct*rho*(w1^2)*D^4 - Ct*rho*(w2^2)*D^4 - Ct*rho*(w3^2)*D^4 + Ct*rho*(w4^2)*D^4); ...
              b*(Ct*rho*(w1^2)*D^4 + Ct*rho*(w2^2)*D^4 - Ct*rho*(w3^2)*D^4 - Ct*rho*(w4^2)*D^4);
              d*(-Cn*rho*(w1^2)*D^4 + Cn*rho*(w2^2)*D^4 - Cn*rho*(w3^2)*D^4 + Cn*rho*(w4^2)*D^4)];

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


    ddt_states = [ Zned;
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
                   vdot];
    end