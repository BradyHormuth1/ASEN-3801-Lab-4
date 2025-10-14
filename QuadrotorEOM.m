function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
 
    phi = var(4); % roll angle
    theta = var(5); % pitch angle
    psi = var(6); % yaw angle
    uE = var(7); % inertial x velocity
    vE = var(8); % interital y velocity
    wE = var(9); % inertial z velocity
    v_intertial = [uE; vE; wE];
    p = var(10); % roll rate
    q = var(11); % pitch rate
    r = var(12); % yaw rate
    ang_vel = [p; q; r];
    Ix = I(1);
    Iy = I(2);
    Iz = I(3);
    
    controlMomMatrix = [-1, -1, -1, -1;
    -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2);
    d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2);
              km, -km, km, -km];
    ControlMom = controlMomMatrix * motor_forces;
    Zc = ControlMom(1);
    Lc = ControlMom(2);
    Mc = ControlMom(3);
    Nc = ControlMom(4);
    
    cphi = cos(phi);
    sphi = sin(phi);
    tphi = tan(phi);
    ctheta = cos(theta);
    stheta = sin(theta);
    ttheta = tan(theta);
    cpsi = cos(psi);
    spsi = sin(psi);
    tpsi = tan(psi);
    sctheta = sec(theta);

    
    R321 = [cphi*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi;
            ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi;
            -stheta,                   sphi*ctheta,           cphi*ctheta];

    ERmatrix = [1, sphi*ttheta, cphi*ttheta;
                0,      cphi,        -sphi;
                0, sphi*sctheta, cphi*sctheta];

    inertialPos_dot = R321 * v_intertial;
    eulerAng_dot = ERmatrix * ang_vel;

    uE_dot = r*vE - q*wE - g*stheta;
    vE_dot = p*wE - r*uE + g*ctheta*sphi;
    wE_dot = q*uE - p*vE + g*ctheta*cphi + (Zc/m);
    inertialVel_dot = [uE_dot; vE_dot; wE_dot];

    p_dot = (((Iy - Iz)/Ix)*q*r) + (Lc/Ix);
    q_dot = (((Iz - Ix)/Iy)*p*r) + (Mc/Iy);
    r_dot = (((Ix - Iy)/Iz)*q*p) + (Nc/Iz);
    eulerRate_dot = [p_dot; q_dot; r_dot];

    var_dot = [inertialPos_dot; eulerAng_dot; inertialVel_dot; eulerRate_dot];
    

end