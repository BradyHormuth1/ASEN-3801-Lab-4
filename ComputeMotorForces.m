function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% ComputeMotorForces: Computes the feedback rotor forces based on the
% distance between each motor and the center of mass and the control moment
% coefficient
%
% Inputs:
%       Fc: Control Force Vector
%       Mc: Control Moment Vector
%       d: Distance from center of each rotor to quadcopter center of mass
%       km: Control Moment Coefficient

    ControlVec = [Fc(3),Gc(1),Gc(2),Gc(3)];
    
    % rotation matrix for rotor force feedback control
    A_matrix = [-1,-1,-1,-1;
        -d/sqrt(2),-d/sqrt(2),d/sqrt(2),d/sqrt(2);
        d/sqrt(2),-d/sqrt(2),-d/sqrt(2),d/sqrt(2);
        km, -km, km, -km];

    motor_forces = A_matrix\ControlVec;


end