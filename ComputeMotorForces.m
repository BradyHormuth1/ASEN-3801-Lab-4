function motor_forces = ComputeMotorForces(Fc, Gc, d, km)


    ControlVec = [Fc(3),Gc(1),Gc(2),Gc(3)];

    A_matrix = [-1,-1,-1,-1;
        -d/sqrt(2),-d/sqrt(2),d/sqrt(2),d/sqrt(2);
        d/sqrt(2),-d/sqrt(2),-d/sqrt(2),d/sqrt(2);
        km, -km, km, -km];

    motor_forces = A_matrix\ControlVec;


end