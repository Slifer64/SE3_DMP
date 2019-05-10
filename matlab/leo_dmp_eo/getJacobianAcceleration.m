function J = getJacobianAcceleration( logQ, dLogQ)

    theta = norm(logQ);
    n = logQ / theta;

    J1 = -dLogQ' * (cos(theta) * n * n' + (eye(3) - n * n') * sin(theta) / theta);
    scalarJ2 = (1 / theta) * (cos(theta) - sin(theta) / theta);
    J2 = ((eye(3) - n * n') * dLogQ * n' + n * dLogQ' * (eye(3) - n * n'));
    
    J = [J1; scalarJ2 * J2];
    
end

