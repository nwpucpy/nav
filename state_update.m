function[ Xk]=  state_update(ekf_param,imu)
    global g0
    gro = imu.grob_rad;
    gro_pre = imu.grob_rad_pre;
    acc = imu.accb_m_s;
    Xk_1 = ekf_param.Xk_1;
    wie = imu.earthRateNED;
    dt = imu.dt_imu;
    
    quat_old = Xk_1(1:4);
    vel_old = Xk_1(5:7);
    pos_old = Xk_1(8:10);
    gyro_bias = Xk_1(11:13);
    acc_bias = Xk_1(14:16);

    Tbn = Quat2Tbn(quat_old);
    deltaAngle_rad = (gro - gyro_bias) + 1/12*cross(gro_pre,gro) - Tbn'*( wie  * dt );
    deltaAngle_norm = norm(deltaAngle_rad);
    delQuat = [cos(0.5*deltaAngle_norm);deltaAngle_rad/deltaAngle_norm*sin(0.5*deltaAngle_norm)];

    qNew = QuatMult(quat_old,delQuat);
    qNew_norm = NormQuat(qNew);

    deltaVelocity = acc - acc_bias;
    % define the velocity update equations
    vel_new = vel_old + Tbn*deltaVelocity + [0;0;g0]*dt  ;
%     vNew = vold + Tbn_true*deltaVelocity*dt - [0;0;-g0]*dt  ;

    posNew = pos_old + (vel_new + vel_old) * 0.5 *dt;

    gro_b_new = gyro_bias;
    acc_b_new = acc_bias;
    
    Xk(1:4,1) = qNew_norm;
    Xk(5:7,1) = vel_new;
    Xk(8:10,1) = posNew;
    Xk(11:13,1) = gro_b_new;
    Xk(14:16,1) = acc_b_new;

end