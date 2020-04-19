function ekf_param = ekf_param_init(traj_ned,imu)
    global Re e wie g0 deg 
    ekf_param.dt_ekf = 0.1;
    %计算初始姿态信息并加入初始姿态误差
    att0_rad = [ traj_ned(1,4) , traj_ned(1,3)  ,  traj_ned(1,2) ] ;%rad
    Q0 =   Eul2Quat(att0_rad);
    
    % 给状态赋初值
    ekf_param.Xk_1=zeros(16,1);
    ekf_param.Xk_1(1:4)=Q0;
    ekf_param.Xkkk_1 = ekf_param.Xk_1;
    % 给P阵赋值
    Q0 =   Eul2Quat([5,5,5]*deg);
    Xvar = [Q0(2),Q0(2),Q0(3),Q0(4),0.3,0.3,0.5,3,3,5,0.9*deg * imu.dt_imu ,0.9*deg *imu.dt_imu,0.9*deg *imu.dt_imu,0.5*imu.dt_imu ,0.5*imu.dt_imu,0.5 *imu.dt_imu];
    %IMU参数：加计参数为加入零偏的1倍，陀螺参数为加入零偏的3倍；将P阵放大是为了估计零偏更快
    %其他与量测1倍sigma相同
    ekf_param.Pk_1 = diag(Xvar.^2);
    ekf_param.imuNoise = [0.05*deg,0.05*deg,0.05*deg,0.05,0.05,0.05]*imu.dt_imu;
    %按照imu噪声1倍sigma设置
    % imuNoise = [0.03*deg,0.03*deg,0.03*deg,0.1,0.1,0.1]*dt_imu;%可估计出零偏
    % 给Q阵赋值
    Q_var2 = [0.001* deg,0.001* deg,0.001* deg,0.001* deg,0.002,0.002,0.002,0.01,0.01,0.01,0.005*deg ,0.005*deg ,0.005*deg ,0.005,0.005,0.005] *imu.dt_imu;
    %IMU参数至少比imuNoise小一个数量级
    %其他的参数至少比量测噪声1倍sigma小一个至2个数量级
    %四元数参数尽量小
    ekf_param.Q_noise = diag(Q_var2.^2)*1;
    % 给R阵赋值     
    Zvar = [0.1,0.1,0.1,1,1,3,0.3*deg];
    % Zvar = [0.3,0.3,0.3,3,3,5];
    % Zvar = [0.1,0.1,0.1];
    ekf_param.R = diag(Zvar.^2);

end