function ekf_param = ekf_param_init(traj_ned,imu)
    global Re e wie g0 deg 
    ekf_param.dt_ekf = 0.1;
    %�����ʼ��̬��Ϣ�������ʼ��̬���
    att0_rad = [ traj_ned(1,4) , traj_ned(1,3)  ,  traj_ned(1,2) ] ;%rad
    Q0 =   Eul2Quat(att0_rad);
    
    % ��״̬����ֵ
    ekf_param.Xk_1=zeros(16,1);
    ekf_param.Xk_1(1:4)=Q0;
    ekf_param.Xkkk_1 = ekf_param.Xk_1;
    % ��P��ֵ
    Q0 =   Eul2Quat([5,5,5]*deg);
    Xvar = [Q0(2),Q0(2),Q0(3),Q0(4),0.3,0.3,0.5,3,3,5,0.9*deg * imu.dt_imu ,0.9*deg *imu.dt_imu,0.9*deg *imu.dt_imu,0.5*imu.dt_imu ,0.5*imu.dt_imu,0.5 *imu.dt_imu];
    %IMU�������ӼƲ���Ϊ������ƫ��1�������ݲ���Ϊ������ƫ��3������P��Ŵ���Ϊ�˹�����ƫ����
    %����������1��sigma��ͬ
    ekf_param.Pk_1 = diag(Xvar.^2);
    ekf_param.imuNoise = [0.05*deg,0.05*deg,0.05*deg,0.05,0.05,0.05]*imu.dt_imu;
    %����imu����1��sigma����
    % imuNoise = [0.03*deg,0.03*deg,0.03*deg,0.1,0.1,0.1]*dt_imu;%�ɹ��Ƴ���ƫ
    % ��Q��ֵ
    Q_var2 = [0.001* deg,0.001* deg,0.001* deg,0.001* deg,0.002,0.002,0.002,0.01,0.01,0.01,0.005*deg ,0.005*deg ,0.005*deg ,0.005,0.005,0.005] *imu.dt_imu;
    %IMU�������ٱ�imuNoiseСһ��������
    %�����Ĳ������ٱ���������1��sigmaСһ����2��������
    %��Ԫ����������С
    ekf_param.Q_noise = diag(Q_var2.^2)*1;
    % ��R��ֵ     
    Zvar = [0.1,0.1,0.1,1,1,3,0.3*deg];
    % Zvar = [0.3,0.3,0.3,3,3,5];
    % Zvar = [0.1,0.1,0.1];
    ekf_param.R = diag(Zvar.^2);

end