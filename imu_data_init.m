function imu = imu_data_init(kk_imu,traj_ned,dt_imu)

     global Re e wie g0 deg 
     imu.dt_imu = dt_imu;
     imu.kk_imu =kk_imu;%�Ƿ����IMU������� 0��������� 1���������
    %ע��IMU�������%����0.3��/s����ƫ   
    imu.gro_bias =  imu.kk_imu  * 0.3*  [  1 ;1 ;1 ]   ;
    imu.gro_noise = imu.kk_imu * 0.05 ;
    % ����0.2m/s^2��ƫ
    imu.acc_bias =  imu.kk_imu * 0.5*  [  1 ; 1 ; 1 ]  ;
    imu.acc_noise =  imu.kk_imu * 0.05;
    %������������50mguss��ƫ
    mag_bias = imu.kk_imu * 20 ;
    mag_noise =  imu.kk_imu * 5;
   %�����ʼ�ٶ�λ����Ϣ
    lat0_rad = traj_ned(1,8) ;%rad
    %���������ת
    earthRateECEF = single([0; 0;  wie]);
    imu.earthRateNED  = ...
    single([cos(lat0_rad)*earthRateECEF(3); ...
    0; ...
    -sin(lat0_rad)*earthRateECEF(3)]);%rad

    imu.grob_rad_pre = zeros(3,1);

end