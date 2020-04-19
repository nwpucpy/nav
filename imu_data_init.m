function imu = imu_data_init(kk_imu,traj_ned,dt_imu)

     global Re e wie g0 deg 
     imu.dt_imu = dt_imu;
     imu.kk_imu =kk_imu;%是否加入IMU数据误差 0：不加误差 1：加入误差
    %注入IMU数据误差%加入0.3°/s的零偏   
    imu.gro_bias =  imu.kk_imu  * 0.3*  [  1 ;1 ;1 ]   ;
    imu.gro_noise = imu.kk_imu * 0.05 ;
    % 加入0.2m/s^2零偏
    imu.acc_bias =  imu.kk_imu * 0.5*  [  1 ; 1 ; 1 ]  ;
    imu.acc_noise =  imu.kk_imu * 0.05;
    %加入磁力计误差50mguss零偏
    mag_bias = imu.kk_imu * 20 ;
    mag_noise =  imu.kk_imu * 5;
   %计算初始速度位置信息
    lat0_rad = traj_ned(1,8) ;%rad
    %计算地球自转
    earthRateECEF = single([0; 0;  wie]);
    imu.earthRateNED  = ...
    single([cos(lat0_rad)*earthRateECEF(3); ...
    0; ...
    -sin(lat0_rad)*earthRateECEF(3)]);%rad

    imu.grob_rad_pre = zeros(3,1);

end