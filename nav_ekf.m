%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 卡尔曼滤波
%状态：四元数，速度，位置，陀螺零偏，加速度计零偏
%量测：gps速度，gps位置，航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;
addpath('F:\4 导航\Study\基于UKF的组合导航算法\traj_build_v2');
%%读取数据并进行处理
%1 时间 
%2 偏航角（rad）3俯仰角（rad）4 滚转角（rad）
%5 Vn 6 Ve 7 Vd
%8Lat(rad) 9 Lon(rad) 10 ht(m)
%11Wx 12Wy 13Wz累加增量前（北X）右(东Y) 下(地Z)
%14Ax 15Ay 16Az累加增量前（X）右(Y) 下(Z)
load trajectory.mat;

glb_vars;
global Re e wie g0 deg  
%%%%%%%%%%%%%%%%%读取数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[dt_imu,angRate_deg_s,accel_m_ss,delMag] = load_data_ned(traj_ned);
%%%%%%%%%%%%%%%%%%%%数据读取完成%%%%%%%%%%%%%%%%%%%%%%%%
%初始化
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kk_imu = 1;
%imu 参数初始化
imu = imu_data_init(kk_imu,traj_ned,dt_imu);
%滤波器参数初始化
ekf_param = ekf_param_init(traj_ned,imu); 
%预分配内存
data_scale = 0.9;%取值在0-1之间，1：表示计算全部数据
data_len = fix(length(traj_ned)*data_scale/(ekf_param.dt_ekf/imu.dt_imu));
Zk_all = zeros(data_len,22);
Att_all = zeros(data_len,10);
Xk_all = zeros(data_len,33);
vel_pos = zeros(data_len,7);

j=1;
  for i=1:1:length(traj_ned) *data_scale
        %IMU数据注入误差
        imu.grob_rad =   angRate_deg_s( i , : )' * imu.dt_imu * deg  + imu.gro_bias * imu.dt_imu * deg   + imu.gro_noise * randn(3,1) * imu.dt_imu  * deg  ;
        imu.accb_m_s  =   accel_m_ss( i , : )' * imu.dt_imu    + imu.acc_bias * imu.dt_imu  + imu.acc_noise * randn(3,1) * imu.dt_imu ;         
        %计算航向角
        if traj_ned(i,2)>2*pi
          psi = mod(traj_ned(i,2),2*pi);
        else
          psi = traj_ned(i,2);
        end
        traj_ned(i,2)  =  psi;
        
        %更新状态
       ekf_param.Xkk_1 =  state_update(ekf_param,imu);      
       %更新协方差矩阵
       ekf_param.Pkk_1 = covariance_update(ekf_param,imu);       
       
       imu.grob_rad_pre = imu.grob_rad ;   
%        ekf_param.F = get_F(ekf_param,imu);
%         ekf_param.Pkk_1=ekf_param.F*ekf_param.Pk_1*ekf_param.F'+ekf_param.Q_noise; % 协方差矩阵递推 
        
        for k=1:1:length(ekf_param.Xk_1)
            Pk_all(i,k) = ekf_param.Pkk_1(k,k);
        end

        if (mod(i,ekf_param.dt_ekf/imu.dt_imu)==0)
            %计算量测矩阵
            H_VP = GetH_VP();
            H_yaw = GetH_yaw(ekf_param.Xkk_1);     
            ekf_param.H = [H_VP;H_yaw];  
            %计算量测
            v_ned = traj_ned(i,5:7)' + 0.1 * randn(3,1);
            p_n = (traj_ned(i,8) - traj_ned(1,8)) * Re + 1 * randn(1);
            p_e = (traj_ned(i,9) - traj_ned(1,9)) * Re * cos(traj_ned(i,8))+ 1 * randn(1);
            p_d = -(traj_ned(i,10) - traj_ned(1,10))+ 3 * randn(1);
            p_ned = [p_n;p_e;p_d];
            yaw = traj_ned(i,2) + 0.3 * deg * randn(1);
            ekf_param.Zk = [ v_ned;p_ned;yaw];%
            %计算量测预测
            att_predict =Quat2Eul(ekf_param.Xkk_1(1:4));
            predict_yaw = att_predict(3)*deg;%rad计算量测预测  
            ekf_param.Zkk_1 = [ ekf_param.Xkk_1(5:10);predict_yaw];
            % 计算增益矩阵
            ekf_param.K = ekf_param.Pkk_1 * ekf_param.H' * inv( ( ekf_param.R + ekf_param.H *ekf_param.Pkk_1 * ekf_param.H' ) );% 计算增益矩阵

            ekf_param.Xk=ekf_param.Xkk_1+ ekf_param.K*(ekf_param.Zk-ekf_param.Zkk_1);% 更新状态
            ekf_param.Pk=(eye(length(ekf_param.Xkk_1))-ekf_param.K*ekf_param.H)*ekf_param.Pkk_1;% 更新协方差矩阵
            % 更新状态与协方差阵
            ekf_param.Xk_1=ekf_param.Xk;
            ekf_param.Pk_1=ekf_param.Pk; 
            %转换为姿态角
            att = Quat2Eul(ekf_param.Xk(1:4));
            att1 = Quat2Eul(ekf_param.Xkkk_1(1:4));
            % 记录数据  
            Zk_all(j,1)= traj_ned(i,1) ; 
            Zk_all(j,2:22) = [ekf_param.Zk',ekf_param.Zkk_1',ekf_param.Zk'-ekf_param.Zkk_1'];
%             Zk_all(j,2:10) = [Zk',Zkk_1',Zk'-Zkk_1'];
%             Zk_all(j,2:19) = [Zk',Zkk_1',Zk'-Zkk_1'];
            Att_all(j,1) = traj_ned(i,1) ; 
            Att_all(j,2:4) = att(1:3); 
            Att_all(j,5:7) = att1(1:3); 
            Att_all(j,8:10) =  [ traj_ned(i,4) , traj_ned(i,3)  ,  traj_ned(i,2) ]/deg; 
            Xk_all (j,1) = traj_ned(i,1) ; 
            Xk_all (j,2:33) = [ekf_param.Xk',ekf_param.Xkkk_1'];
            vel_pos(j,1) = traj_ned(i,1);
            vel_pos(j,2:4) = traj_ned(i,5:7) ;
            p_n = (traj_ned(i,8) - traj_ned(1,8)) * Re ;
            p_e = (traj_ned(i,9) - traj_ned(1,9)) * Re * cos(traj_ned(i,8));
            p_d = -(traj_ned(i,10) - traj_ned(1,10));
            vel_pos(j,5:7) =[p_n,p_e,p_d] ;
            j=j+1;
        else
            
            ekf_param.Xk_1=ekf_param.Xkk_1;
            ekf_param.Pk_1=ekf_param.Pkk_1; 
            
        end
        if (0==mod(i,1000))
           i * imu.dt_imu
        end
       
  end

figure
% plot( traj_ned(:,1) , traj_ned(:,4) /deg, Att_all (:,1) , Att_all (:,2),Att_all (:,1) , Att_all (:,5),'LineWidth',2);
plot( traj_ned(:,1) , traj_ned(:,4) /deg, Att_all (:,1) , Att_all (:,2),'LineWidth',2);
grid on;
title('滚转角');
xlabel('time / s');
ylabel('Roll / °');
% legend('理论值','估计值','递推值');
legend('理论值','估计值');

figure
% plot( traj_ned(:,1) , traj_ned(:,3)/deg , Att_all (:,1) , Att_all (:,3),Att_all (:,1) , Att_all (:,6),'LineWidth',2);
plot( traj_ned(:,1) , traj_ned(:,3)/deg , Att_all (:,1) , Att_all (:,3),'LineWidth',2);
grid on;
title('俯仰角');
xlabel('time / s');
ylabel('Pitch/ °');
% legend('理论值','估计值','递推值');
legend('理论值','估计值');

figure
% plot( traj_ned(:,1) , traj_ned(:,2)/deg , Att_all (:,1) , Att_all (:,4),Att_all (:,1) , Att_all (:,7),'LineWidth',2);
plot( traj_ned(:,1) , traj_ned(:,2)/deg , Att_all (:,1) , Att_all (:,4),'LineWidth',2);
grid on;
title('偏航角');
xlabel('time / s');
ylabel('Yaw / °');
% legend('理论值','估计值','递推值');
legend('理论值','估计值');

figure
% plot( Zk_all(:,1) , Zk_all(:,2), Xk_all (:,1) , Xk_all (:,6),Xk_all (:,1) , Xk_all (:,22),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,2), Xk_all (:,1) , Xk_all (:,6),vel_pos(:,1),vel_pos(:,2),'LineWidth',2);
grid on;
title('北向速度');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
legend('量测值','估计值','理论值');

figure
% plot( Zk_all(:,1) , Zk_all(:,3) , Xk_all (:,1) , Xk_all (:,7),Xk_all (:,1) , Xk_all (:,23),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,3) , Xk_all (:,1) , Xk_all (:,7),vel_pos(:,1),vel_pos(:,3),'LineWidth',2);
grid on;
title('东向速度');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
legend('量测值','估计值','理论值');

figure
% plot( Zk_all(:,1) , Zk_all(:,4) , Xk_all (:,1) , Xk_all (:,8),Xk_all (:,1) , Xk_all (:,24),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,4) , Xk_all (:,1) , Xk_all (:,8),vel_pos(:,1),vel_pos(:,4),'LineWidth',2);
grid on;
title('地向速度');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
legend('量测值','估计值','理论值');

figure
% plot( Zk_all(:,1) , Zk_all(:,5), Xk_all (:,1) , Xk_all (:,9),Xk_all (:,1) , Xk_all (:,25),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,9),Xk_all (:,1) , Xk_all (:,25),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,5), Xk_all (:,1) , Xk_all (:,9),vel_pos(:,1),vel_pos(:,5),'LineWidth',2);
grid on;
title('北向位置');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
% legend('估计值','递推值');
legend('量测值','估计值','理论值');

figure
% plot( Zk_all(:,1) , Zk_all(:,6) , Xk_all (:,1) , Xk_all (:,10),Xk_all (:,1) , Xk_all (:,26),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,10),Xk_all (:,1) , Xk_all (:,26),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,6) , Xk_all (:,1) , Xk_all (:,10),vel_pos(:,1),vel_pos(:,6),'LineWidth',2);
grid on;
title('东向位置');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
% legend('估计值','递推值');
legend('量测值','估计值','理论值');

figure
% plot( Zk_all(:,1) , Zk_all(:,7) , Xk_all (:,1) , Xk_all (:,11),Xk_all (:,1) , Xk_all (:,27),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,11),Xk_all (:,1) , Xk_all (:,27),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,7) , Xk_all (:,1) , Xk_all (:,11),vel_pos(:,1),vel_pos(:,7),'LineWidth',2);
grid on;
title('地向位置');
xlabel('time / s');
ylabel('m/s');
% legend('量测值','估计值','递推值');
% legend('估计值','递推值');
legend('量测值','估计值','理论值');


figure
plot( Att_all (:,1) , Xk_all(:,12)/imu.dt_imu/deg ,'LineWidth',2);
grid on;
title('X轴陀螺零偏');
xlabel('time / s');
ylabel('° / s');

figure
plot( Att_all (:,1) , Xk_all(:,13) /imu.dt_imu/deg,'LineWidth',2);
grid on;
title('Y轴陀螺零偏');
xlabel('time / s');
ylabel('° / s');

figure
plot( Att_all (:,1) , Xk_all(:,14)/imu.dt_imu/deg ,'LineWidth',2);
grid on;
title('Z轴陀螺零偏');
xlabel('time / s');
ylabel('° / s');

figure
plot( Att_all (:,1) , Xk_all(:,15)/imu.dt_imu ,'LineWidth',2);
grid on;
title('X轴加计零偏');
xlabel('time / s');
ylabel('m / s^2');

figure
plot( Att_all (:,1) , Xk_all(:,16) /imu.dt_imu,'LineWidth',2);
grid on;
title('Y轴加计零偏');
xlabel('time / s');
ylabel('m / s^2');

figure
plot( Att_all (:,1) , Xk_all(:,17)/imu.dt_imu ,'LineWidth',2);
grid on;
title('Z轴加计零偏');
xlabel('time / s');
ylabel('m / s^2');





