%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������˲�
%״̬����Ԫ�����ٶȣ�λ�ã�������ƫ�����ٶȼ���ƫ
%���⣺gps�ٶȣ�gpsλ�ã������
%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;
addpath('F:\4 ����\Study\����UKF����ϵ����㷨\traj_build_v2');
%%��ȡ���ݲ����д���
%1 ʱ�� 
%2 ƫ���ǣ�rad��3�����ǣ�rad��4 ��ת�ǣ�rad��
%5 Vn 6 Ve 7 Vd
%8Lat(rad) 9 Lon(rad) 10 ht(m)
%11Wx 12Wy 13Wz�ۼ�����ǰ����X����(��Y) ��(��Z)
%14Ax 15Ay 16Az�ۼ�����ǰ��X����(Y) ��(Z)
load trajectory.mat;

glb_vars;
global Re e wie g0 deg  
%%%%%%%%%%%%%%%%%��ȡ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[dt_imu,angRate_deg_s,accel_m_ss,delMag] = load_data_ned(traj_ned);
%%%%%%%%%%%%%%%%%%%%���ݶ�ȡ���%%%%%%%%%%%%%%%%%%%%%%%%
%��ʼ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kk_imu = 1;
%imu ������ʼ��
imu = imu_data_init(kk_imu,traj_ned,dt_imu);
%�˲���������ʼ��
ekf_param = ekf_param_init(traj_ned,imu); 
%Ԥ�����ڴ�
data_scale = 0.9;%ȡֵ��0-1֮�䣬1����ʾ����ȫ������
data_len = fix(length(traj_ned)*data_scale/(ekf_param.dt_ekf/imu.dt_imu));
Zk_all = zeros(data_len,22);
Att_all = zeros(data_len,10);
Xk_all = zeros(data_len,33);
vel_pos = zeros(data_len,7);

j=1;
  for i=1:1:length(traj_ned) *data_scale
        %IMU����ע�����
        imu.grob_rad =   angRate_deg_s( i , : )' * imu.dt_imu * deg  + imu.gro_bias * imu.dt_imu * deg   + imu.gro_noise * randn(3,1) * imu.dt_imu  * deg  ;
        imu.accb_m_s  =   accel_m_ss( i , : )' * imu.dt_imu    + imu.acc_bias * imu.dt_imu  + imu.acc_noise * randn(3,1) * imu.dt_imu ;         
        %���㺽���
        if traj_ned(i,2)>2*pi
          psi = mod(traj_ned(i,2),2*pi);
        else
          psi = traj_ned(i,2);
        end
        traj_ned(i,2)  =  psi;
        
        %����״̬
       ekf_param.Xkk_1 =  state_update(ekf_param,imu);      
       %����Э�������
       ekf_param.Pkk_1 = covariance_update(ekf_param,imu);       
       
       imu.grob_rad_pre = imu.grob_rad ;   
%        ekf_param.F = get_F(ekf_param,imu);
%         ekf_param.Pkk_1=ekf_param.F*ekf_param.Pk_1*ekf_param.F'+ekf_param.Q_noise; % Э���������� 
        
        for k=1:1:length(ekf_param.Xk_1)
            Pk_all(i,k) = ekf_param.Pkk_1(k,k);
        end

        if (mod(i,ekf_param.dt_ekf/imu.dt_imu)==0)
            %�����������
            H_VP = GetH_VP();
            H_yaw = GetH_yaw(ekf_param.Xkk_1);     
            ekf_param.H = [H_VP;H_yaw];  
            %��������
            v_ned = traj_ned(i,5:7)' + 0.1 * randn(3,1);
            p_n = (traj_ned(i,8) - traj_ned(1,8)) * Re + 1 * randn(1);
            p_e = (traj_ned(i,9) - traj_ned(1,9)) * Re * cos(traj_ned(i,8))+ 1 * randn(1);
            p_d = -(traj_ned(i,10) - traj_ned(1,10))+ 3 * randn(1);
            p_ned = [p_n;p_e;p_d];
            yaw = traj_ned(i,2) + 0.3 * deg * randn(1);
            ekf_param.Zk = [ v_ned;p_ned;yaw];%
            %��������Ԥ��
            att_predict =Quat2Eul(ekf_param.Xkk_1(1:4));
            predict_yaw = att_predict(3)*deg;%rad��������Ԥ��  
            ekf_param.Zkk_1 = [ ekf_param.Xkk_1(5:10);predict_yaw];
            % �����������
            ekf_param.K = ekf_param.Pkk_1 * ekf_param.H' * inv( ( ekf_param.R + ekf_param.H *ekf_param.Pkk_1 * ekf_param.H' ) );% �����������

            ekf_param.Xk=ekf_param.Xkk_1+ ekf_param.K*(ekf_param.Zk-ekf_param.Zkk_1);% ����״̬
            ekf_param.Pk=(eye(length(ekf_param.Xkk_1))-ekf_param.K*ekf_param.H)*ekf_param.Pkk_1;% ����Э�������
            % ����״̬��Э������
            ekf_param.Xk_1=ekf_param.Xk;
            ekf_param.Pk_1=ekf_param.Pk; 
            %ת��Ϊ��̬��
            att = Quat2Eul(ekf_param.Xk(1:4));
            att1 = Quat2Eul(ekf_param.Xkkk_1(1:4));
            % ��¼����  
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
title('��ת��');
xlabel('time / s');
ylabel('Roll / ��');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ');

figure
% plot( traj_ned(:,1) , traj_ned(:,3)/deg , Att_all (:,1) , Att_all (:,3),Att_all (:,1) , Att_all (:,6),'LineWidth',2);
plot( traj_ned(:,1) , traj_ned(:,3)/deg , Att_all (:,1) , Att_all (:,3),'LineWidth',2);
grid on;
title('������');
xlabel('time / s');
ylabel('Pitch/ ��');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ');

figure
% plot( traj_ned(:,1) , traj_ned(:,2)/deg , Att_all (:,1) , Att_all (:,4),Att_all (:,1) , Att_all (:,7),'LineWidth',2);
plot( traj_ned(:,1) , traj_ned(:,2)/deg , Att_all (:,1) , Att_all (:,4),'LineWidth',2);
grid on;
title('ƫ����');
xlabel('time / s');
ylabel('Yaw / ��');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,2), Xk_all (:,1) , Xk_all (:,6),Xk_all (:,1) , Xk_all (:,22),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,2), Xk_all (:,1) , Xk_all (:,6),vel_pos(:,1),vel_pos(:,2),'LineWidth',2);
grid on;
title('�����ٶ�');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,3) , Xk_all (:,1) , Xk_all (:,7),Xk_all (:,1) , Xk_all (:,23),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,3) , Xk_all (:,1) , Xk_all (:,7),vel_pos(:,1),vel_pos(:,3),'LineWidth',2);
grid on;
title('�����ٶ�');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,4) , Xk_all (:,1) , Xk_all (:,8),Xk_all (:,1) , Xk_all (:,24),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,4) , Xk_all (:,1) , Xk_all (:,8),vel_pos(:,1),vel_pos(:,4),'LineWidth',2);
grid on;
title('�����ٶ�');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,5), Xk_all (:,1) , Xk_all (:,9),Xk_all (:,1) , Xk_all (:,25),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,9),Xk_all (:,1) , Xk_all (:,25),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,5), Xk_all (:,1) , Xk_all (:,9),vel_pos(:,1),vel_pos(:,5),'LineWidth',2);
grid on;
title('����λ��');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
% legend('����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,6) , Xk_all (:,1) , Xk_all (:,10),Xk_all (:,1) , Xk_all (:,26),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,10),Xk_all (:,1) , Xk_all (:,26),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,6) , Xk_all (:,1) , Xk_all (:,10),vel_pos(:,1),vel_pos(:,6),'LineWidth',2);
grid on;
title('����λ��');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
% legend('����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');

figure
% plot( Zk_all(:,1) , Zk_all(:,7) , Xk_all (:,1) , Xk_all (:,11),Xk_all (:,1) , Xk_all (:,27),'LineWidth',2);
% plot( Xk_all (:,1) , Xk_all (:,11),Xk_all (:,1) , Xk_all (:,27),'LineWidth',2);
plot( Zk_all(:,1) , Zk_all(:,7) , Xk_all (:,1) , Xk_all (:,11),vel_pos(:,1),vel_pos(:,7),'LineWidth',2);
grid on;
title('����λ��');
xlabel('time / s');
ylabel('m/s');
% legend('����ֵ','����ֵ','����ֵ');
% legend('����ֵ','����ֵ');
legend('����ֵ','����ֵ','����ֵ');


figure
plot( Att_all (:,1) , Xk_all(:,12)/imu.dt_imu/deg ,'LineWidth',2);
grid on;
title('X��������ƫ');
xlabel('time / s');
ylabel('�� / s');

figure
plot( Att_all (:,1) , Xk_all(:,13) /imu.dt_imu/deg,'LineWidth',2);
grid on;
title('Y��������ƫ');
xlabel('time / s');
ylabel('�� / s');

figure
plot( Att_all (:,1) , Xk_all(:,14)/imu.dt_imu/deg ,'LineWidth',2);
grid on;
title('Z��������ƫ');
xlabel('time / s');
ylabel('�� / s');

figure
plot( Att_all (:,1) , Xk_all(:,15)/imu.dt_imu ,'LineWidth',2);
grid on;
title('X��Ӽ���ƫ');
xlabel('time / s');
ylabel('m / s^2');

figure
plot( Att_all (:,1) , Xk_all(:,16) /imu.dt_imu,'LineWidth',2);
grid on;
title('Y��Ӽ���ƫ');
xlabel('time / s');
ylabel('m / s^2');

figure
plot( Att_all (:,1) , Xk_all(:,17)/imu.dt_imu ,'LineWidth',2);
grid on;
title('Z��Ӽ���ƫ');
xlabel('time / s');
ylabel('m / s^2');





