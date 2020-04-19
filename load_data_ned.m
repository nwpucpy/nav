function [dtIMU,angRate_deg_s,accel_m_ss,delMag] = load_data_ned(traj)
    global Re e wie g0 deg  
    Data = traj;
    data_len = length(Data);
    dtIMU = 0.005;
    deg2rad = pi/180;
    %计算角增量
    delAngIMU_ENU( 1 , : ) = Data(1,11:13);
    delAngIMU_ENU( 2:data_len , : ) = diff(Data(:,11:13));
    angRate(:,2:4) = delAngIMU_ENU/dtIMU/deg2rad;
    angRate(:,1) = Data(:,1);
    %转换到NED(rad)
    angRate_deg_s= [ angRate(:,2) , angRate(:,3) ,  angRate(:,4) ]   ;%°/s
    %计算速度增量
    delVelIMU_ENU( 1 , : ) = Data(1,14:16);
    delVelIMU_ENU( 2:data_len , : ) = diff(Data(:,14:16));
    accel(:,2:4)  = delVelIMU_ENU/dtIMU;
    accel(:,1) = Data(:,1);
    %转换到NED
    accel_m_ss= [ accel(:,2) , accel(:,3) ,  accel(:,4) ]  ;%m/ss
%     %计算磁力计增量
%     delMag( 1 , : ) = Data(1,17:19);
%     delMag( 2:data_len , : ) = diff(Data(:,17:19));
%     delMag_rate(:,2:4)  = delMag/dtIMU;
%     delMag_rate(:,1) = Data(:,1);
%     %转换到NED
%     delMag= [ delMag_rate(:,3) , delMag_rate(:,2) ,  -1*delMag_rate(:,4) ]  ;%m/ss
    delMag = zeros(data_len,3);
    
end