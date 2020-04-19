function [dtIMU,deg2rad,delAngIMU,delVelIMU,delMag] = load_data_enu(traj)
    Data = traj;
    data_len = length(Data);
    dtIMU = 0.005;
    % g0 = 9.7803267714;
    deg2rad = pi/180;
    %计算角增量
    delAngIMU_ENU( 1 , : ) = Data(1,11:13);
    delAngIMU_ENU( 2:data_len , : ) = diff(Data(:,11:13));
    angRate(:,2:4) = delAngIMU_ENU/dtIMU/pi*180;
    angRate(:,1) = Data(:,1);
    %转换到NED(rad)
    delAngIMU= [ angRate(:,3) , angRate(:,2) ,  -1*angRate(:,4) ]   ;%°/s
    %计算速度增量
    delVelIMU_ENU( 1 , : ) = Data(1,14:16);
    delVelIMU_ENU( 2:data_len , : ) = diff(Data(:,14:16));
    accel(:,2:4)  = delVelIMU_ENU/dtIMU;
    accel(:,1) = Data(:,1);
    %转换到NED
    delVelIMU= [ accel(:,3) , accel(:,2) ,  -1*accel(:,4) ]  ;%m/ss
    %计算磁力计增量
    delMag( 1 , : ) = Data(1,17:19);
    delMag( 2:data_len , : ) = diff(Data(:,17:19));
    delMag_rate(:,2:4)  = delMag/dtIMU;
    delMag_rate(:,1) = Data(:,1);
    %转换到NED
    delMag= [ delMag_rate(:,3) , delMag_rate(:,2) ,  -1*delMag_rate(:,4) ]  ;%m/ss

end