%四元数转换为姿态角，单位：度
function Att = Quat2Eul( Q0 )

% Q对应Tb2n
% 旋转顺序XYZ
% x=roll
% y=pitch
% z=yaw

% Tb2n =
%  
% [               c_y*c_z,              -c_y*s_z,      s_y]
% [ c_x*s_z + c_z*s_x*s_y, c_x*c_z - s_x*s_y*s_z, -c_y*s_x]
% [ s_x*s_z - c_x*c_z*s_y, c_z*s_x + c_x*s_y*s_z,  c_x*c_y]


% Tbn(1,1) = u(1)^2+u(2)^2-u(3)^2-u(4)^2;
% Tbn(2,1) = 2*(u(2)*u(3)+u(1)*u(4));
% Tbn(3,1) = 2*(u(2)*u(4)-u(1)*u(3));
% 
% Tbn(1,2) = 2*(u(2)*u(3)-u(1)*u(4));
% Tbn(2,2) = u(1)^2-u(2)^2+u(3)^2-u(4)^2;
% Tbn(3,2) = 2*(u(3)*u(4)+u(1)*u(2));
% 
% Tbn(1,3) = 2*(u(2)*u(4)+u(1)*u(3));
% Tbn(2,3) = 2*(u(3)*u(4)-u(1)*u(2));
% Tbn(3,3) = u(1)^2-u(2)^2-u(3)^2+u(4)^2;

    Roll = atan2(  2 * ( Q0(3) * Q0(4) + Q0(1) * Q0(2) )  ,  ( Q0(1)^2  - Q0(2)^2 - Q0(3)^2 + Q0(4)^2) );
    Pitch = -asin( 2 * (Q0(2) * Q0(4) - Q0(1) *  Q0(3) ));
    Yaw = atan2(  2 * ( Q0(2) * Q0(3) + Q0(1) * Q0(4)  ) ,  ( Q0(1)^2  + Q0(2)^2 - Q0(3)^2 - Q0(4)^2) );
    Yaw = mod(Yaw,2*pi);
    
%     Roll = - atan2(  2 * ( Q0(3) * Q0(4) - Q0(1) * Q0(2) )  ,  ( Q0(1)^2  - Q0(2)^2 - Q0(3)^2 + Q0(4)^2) );
%     Pitch = asin( 2 * (Q0(2) * Q0(4)+ Q0(1) *  Q0(3)) );
%     Yaw = atan2(  2 * ( Q0(2) * Q0(3) - Q0(1) * Q0(4)  ) ,  ( Q0(1)^2  + Q0(2)^2 - Q0(3)^2 - Q0(4)^2) );

    Att = [ Roll , Pitch , Yaw ]*180/pi;

end