function Q =   Eul2Quat(EulAHRS)
% EulAHRS
% Q∂‘”¶Tb2n
% Tb2n =
%  
% [               c_y*c_z,              -c_y*s_z,      s_y]
% [ c_x*s_z + c_z*s_x*s_y, c_x*c_z - s_x*s_y*s_z, -c_y*s_x]
% [ s_x*s_z - c_x*c_z*s_y, c_z*s_x + c_x*s_y*s_z,  c_x*c_y]

u(1:3) =  cos(0.5*EulAHRS);
u(4:6) =  sin(0.5*EulAHRS);

Q(1) = u(1) * u(2) * u(3) + u(4) * u(5) * u(6);
Q(2) = u(4) * u(2) * u(3) - u(1) * u(5) * u(6);
Q(3) = u(1) * u(5) * u(3) + u(4) * u(2) * u(6);
Q(4) = u(1) * u(2) * u(6) - u(4) * u(5) * u(3);

end