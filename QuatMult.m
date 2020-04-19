function quatOut = QuatMult(quatA,quatB)
% Calculate the following quaternion product quatA * quatB using the 
% standard identity 
%     w1 = quatA(1);
%     x1 = quatA(2);
%     y1 = quatA(3);
%     z1 = quatA(4);
% 
%     w2 = quatB(1);
%     x2 = quatB(2);
%     y2 = quatB(3);
%     z2 = quatB(4);
% 
%     ret(1) = w1*w2 - x1*x2 - y1*y2 - z1*z2;
%     ret(2) = w1*x2 + x1*w2 + y1*z2 - z1*y2;
%     ret(3) = w1*y2 - x1*z2 + y1*w2 + z1*x2;
%     ret(4) = w1*z2 + x1*y2 - y1*x2 + z1*w2;


quatOut = [quatA(1)*quatB(1)-quatA(2:4)'*quatB(2:4); quatA(1)*quatB(2:4) + quatB(1)*quatA(2:4) + cross(quatA(2:4),quatB(2:4))];


end