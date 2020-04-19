%   定义全局变量
global Re e wie g0 ppm ug deg min sec hur dph 

    Re = 6378160;               %  地球半径
    e = 1/298.257223563;        %  地球椭圆度
    wie = 7.2921151467e-5;      %  自转角速率
    g0 = 9.7803267714;          %  重力加速度
    ppm = 1.0e-6;               %  百万分之一
    mg = 1.0e-3*g0;             %  毫重力加速度
    ug = 1.0e-6*g0;             %  微重力加速度
    deg = pi/180;               %  角度
    min = deg/60;               %  角分
    sec = min/60;               %  角秒
    hur = 3600;                 %  小时
    dph = deg/hur;              %  度/小时
