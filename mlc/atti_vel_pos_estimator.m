%%chuzhiwei 2020.08.05
clearvars;
clc;
format long;
addpath('../data');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%  输入数据  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g_pos_bias = [0; 0;  0];
ant1 = [0.8725;	-0.068;	-1.6455];%%主天线坐标！
ant2 = [0.8725;	1.532;	-1.6455];%从天线在IMU系中坐标

baseline = ant2 - ant1;
AA = atan(baseline(2)/baseline(1)) * 180 /pi;
HH = -atan(baseline(3) / sqrt(baseline(2) * baseline(2) + baseline(1) * baseline(1))) * 180 / pi;

um482_data = importdata('GPS2.txt');
ub482 = um482_data.data;
adis_data = importdata('IMU2.txt');
adis = adis_data.data;
realtimeFusion_data = importdata('NAV.txt');
realtimeFusion = realtimeFusion_data.data;
fusion_time = realtimeFusion(:,1);
fusion_row = size(fusion_time, 1);
fusion_yaw = realtimeFusion(:,2);
fusion_pitch = realtimeFusion(:,3);
fusion_roll = realtimeFusion(:,4);
fusion_northvel = realtimeFusion(:,5);
fusion_eastvel = realtimeFusion(:,6);
fusion_downvel = realtimeFusion(:,7);
fusion_northpos = realtimeFusion(:,8);
fusion_eastpos = realtimeFusion(:,9);
fusion_downpos = realtimeFusion(:,10);
fusion_lat =  realtimeFusion(:,11);
fusion_lon =  realtimeFusion(:,12);
fusion_height =  realtimeFusion(:,13);
fusion_UTMx =  realtimeFusion(:,14);
fusion_UTMy =  realtimeFusion(:,15);
ub482_time = roundn(ub482(:,1),-2);
ub482_lat = ub482(:,4);
ub482_lon = ub482(:,5);
ub482_alt = ub482(:,6);
ub482_northvel = ub482(:,7);
ub482_eastvel = ub482(:,8);
ub482_downvel = ub482(:,9);
ub482_heading = ub482(:,10);
ub482_hdop = ub482(:,11);
ub482_posType = ub482(:,15);
ub482_yawType = ub482(:,16);
ub482_svn =  ub482(:,18);
fix1 = find(ub482_posType == 50);
fix2 = find(ub482_hdop < 0.8);
fix_num = [fix1(1),fix2(1)];
fix = max(fix_num);
setNED_time = ub482_time(fix);
imu_time = roundn(adis(:,1),-2);
imu_fix = find(imu_time == setNED_time);
accx = adis(:,8);
accx = moving_average_filter(accx, 3);
accy = adis(:,9);
accy = moving_average_filter(accy, 3);
accz = adis(:,10);
accz = moving_average_filter(accz, 3);
gyrox = adis(:,11);
% gyrox = moving_average_filter(gyrox, 3);
gyroy = adis(:,12);
% gyroy = moving_average_filter(gyroy, 3);
gyroz = adis(:,13);
% gyroz = moving_average_filter(gyroz, 3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%  UKF  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 0.01;%采样周期(s) (IMU: 100Hz)

row = size(imu_time, 1); %数据行数，即总的采样次数

Xhat(1,:) = [0 0 0];%%目标初始估计姿态角（单位°）
Xhat_1(1,:) = [0 0 0];

Z = zeros(row,3); %%量测航姿初始化
pitch_o = zeros(row,1);%%pitch的量测值
roll_o = zeros(row,1);%%roll的量测值
yaw_o =  zeros(row,1);%%yaw的量测值
P = 100 * diag([1,1,1]);%初始估计误差协方差

Gyro = [gyrox , gyroy , gyroz];
Accel = [accx, accy, accz];

%此处加入自适应低通滤波尝试
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 加速度计量测姿态角 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%计算pitch/roll
for i = 1 : row
    
    a1 = Accel(i, 1);
    a2 = -Accel(i, 2);
    a3 = -Accel(i, 3);
    pitch_o(i) = atan(a1 / sqrt((a2)^2 + (a3)^2));
    if  ( a3>0 )
        roll_o(i) = atan( a2/a3 );%%%%%%%%%%0-+-90度
    elseif  ( (a2>=0) && (a3<0) )
        roll_o(i) = pi + atan(a2/a3);%%%%%90-180度
    elseif  ( (a2<0) && (a3<0) )
        roll_o(i) = -pi + atan(a2/a3);%%%%%%-90--180度
    end
    
    %Z(t, 1) = yaw_o(t)*180/pi - 1.05;%减磁偏角
    Z(i, 2) = pitch_o(i)*180/pi;
    Z(i, 3) = roll_o(i)*180/pi;
    
end


% for t = 1 : row   %%%%%俯仰横滚量测赋值
%
% %    Z(t, 1) = yaw_o(t)*180/pi - 1.05;%减磁偏角
%    Z(t, 2) = pitch_o(t)*180/pi;
%    Z(t, 3) = roll_o(t)*180/pi;
%
% end

Xhat(1,:) = [ub482_heading(1) Z(1, 2) Z(1, 3)];
xik = 0;
Ro = [1e-8,0,0
    0,1e-8,0
    0,0,1e-8];
R = Ro;

Q = (1e-10) * diag([1,1,1]);
Yaw_error = zeros(row,1);
% Yaw_error(1) = 90;
qmole = zeros(row,1);
% qmole(1) = 1;
qdeno = zeros(row,1);
% qdeno(1) = 0.000001;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 时间更新方程（预测） %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z_yaw1 = zeros(row,1);
for t = 2 : row
    
    sp = chol((3+xik) * P);
    X = UT_transform(Xhat(t-1,:), sp);
    W0 = xik / (3+xik);
    W1 = 1 / (2 * (3+xik));
    W = [W0, W1, W1, W1, W1, W1, W1];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %先验估计及其均值和协方差
    for i = 1 : 7
        X(:,i) = sigma_predict(X(:,i),Gyro(t,:),T);
    end
    
    Xhat(t,:) = sigma_mean_value(X,W);%样本点均值
    
    Xhat(t, 1) = norm_yaw(Xhat(t, 1));
    Xhat(t, 2) = norm_pitch(Xhat(t, 2));
    Xhat(t, 3) = norm_roll(Xhat(t, 3));
    
    if ( ~ismember(imu_time(t), ub482_time) )
        Z(t,1) = Xhat(t,1);
        dualHeadingFlag = 0;
    else
        pt = find(ub482_time == imu_time(t));
        if ((ub482_posType(pt)==50) && ub482_yawType(pt)==50)
            dualHeadingFlag = 1;
            Z(t,1) = ub482_heading(pt);
            
        else
            Z(t,1) = Xhat(t,1);
            dualHeadingFlag = 0;
        end
    end
    
    Xhat_1(t,:) = Xhat(t,:);
    if(Xhat_1(t,1) < 0)
        Xhat_1(t,1) = Xhat_1(t,1)+360;
    end
    if(Xhat_1(t,1) >= 360)
        Xhat_1(t,1) = Xhat_1(t,1)-360;
    end
    
    Xhat_yaw_d =  Xhat_1(t,1) - Xhat(t-1,1);
    Xhat_pitch_d = Xhat_1(t,2) - Xhat(t-1,2);
    Xhat_roll_d =  Xhat_1(t,3) - Xhat(t-1,3) ;
    Z_yaw_d =  Z(t,1) - Z(t-1,1) ;
    Z_pitch_d =  Z(t,2) - Z(t-1,2) ;
    Z_roll_d =  Z(t,3) - Z(t-1,3) ;
    
    Yaw_d = abs( Xhat_yaw_d - Z_yaw_d );
    Pitch_d = abs( Xhat_pitch_d - Z_pitch_d);
    Roll_d = abs( Xhat_roll_d - Z_roll_d);
    
    P =  prior_P(W, X, Xhat(t,:), Q);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 测量更新方程（修正） %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Pzz = P_zz(W, X, Xhat(t,:), R);
    Pxz = Pzz - R;
    
    K = Pxz / (Pzz);
    Z_X = Z(t,:)' - Xhat(t,:)';
    if(Z_X > 180)
        Z_X = Z_X - 360;
    elseif(Z_X < -180)
        Z_X  = Z_X +360;
    end
    
    Xhat(t, :) = Xhat(t, :)' + K * (Z_X);
    
    if(Xhat(t,1) < 0)
        Xhat(t,1) = Xhat(t,1)+360;
    end
    if(Xhat(t,1) >= 360)
        Xhat(t,1) = Xhat(t,1)-360;
    end
    
    P = P - K * Pzz * K';
    
    if  Pitch_d > 0.03
        Xhat(t, 2) = Xhat_1(t, 2);%pitch
    end
    if  Roll_d > 0.01
        Xhat(t, 3) = Xhat_1(t, 3);%roll
    end
    
    if ((dualHeadingFlag==1)&&(ub482_heading(pt)~=0))
        Qmole = sin(Xhat(t, 3) / 180 * pi) * tan(HH * pi / 180) + cos(Xhat(t, 3) / 180 * pi) * sin(AA / 180 * pi);
        Qdeno = sin(Xhat(t, 3) / 180 * pi) * sin( Xhat(t, 2) / 180 * pi) * sin(AA / 180 * pi) +  cos( Xhat(t, 2) / 180 * pi) * cos(AA / 180 * pi) - cos(Xhat(t, 3) / 180 * pi) * sin( Xhat(t, 2)/ 180 * pi) * tan(HH * pi / 180);
        yaw_error = atan(Qmole / Qdeno) *  180 / pi;
        if (AA > 85)
            yaw_error = abs(yaw_error);
        elseif (AA < -85)
            yaw_error = -abs(yaw_error);
        end
        
        qmole(t) = Qmole;
        qdeno(t) = Qdeno;
        Yaw_error(t) = yaw_error;
        Xhat(t,1) = ub482_heading(pt) - yaw_error;
        Z_yaw1(t) = ub482_heading(pt) - yaw_error;
        
        if(Xhat(t,1) < 0)
            Xhat(t,1) = Xhat(t,1)+360;
        end
        if(Xhat(t,1) >= 360)
            Xhat(t,1) = Xhat(t,1)-360;
        end
    else
        if(abs(Xhat_1(t, 1)-Xhat(t-1, 1)) < 180)
            Xhat(t, 1) = 0.98 *  Xhat_1(t, 1) + 0.02  * Xhat(t-1, 1);
        end
    end
    if (imu_time(t) < setNED_time)
        Xhat(t,1) = 0;
    end
    
end

Yaw = Xhat(:,1);
Pitch  = Xhat(:,2);
Roll = Xhat(:,3);

%%%%%%% 真航向 %%%%%%%
figure(1)
plot(imu_time, Yaw,  ub482_time, ub482_heading, fusion_time, fusion_yaw);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('航向（°）');
legend('仿真','UM482','嵌入式实时运行');
title('真航向','fontsize',13);
%
% %%%%%%%% 俯仰角 %%%%%%%
figure(2)
plot(imu_time, Pitch, fusion_time, fusion_pitch);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('俯仰（°）');
legend('Fusion','嵌入式实时运行');
title('俯仰角','fontsize',13);
%%%%%%%% 横滚角 %%%%%%%
figure(3)
plot(imu_time, Roll, fusion_time, fusion_roll);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('横滚（°）');
legend('Fusion','嵌入式实时运行');
title('横滚角','fontsize',13);


ub482_vel_flag = 0;
accelx = zeros(row,1);
accely = zeros(row,1);
accelz = zeros(row,1);
for i = 1 : row
    accelx(i) = (accx(i) - sin(Pitch(i)*pi/180))*9.80665;
    accely(i) = (accy(i) + cos(Pitch(i)*pi/180)*sin(Roll(i)*pi/180))*9.80665;
    accelz(i) = (accz(i) + cos(Pitch(i)*pi/180)*cos(Roll(i)*pi/180))*9.80665;
end

%%%%%%%%%%%%%%% KF迭代

X_V(:,1) = [ub482_northvel(1); ub482_eastvel(1); ub482_downvel(1)];%%目标初始估计三轴速度（单位：m/s）
X_V_1(:,1) = [0 0 0];%%先验

Z_V = zeros(3,row);
Z_V(:,1) = [0; 0; 0];%%量测速度

P = 100*diag([1,1,1]);%初始估计误差协方差
A = diag([1,1,1]);%线性状态方程中的状态矩阵
I = diag([1,1,1]);

Ro = [1e-2,0, 0
    0, 1e-2,0
    0, 0, 1e-2];

R = Ro;
Q = (1e-3) * diag([1,1,1]);
%%IMU体坐标到局部地理坐标系（t系，NED)的旋转矩阵
ACC = zeros(3, row);
for i = 2 : row
    
    C_b_t = [cos(Pitch(i)*pi/180) * cos(Yaw(i)*pi/180), sin(Pitch(i)*pi/180) * sin(Roll(i)*pi/180) * cos(Yaw(i)*pi/180) - cos(Roll(i)*pi/180) * sin(Yaw(i)*pi/180),  sin(Pitch(i)*pi/180) * cos(Roll(i)*pi/180) * cos(Yaw(i)*pi/180) + sin(Roll(i)*pi/180) * sin(Yaw(i)*pi/180);
        cos(Pitch(i)*pi/180) * sin(Yaw(i)*pi/180), sin(Pitch(i)*pi/180) * sin(Roll(i)*pi/180) * sin(Yaw(i)*pi/180) + cos(Roll(i)*pi/180) * cos(Yaw(i)*pi/180),  sin(Pitch(i)*pi/180) * cos(Roll(i)*pi/180) * sin(Yaw(i)*pi/180) - sin(Roll(i)*pi/180) * cos(Yaw(i)*pi/180);
        -sin(Pitch(i)*pi/180)           ,                                                   cos(Pitch(i)*pi/180) * sin(Roll(i)*pi/180)              ,                                            cos(Pitch(i)*pi/180) * cos(Roll(i)*pi/180)                       ];
    
    ACC(:,i-1) = C_b_t*[accelx(i-1); accely(i-1); accelz(i-1)]; %%%计算地理坐标系下三轴运动加速度ACC = C_b_t*accel
    
    X_V(:,i) = X_V(:,i-1) + ACC(:,i-1)*T; %%%状态变量先验估计值
    
    %%%%%%%%%%%% 对先验估计变化量做限幅，以适应无RTK时误差漂移（有一定效果，2019.08.29，这个滤波会使数据更平滑、误差减小）
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X_V_1(:,i) = X_V(:,i);%%保存先验估计
    
    P = A*P*A' + Q; %%%先验误差协方差矩阵
    
    H = eye(3);
    
    K = P*H'/((H*P*H'+R)); %%计算kalman增益
    
    if ( ~ismember(imu_time(i), ub482_time) )  %无ub482信号时，先验估计即为量测（相当于使用Accel积分解算）
        Z_V(:,i) = X_V(:,i);
        ub482_vel_flag = 0;
    else
        pt = find(ub482_time == imu_time(i));
        if ((ub482_posType(pt) == 50) && (ub482_hdop(pt) < 0.8))
            ub482_vel_flag = 1;
            Z_V(:,i) = [ub482_northvel(pt); ub482_eastvel(pt); ub482_downvel(pt)];
        else
            %%%%%%零速监测优化后这里再加入速度置0的情况!!!!!!!
            V_xy = sqrt(X_V(1,i-1) * X_V(1,i-1) +  X_V(2,i-1) * X_V(2,i-1));
            X_V(1,i) = V_xy * cos(Yaw(i)*pi/180);
            X_V(2,i) = V_xy * sin(Yaw(i)*pi/180);
            X_V(3,i) = 0;
            %            Z_V(:,i) = X_V(:,i);
            ub482_vel_flag = 0;
        end
    end
    X_V(:,i) = X_V(:,i) + K*(Z_V(:,i) - X_V(:,i));%%计算后验估计值
    
    if (imu_time(i) < setNED_time)
        X_V(1,i) = 0;
        X_V(2,i) = 0;
        X_V(3,i) = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P = (I - K*H) * P;
    
end

ub482_Vel_x = X_V(1,:);
ub482_Vel_y = X_V(2,:);
ub482_Vel_z = X_V(3,:);

% %%%%%%%北向速度
figure(4)
plot(ub482_time, ub482_northvel, imu_time, ub482_Vel_x, fusion_time, fusion_northvel);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('北向速度（m/s）');
legend('ub482','IMU/ub482融合','嵌入式实时运行');
title('北向速度','fontsize',13);

%%%%%%东向速度
figure(5)
plot(ub482_time, ub482_eastvel, imu_time,  ub482_Vel_y, fusion_time,  fusion_eastvel);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('东向速度（m/s）');
legend('ub482','IMU/ub482融合','嵌入式实时运行');
title('东向速度','fontsize',13);
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WGS84（经纬度）坐标到ECEF坐标及局部地理（北东地）坐标的转换 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = 6378137.0;   %% WGS84长半轴
% e = 1/298.257223565; %%WGS84椭球扁率
e = 0.08181919;%第一偏心率

%%%%%%%%%%%%%%%%%%%%WGS84坐标系到ECEF坐标系的转换

%%%%%%ub482数据%%%%%%%%%
%%%初始化向量组
ub482_row = size( ub482_time, 1 );
ub482_N = zeros( ub482_row,1 );
ub482_xt = zeros( ub482_row,1 );
ub482_yt = zeros( ub482_row,1 );
ub482_zt = zeros( ub482_row,1 );
ub482_W_t = zeros( 3,ub482_row );
ub482_W_loc = zeros( 3,ub482_row );
ub482_x_loc = zeros( ub482_row,1 );
ub482_y_loc = zeros( ub482_row,1 );
ub482_z_loc = zeros( ub482_row,1 );
%计算各点在ECEF坐标系（t）下坐标
for t = 1 : ub482_row
    
    ub482_N(t) = a / sqrt( 1-(e^2) * ( (sin(ub482_lat(t)*pi/180))^2 ) ); %卯酉圈半径
    
    ub482_xt(t) = ( ub482_N(t) + ub482_alt(t) ) * cos( ub482_lat(t)*pi/180 ) * cos( ub482_lon(t)*pi/180 );
    ub482_yt(t) = ( ub482_N(t) + ub482_alt(t) ) * cos( ub482_lat(t)*pi/180 ) * sin( ub482_lon(t)*pi/180 );
    ub482_zt(t) = ( ub482_N(t) * (1-e^2) + ub482_alt(t) ) * sin( ub482_lat(t)*pi/180 );
    
end
%%%取出第一个fix定位点起始点为局部地理坐标系的原点（x0,y0,z0）
ub482_x0 = ub482_xt(fix);
ub482_y0 = ub482_yt(fix);
ub482_z0 = ub482_zt(fix);
ub482_t = [ub482_x0, ub482_y0, ub482_z0]';

ub482_R = [ -sin( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 ),   -sin( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 );
    -sin( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 ),    cos( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 );
    cos( ub482_lat(fix)*pi/180 ) ,                                               0                      ,                 -sin( ub482_lat(fix)*pi/180 )    ];

for t = 1 : ub482_row
    
    ub482_W_t(:,t) = [ub482_xt(t), ub482_yt(t), ub482_zt(t)]';
    ub482_W_loc(:,t) = (ub482_R) \ ( ub482_W_t(:,t) - ub482_t );
    ub482_x_loc(t) = ub482_W_loc(1,t);
    ub482_y_loc(t) = ub482_W_loc(2,t);
    ub482_z_loc(t) = ub482_W_loc(3,t);
    
end

%%%%%%%%%%%%%%% KF迭代

ub482_Pos_x_o = ub482_x_loc;%%北向位置的量测值
ub482_Pos_y_o = ub482_y_loc;%%东向位置的量测值
ub482_Pos_z_o = ub482_z_loc;%%垂向位置的量测值

for i = 1 : fix
    ub482_Pos_x_o(i) = 0;
    ub482_Pos_y_o(i) = 0;
    ub482_Pos_z_o(i) = 0;
end

Z_P = zeros(3,row);


C_b_t_1 = [cos(Pitch(imu_fix)*pi/180) * cos(Yaw(imu_fix)*pi/180), sin(Pitch(imu_fix)*pi/180) * sin(Roll(imu_fix)*pi/180) * cos(Yaw(imu_fix)*pi/180) - cos(Roll(imu_fix)*pi/180) * sin(Yaw(imu_fix)*pi/180),  sin(Pitch(imu_fix)*pi/180) * cos(Roll(imu_fix)*pi/180) * cos(Yaw(imu_fix)*pi/180) + sin(Roll(imu_fix)*pi/180) * sin(Yaw(imu_fix)*pi/180);
    cos(Pitch(imu_fix)*pi/180) * sin(Yaw(imu_fix)*pi/180), sin(Pitch(imu_fix)*pi/180) * sin(Roll(imu_fix)*pi/180) * sin(Yaw(imu_fix)*pi/180) + cos(Roll(imu_fix)*pi/180) * cos(Yaw(imu_fix)*pi/180),  sin(Pitch(imu_fix)*pi/180) * cos(Roll(imu_fix)*pi/180) * sin(Yaw(imu_fix)*pi/180) - sin(Roll(imu_fix)*pi/180) * cos(Yaw(imu_fix)*pi/180);
    -sin(Pitch(imu_fix)*pi/180)           ,                                                   cos(Pitch(imu_fix)*pi/180) * sin(Roll(imu_fix)*pi/180)              ,                                            cos(Pitch(imu_fix)*pi/180) * cos(Roll(imu_fix)*pi/180)                       ];
ant1_bias =  C_b_t_1 * ant1;
Z_P(:,imu_fix) = [ub482_Pos_x_o(fix); ub482_Pos_y_o(fix); ub482_Pos_z_o(fix)];
H = diag([1,1,1]);%量测函数h关于x偏导的雅克比矩阵,初始化
P = 100*diag([1,1,1]);%初始估计误差协方差
A = diag([1,1,1]);%线性状态方程中的状态矩阵
I = diag([1,1,1]);
T = 0.01;

X_P(:,1) = [0; 0; 0];%用第一个量测值初始化状态变量
b_X_P(:,1) =  [0; 0; 0];
pos_bias(:,imu_fix) = C_b_t_1 * g_pos_bias;
X_P_1(:,1) = [0; 0; 0];
Ro = [1e-5,  0,     0
    0,    1e-5,   0
    0,     0 ,  1e-5];
R = Ro;
Q = (1e-3) * diag([1,1,1]);
%%IMU体坐标到局部地理坐标系（t系，NED)的旋转矩阵
ACC = zeros(3, row);
for i = 2 : row
    if(imu_time(i) >= setNED_time)
        C_b_t = [cos(Pitch(i)*pi/180) * cos(Yaw(i)*pi/180), sin(Pitch(i)*pi/180) * sin(Roll(i)*pi/180) * cos(Yaw(i)*pi/180) - cos(Roll(i)*pi/180) * sin(Yaw(i)*pi/180),  sin(Pitch(i)*pi/180) * cos(Roll(i)*pi/180) * cos(Yaw(i)*pi/180) + sin(Roll(i)*pi/180) * sin(Yaw(i)*pi/180);
            cos(Pitch(i)*pi/180) * sin(Yaw(i)*pi/180), sin(Pitch(i)*pi/180) * sin(Roll(i)*pi/180) * sin(Yaw(i)*pi/180) + cos(Roll(i)*pi/180) * cos(Yaw(i)*pi/180),  sin(Pitch(i)*pi/180) * cos(Roll(i)*pi/180) * sin(Yaw(i)*pi/180) - sin(Roll(i)*pi/180) * cos(Yaw(i)*pi/180);
            -sin(Pitch(i)*pi/180)           ,                                                   cos(Pitch(i)*pi/180) * sin(Roll(i)*pi/180)              ,                                            cos(Pitch(i)*pi/180) * cos(Roll(i)*pi/180)                       ];
        
        ACC(:,i) = C_b_t*[accelx(i); accely(i); accelz(i)]; %%%计算地理坐标系下三轴运动加速度ACC = C_b_t*accel
        
        X_P(:,i) = X_P(:,i-1) + X_V(:,i-1)*T + 0.5*ACC(:,i)*T*T; %%%状态变量先验估计值
        X_P_1(:,i) = X_P(:,i) ;
        
        P = A*P*A' + Q; %%%先验误差协方差矩阵
        
        K = P*H'/((H*P*H'+R)); %%计算kalman增益
        
        if ( ~ismember(imu_time(i), ub482_time) )  %无ub482信号时，先验估计即为量测（相当于使用Accel积分解算）
            Z_P(:,i) = X_P(:,i);
            dop_flag = 0;
        else
            pt = find(ub482_time == imu_time(i));
            if ((ub482_posType(pt) == 50) && (ub482_hdop(pt) < 0.8))
                dop_flag = 0;
                ant1_error = C_b_t * ant1;
                Z_P(:,i) = [ub482_Pos_x_o(pt) - ant1_error(1) + ant1_bias(1); ub482_Pos_y_o(pt) - ant1_error(2) + ant1_bias(2); ub482_Pos_z_o(pt)  - ant1_error(3) + ant1_bias(3)];
            elseif ((ub482_posType(pt) == 50) && (ub482_hdop(pt) >= 0.8))
                dop_flag = 1;
                ant1_error = C_b_t * ant1;
                Z_P(:,i) = [ub482_Pos_x_o(pt) - ant1_error(1) + ant1_bias(1); ub482_Pos_y_o(pt) - ant1_error(2) + ant1_bias(2); ub482_Pos_z_o(pt)  - ant1_error(3) + ant1_bias(3)];
                X_P(:,i) = (ub482_hdop(pt) - 0.75) * X_P(:,i)  + (1.75 - ub482_hdop(pt)) * Z_P(:,i);
            else
                dop_flag = 0;
                Z_P(:,i) = X_P(:,i);
            end
        end
        if (dop_flag == 0)
            X_P(:,i) = X_P(:,i) + K*(Z_P(:,i) - X_P(:,i));%%计算后验估计值
        end
        
        pos_bias(:,i) = C_b_t * g_pos_bias;
        
        P = (I - K*H) * P;
    else
        X_P(1,i) = 0;
        X_P(2,i) = 0;
        X_P(3,i) = 0;
    end
end

for i = 1 : row
    %     if(imu_time(i)>setNED_time)
    X_P(:,i) = X_P(:,i) +  pos_bias(:,i);
end

ub482_Pos_x = X_P(1,:);
ub482_Pos_y = X_P(2,:);
ub482_Pos_z = X_P(3,:);


%%%%%%%%%%%%%%%%%%% 地理坐标系（NED,北东地）位置图 %%%%%%%%%%%%%%%%%%

% figure(7)
% plot( ub482_time, ub482_Pos_x_o,imu_time, ub482_Pos_x, fusion_time, fusion_northpos);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('North_Pos（m）');
% legend('ub482','IMU/ub482融合','嵌入式实时运行');
% title('北向位移','fontsize',13);
%
%  %%%%%%%% 东向位移 %%%%%%%
% figure(8)
% plot( ub482_time, ub482_Pos_y_o,imu_time, ub482_Pos_y, fusion_time, fusion_eastpos);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('East_Pos（m）');
% legend('ub482','IMU/ub482融合','嵌入式实时运行');
% title('东向位移','fontsize',13);
figure(7)
plot( ub482_time, ub482_Pos_x_o,imu_time, ub482_Pos_x, fusion_time, fusion_northpos);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('North_Pos（m）');
legend('ub482','IMU/ub482融合','嵌入式实时运行');
title('北向位移','fontsize',13);

%%%%%%%% 东向位移 %%%%%%%
figure(8)
plot( ub482_time, ub482_Pos_y_o,imu_time, ub482_Pos_y, fusion_time, fusion_northpos);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('East_Pos（m）');
legend('ub482','IMU/ub482融合','嵌入式实时运行');
title('东向位移','fontsize',13);
%%%%%%%% 地向位移 %%%%%%%
figure(9)
plot( ub482_time, ub482_Pos_z_o,imu_time, ub482_Pos_z);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('Down_Pos（m）');
legend('ub482','IMU/ub482融合');
title('地向位移','fontsize',13);

% figure(7)
% plot(fusion_time, fusion_northpos);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('北向位移（m）');
% legend('导航融合模块');
% title('北向位移','fontsize',13);
% posx_std = std(fusion_northpos);
% disp(['fusion_posx std  =' num2str(posx_std)]);
% posx_d = max(fusion_northpos)-min(fusion_northpos);
% disp(['posx_d  = ±' num2str(posx_d/2)]);
%  %%%%%%%% 东向位移 %%%%%%%
% figure(8)
% plot( fusion_time, fusion_eastpos);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('东向位移（m）');
% legend('导航融合模块');
% title('东向位移','fontsize',13);
% posy_std = std(fusion_eastpos);
% disp(['fusion_posy std  =' num2str(posy_std)]);
% posy_d = max(fusion_eastpos)-min(fusion_eastpos);
% disp(['posy_d  = ±' num2str(posy_d/2)]);
% %  %%%%%%%% 地向位移 %%%%%%%
% figure(9)
% plot(fusion_time, fusion_downpos);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('地向位移（m）');
% legend('导航融合模块');
% title('地向位移','fontsize',13);
% posz_std = std(fusion_downpos);
% disp(['fusion_posz std  =' num2str(posz_std)]);

% posxy_std = sqrt((posx_std^2 + posy_std^2)/2);
% disp(['fusion_posxy std  =' num2str(posxy_std)]);
% posz_d = max(fusion_downpos)-min(fusion_downpos);
% disp(['posz_d  = ±' num2str(posz_d/2)]);
%%%%%%%% 水平轨迹 %%%%%%%
figure(10)
plot( ub482_Pos_y_o, ub482_Pos_x_o, ub482_Pos_y(imu_fix:end), ub482_Pos_x(imu_fix:end), fusion_eastpos(1), fusion_northpos(1),'o' , fusion_eastpos(fusion_row),  fusion_northpos(fusion_row),'o');
set(gca,'ygrid','on');
xlabel('东向位移（m）');
ylabel('北向位移（m）');
legend('ub482','IMU/ub482融合','起始点','终止点');
title('水平轨迹','fontsize',13);
axis equal

% figure(10)
% plot( ub482_Pos_y_o, ub482_Pos_x_o, ub482_Pos_y, ub482_Pos_x, fusion_eastpos, fusion_northpos, fusion_eastpos(1), fusion_northpos(1),'o' , fusion_eastpos(fusion_row),  fusion_northpos(fusion_row),'o');
% set(gca,'ygrid','on');
% xlabel('东向位移（m）');
% ylabel('北向位移（m）');
% legend('ub482','IMU/ub482融合','嵌入式实时运行','起始点','终止点');
% title('水平轨迹','fontsize',13);
% axis equal

%
% figure(10)
% plot(fusion_eastpos, fusion_northpos, fusion_eastpos(1), fusion_northpos(1),'o' , fusion_eastpos(fusion_row),  fusion_northpos(fusion_row),'o');
% set(gca,'ygrid','on');
% xlabel('东向位移（m）');
% ylabel('北向位移（m）');
% legend('导航融合模块','起始点','终止点');
% title('水平运动轨迹','fontsize',13);
%
% figure(11)
% plot(fusion_time, fusion_lat);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('纬度（°）');
% legend('导航融合模块');
% title('纬度','fontsize',13);
%
% figure(12)
% plot(fusion_time, fusion_lon);
% set(gca,'ygrid','on');
% xlabel('相对时间（s）');
% ylabel('经度（°）');
% legend('导航融合模块');
% title('经度','fontsize',13);
%
% figure(13)
% plot(fusion_UTMy*1000 - fusion_UTMy(1)*1000, fusion_UTMx*1000 - fusion_UTMx(1)*1000);
% set(gca,'ygrid','on');
% xlabel('东向位移（m）');

dfx_gnss = diff(ub482_Pos_x_o);
dfy_gnss = diff(ub482_Pos_y_o);
figure(15)
plot(dfx_gnss);
title('卫星北向位置差分');
figure(16)
plot(dfy_gnss);
title('卫星东向位置差分');

fusion_northpos_o = ub482_Pos_x(1:3:end);
fusion_eastpos_o = ub482_Pos_y(1:3:end);
dfx_fusion = diff(fusion_northpos_o);
dfy_fusion = diff(fusion_eastpos_o);

figure(17)
plot(dfx_fusion);
title('融合北向位置差分33Hz');

figure(18)
plot(dfy_fusion);
title('融合东向位置差分33Hz');

dftime_gnss = diff(ub482_time);
for i = 1 : fix-1
    dftime_gnss(i) = 0.05;
end
figure(19)
plot(dftime_gnss)
title('卫星时间差分');

dftime_fusion = diff(fusion_time);
figure(20)
plot(dftime_fusion)
title('融合时间差分');

