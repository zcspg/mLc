clearvars;
clc;


nav_data = xlsread('nav.xlsx');
ub482 = xlsread('um482.xlsx');

nav_time = nav_data(:,1);
nav_yaw = nav_data(:,2);
nav_pitch = nav_data(:,3);
nav_roll = nav_data(:,4);
nav_veln = nav_data(:,5);
nav_vele = nav_data(:,6);
nav_veld = nav_data(:,7);
nav_posn = nav_data(:,8);
nav_pose = nav_data(:,9);
nav_posd = nav_data(:,10);
nav_lat = nav_data(:,11);
nav_lon = nav_data(:,12);
nav_alt = nav_data(:,13);



ub482_time = roundn(ub482(:,1),-2);%取两位小数，精确到10ms
ub482_lat = ub482(:,2);
ub482_lon = ub482(:,3);
ub482_alt = ub482(:,4);
ub482_northvel = ub482(:,5);
ub482_eastvel = ub482(:,6);
ub482_downvel = ub482(:,7);
ub482_heading = ub482(:,8);
ub482_hdop = ub482(:,9);
ub482_fixType = ub482(:,10) - 48;
ub482_svn =  ub482(:,11);

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
ub482_W_t = zeros( 3,ub482_row );%%各点在ECEF坐标系下的坐标集合
ub482_W_loc = zeros( 3,ub482_row );%各点在局部地理loc坐标系下的坐标集合
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

pt = 0;

for i = 1 : ub482_row
    if(ub482_fixType(i)==2)
        pt = i;
        break
    end
end

%%%取出起始点为局部地理坐标系的原点（x0,y0,z0）
ub482_x0 = ub482_xt(pt);
ub482_y0 = ub482_yt(pt);
ub482_z0 = ub482_zt(pt);
ub482_t = [ub482_x0, ub482_y0, ub482_z0]'; %%即为平移向量
%%以起始点为原点的地理坐标系到ECEF坐标系的旋转矩阵（这里为常量）
ub482_R = [ -sin( ub482_lat(pt)*pi/180 )*cos( ub482_lon(pt)*pi/180 ),   -sin( ub482_lon(pt)*pi/180 ) ,  -cos( ub482_lat(pt)*pi/180 )*cos( ub482_lon(pt)*pi/180 );
                  -sin( ub482_lat(pt)*pi/180 )*sin( ub482_lon(pt)*pi/180 ),    cos( ub482_lon(pt)*pi/180 ) ,  -cos( ub482_lat(pt)*pi/180 )*sin( ub482_lon(pt)*pi/180 );
                              cos( ub482_lat(pt)*pi/180 ) ,                                               0                      ,                 -sin( ub482_lat(pt)*pi/180 )    ];
                         
%%计算各点在该地理坐标系下的局部坐标（x_loc, y_loc, z_loc）
for t = 1 : ub482_row
    
    ub482_W_t(:,t) = [ub482_xt(t), ub482_yt(t), ub482_zt(t)]';   
    ub482_W_loc(:,t) = (ub482_R) \ ( ub482_W_t(:,t) - ub482_t ); %%"\"为左乘逆
    ub482_x_loc(t) = ub482_W_loc(1,t);
    ub482_y_loc(t) = ub482_W_loc(2,t);
    ub482_z_loc(t) = ub482_W_loc(3,t);   
    
end

%%%%%%%%%%%%%%% KF迭代

%%%%%以下量测已转换至起点为原点的NED系
ub482_Pos_x_o = ub482_x_loc;%%北向位置的量测值
ub482_Pos_y_o = ub482_y_loc;%%东向位置的量测值
ub482_Pos_z_o = ub482_z_loc;%%垂向位置的量测值

figure(1)
% subplot(3,1,1)
% plot(nav_time, nav_yaw, vio_time, vio_yaw);
% xlabel('时间(t/s)');
% ylabel('航向 (°)');
% legend('嵌入式实时导航','VIO输出');
% title('yaw');
% grid;
subplot(3,1,1)
plot(nav_time, nav_yaw);
xlabel('时间(t/s)');
ylabel('航向 (°)');
legend('嵌入式实时导航');
title('yaw');
grid;


subplot(3,1,2)
plot(nav_time, nav_pitch);
xlabel('时间(t/s)');
ylabel('俯仰 (°)');
legend('嵌入式实时导航');
title('pitch');
grid;

subplot(3,1,3)
plot(nav_time, nav_roll);
xlabel('时间(t/s)');
ylabel('横滚 (°)');
legend('嵌入式实时导航');
title('roll');
grid;


% figure(2)
% subplot(3,1,1)
% plot(nav_time, nav_veln, vio_time, vio_velx);
% xlabel('时间(t/s)');
% ylabel('北向速度(m/s)');
% legend('嵌入式实时导航','VIO平均速度');
% title('vel-n');
% grid;
% subplot(3,1,1)
% plot(nav_time, nav_veln, vio_time, vio_velx);
% xlabel('时间(t/s)');
% ylabel('北向速度(m/s)');
% legend('嵌入式实时导航','VIO平均速度');
% title('vel-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_vele, vio_time, vio_vely);
% xlabel('时间(t/s)');
% ylabel('东向速度(m/s)');
% legend('嵌入式实时导航','VIO平均速度');
% title('vel-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_veld, vio_time, vio_velz);
% xlabel('时间(t/s)');
% ylabel('地向速度(m/s)');
% legend('嵌入式实时导航','VIO平均速度');
% title('vel-d');
% grid;
% 
% 
% figure(3)
% subplot(3,1,1)
% plot(nav_time, nav_posn, vio_time, vio_posx);
% xlabel('时间(t/s)');
% ylabel('北向位移(m)');
% legend('嵌入式实时导航','VIO输出');
% title('pos-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_pose, vio_time, vio_posy);
% xlabel('时间(t/s)');
% ylabel('东向位移(m)');
% legend('嵌入式实时导航','VIO输出');
% title('pos-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_posd, vio_time, vio_posz);
% xlabel('时间(t/s)');
% ylabel('地向位移(m)');
% legend('嵌入式实时导航','VIO输出');
% title('pos-d');
% grid;

figure(2)
subplot(3,1,1)
plot(nav_time, nav_veln);
xlabel('时间(t/s)');
ylabel('北向速度(m/s)');
legend('嵌入式实时导航');
title('vel-n');
grid;
subplot(3,1,1)
plot(nav_time, nav_veln);
xlabel('时间(t/s)');
ylabel('北向速度(m/s)');
legend('嵌入式实时导航');
title('vel-n');
grid;


subplot(3,1,2)
plot(nav_time, nav_vele);
xlabel('时间(t/s)');
ylabel('东向速度(m/s)');
legend('嵌入式实时导航');
title('vel-e');
grid;


subplot(3,1,3)
plot(nav_time, nav_veld);
xlabel('时间(t/s)');
ylabel('地向速度(m/s)');
legend('嵌入式实时导航');
title('vel-d');
grid;


% figure(3)
% subplot(3,1,1)
% plot(nav_time, nav_posn, ub482_time, ub482_Pos_x_o);
% xlabel('时间(t/s)');
% ylabel('北向位移(m)');
% legend('嵌入式实时导航','UM482位置');
% title('pos-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_pose, ub482_time, ub482_Pos_y_o);
% xlabel('时间(t/s)');
% ylabel('东向位移(m)');
% legend('嵌入式实时导航','UM482位置');
% title('pos-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_posd, ub482_time, ub482_Pos_z_o);
% xlabel('时间(t/s)');
% ylabel('地向位移(m)');
% legend('嵌入式实时导航','UM482位置');
% title('pos-d');
% grid;
figure(3)
subplot(3,1,1)
plot(nav_time, nav_posn);
xlabel('时间(t/s)');
ylabel('北向位移(m)');
legend('嵌入式实时导航');
title('pos-n');
grid;


subplot(3,1,2)
plot(nav_time, nav_pose);
xlabel('时间(t/s)');
ylabel('东向位移(m)');
legend('嵌入式实时导航');
title('pos-e');
grid;


subplot(3,1,3)
plot(nav_time, nav_posd);
xlabel('时间(t/s)');
ylabel('地向位移(m)');
legend('嵌入式实时导航');
title('pos-d');
grid;
% figure(4)
% subplot(3,1,1)
% plot(nav_time, nav_lat, ub482_time, ub482_lat);
% xlabel('时间(t/s)');
% ylabel('纬度(°)');
% legend('嵌入式实时导航','UM482');
% title('latitude');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_lon, ub482_time, ub482_lon);
% xlabel('时间(t/s)');
% ylabel('经度(°)');
% legend('嵌入式实时导航','UM482');
% title('longitude');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_alt, ub482_time, ub482_alt);
% xlabel('时间(t/s)');
% ylabel('WGS84高(m)');
% legend('嵌入式实时导航','UM482');
% title('height');
% grid;
figure(4)
subplot(3,1,1)
plot(nav_time, nav_lat);
xlabel('时间(t/s)');
ylabel('纬度(°)');
legend('嵌入式实时导航');
title('latitude');
grid;


subplot(3,1,2)
plot(nav_time, nav_lon);
xlabel('时间(t/s)');
ylabel('经度(°)');
legend('嵌入式实时导航');
title('longitude');
grid;


subplot(3,1,3)
plot(nav_time, nav_alt);
xlabel('时间(t/s)');
ylabel('WGS84高(m)');
legend('嵌入式实时导航');
title('height');
grid;


% figure(5)
% plot(nav_pose, nav_posn,vio_posy, vio_posx,'LineWidth', 2);
% xlabel('右向位移(m)');
% ylabel('前向位移(m)');
% title('水平运动轨迹');
% legend('嵌入式实时导航','VIO输出');
% grid;
figure(5)
plot(nav_pose, nav_posn, ub482_Pos_y_o, ub482_Pos_x_o, 'LineWidth', 2);
xlabel('右向位移(m)');
ylabel('前向位移(m)');
title('水平运动轨迹');
legend('嵌入式实时导航','UM482');
grid;
% figure(5)
% plot(nav_pose, nav_posn,'LineWidth', 2);
% xlabel('右向位移(m)');
% ylabel('前向位移(m)');
% title('水平运动轨迹');
% legend('嵌入式实时导航');
% grid;