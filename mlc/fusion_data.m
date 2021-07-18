%%chuzhiwei 2020.08.05
clearvars;
clc;
format long;

um482_data = importdata('GPS2.txt');
% ub482 = xlsread('gps.xlsx');
ub482 = um482_data.data;
realtimeFusion_data = importdata('NAV.txt');
realtimeFusion = realtimeFusion_data.data;
fusion_time = roundn(realtimeFusion(:,1),-2);
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
ub482_time = roundn(ub482(:,1),-2);%取两位小数，精确到10ms
ub482_lat = ub482(:,4);
ub482_lon = ub482(:,5);
ub482_alt = ub482(:,6);
ub482_northvel = ub482(:,7);
ub482_eastvel = ub482(:,8);
ub482_downvel = ub482(:,9);
ub482_heading = ub482(:,10);
% ub482_row = size(ub482_heading, 1);
% ub482_heading = adaptive_low_pass(ub482_heading, ub482_row, 0.1, 4, 0.02, 0.1);
ub482_hdop = ub482(:,11);
dif_age = ub482(:,14);
ub482_posType = ub482(:,15);
ub482_yawType = ub482(:,16);
ub482_svn =  ub482(:,18);
fix1 = find(ub482_posType == 50);
fix2 = find(ub482_hdop < 0.8);
fix_num = [fix1(1),fix2(1)];
fix = max(fix_num);
setNED_time = ub482_time(fix);
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
%%%取出第一个fix定位点起始点为局部地理坐标系的原点（x0,y0,z0）
ub482_x0 = ub482_xt(fix(1));
ub482_y0 = ub482_yt(fix(1));
ub482_z0 = ub482_zt(fix(1));
ub482_t = [ub482_x0, ub482_y0, ub482_z0]'; %%即为平移向量
%%以起始点为原点的地理坐标系到ECEF坐标系的旋转矩阵（这里为常量）
ub482_R = [ -sin( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 ),   -sin( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 );
                  -sin( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 ),    cos( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 );
                              cos( ub482_lat(fix)*pi/180 ) ,                                               0                      ,                 -sin( ub482_lat(fix)*pi/180 )    ];                         
%%计算各点在该地理坐标系下的局部坐标（x_loc, y_loc, z_loc）
for t = 1 : ub482_row
    
    ub482_W_t(:,t) = [ub482_xt(t), ub482_yt(t), ub482_zt(t)]';   
    ub482_W_loc(:,t) = (ub482_R) \ ( ub482_W_t(:,t) - ub482_t ); %%"\"为左乘逆
    ub482_x_loc(t) = ub482_W_loc(1,t);
    ub482_y_loc(t) = ub482_W_loc(2,t);
    ub482_z_loc(t) = ub482_W_loc(3,t);   
    
end

%%%%%以下量测已转换至起点为原点的NED系
ub482_Pos_x_o = ub482_x_loc;%%北向位置的量测值
ub482_Pos_y_o = ub482_y_loc;%%东向位置的量测值
ub482_Pos_z_o = ub482_z_loc;%%垂向位置的量测值

for i = 1 : fix
    ub482_Pos_x_o(i) = 0;
    ub482_Pos_y_o(i) = 0;
    ub482_Pos_z_o(i) = 0;
end

figure(1)
plot( ub482_time, ub482_Pos_x_o);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('NorthPos（m）');
legend('ub482');
title('北向位移','fontsize',13);

 %%%%%%%% 东向位移 %%%%%%%
figure(2)
plot( ub482_time, ub482_Pos_y_o);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('EastPos（m）');
legend('ub482');
title('东向位移','fontsize',13);
 %%%%%%%% 地向位移 %%%%%%%
figure(3)
plot( ub482_time, ub482_Pos_z_o);
set(gca,'ygrid','on');
xlabel('相对时间（s）');
ylabel('DownPos（m）');
legend('ub482');
title('地向位移','fontsize',13);


 %%%%%%%% 水平轨迹 %%%%%%%
figure(4)
plot( ub482_Pos_y_o, ub482_Pos_x_o , fusion_eastpos, fusion_northpos, ub482_Pos_y_o(1),  ub482_Pos_x_o(1),'o', ub482_Pos_y_o(ub482_row),  ub482_Pos_x_o(ub482_row),'o');
set(gca,'ygrid','on');
xlabel('东向位移（m）');
ylabel('北向位移（m）');
legend('ub482','融合','起始点','终止点');
title('水平轨迹','fontsize',13);
axis equal


dfx_gnss = diff(ub482_Pos_x_o);
dfy_gnss = diff(ub482_Pos_y_o);
figure(5)
plot(ub482_time(2:end),dfx_gnss);
title('卫星北向位置差分');
xlabel('时间（s）');
ylabel('北向位移差分（m）');
figure(6)
plot(ub482_time(2:end),dfy_gnss);
title('卫星东向位置差分');
xlabel('时间（s）');
ylabel('东向位移差分（m）');

% dftime_gnss = diff(ub482_time);
% for i = 1 : fix-1
%     dftime_gnss(i) = 0.05;
% end
% 
% figure(7)
% plot(ub482_time(2:end),dftime_gnss)
% title('卫星时间差分');
% 
% figure(8)
% plot(ub482_time,dif_age)
% title('差分龄期');
% 
% figure(9)
% plot(ub482_time, ub482_hdop);
% title('卫星HDOP');
% xlabel('时间（s）');
% ylabel('HDOP');
% 
% figure(10)
% plot(ub482_time, ub482_posType);
% title('卫星PosType');
% xlabel('时间（s）');
% ylabel('PosType');

figure(11)
% scatter(ub482_time, ub482_Pos_x_o);
% hold  on;
% scatter(fusion_time,fusion_northpos);
plot(ub482_time, ub482_Pos_x_o, fusion_time,fusion_northpos);
legend('卫星北向位移','融合北向位移');

figure(12)
plot(ub482_time, ub482_Pos_y_o, fusion_time,fusion_eastpos);
legend('卫星东向位移','融合东向位移');

d_fusion_northpos = fusion_northpos(1:1:end);
dif_fusion_northpos = diff(d_fusion_northpos);
figure(13)
plot(fusion_time(2:end), dif_fusion_northpos);
legend('融合北向位移差分');

d_fusion_eastpos = fusion_eastpos(1:1:end);
dif_fusion_eastpos = diff(d_fusion_eastpos);
figure(14)
plot(dif_fusion_eastpos);
legend('融合东向位移差分');

figure(15)
plot(fusion_time(2:end),diff(fusion_time))
title('融合时间差分');

figure(16)
scatter(ub482_time, ub482_lat);
hold  on;
scatter(fusion_time, fusion_lat);
plot(ub482_time, ub482_lat, fusion_time, fusion_lat);
legend('UM482','Fusion');
title('纬度');
xlabel('时间（s）');
ylabel('纬度°');

figure(17)
scatter(ub482_time, ub482_lon);
hold  on;
scatter(fusion_time, fusion_lon);
plot(ub482_time, ub482_lon, fusion_time, fusion_lon);
legend('UM482','Fusion');
title('经度');
xlabel('时间（s）');
ylabel('经度°');

figure(18)
scatter(ub482_time, ub482_alt);
hold  on;
scatter(fusion_time, fusion_height);
plot(ub482_time, ub482_alt, fusion_time, fusion_height);
legend('UM482','Fusion');
title('高程');
xlabel('时间（s）');
ylabel('高度（m）');

figure(19)
plot(fusion_time, fusion_pitch);
title('pitch');
xlabel('时间（s）');
ylabel('pitch（°）');


figure(20)
plot(fusion_time, fusion_northvel);
title('velx');
xlabel('时间（s）');
ylabel('velx（m/s）');

figure(21)
plot(fusion_time, fusion_yaw);
title('yaw');
xlabel('时间（s）');
ylabel('yaw（°）');

% diff_px = zeros(ub482_row,1);
% for t = 1 : ub482_row
%    pt = find(fusion_time == ub482_time(t));
%    diff_px(t) = ub482_Pos_x_o(t) - fusion_northpos(pt);
% end
% figure(22)
% plot(ub482_time, diff_px);
% legend('卫星-融合北向位移');