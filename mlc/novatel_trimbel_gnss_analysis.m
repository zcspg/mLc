%%chuzhiwei 2020.09.09
clearvars;
clc;
format long;

um482_data = importdata('GPS2.txt');
%ub482 = xlsread('gps.xlsx');
ub482 = um482_data.data;

ub482_time = roundn(ub482(:,1),-2);
ub482_row = size(ub482_time, 1);
ub482_lat = ub482(:,4);
ub482_lon = ub482(:,5);
ub482_alt = ub482(:,6);
ub482_northvel = ub482(:,7);
ub482_eastvel = ub482(:,8);
ub482_downvel = ub482(:,9);
ub482_heading = ub482(:,10);
ub482_hdop = ub482(:,11);
ub482_age = ub482(:,14);
ub482_posType = ub482(:,15);
ub482_yawType = ub482(:,16);
ub482_solnSvs =  ub482(:,18);
ub482_t = 1 : ub482_row;

trimble_gga = xlsread('Gpgga.csv');
trimble_time = trimble_gga(:,1); 
trimble_row = size(trimble_time, 1);
trimble_t = 1 : trimble_row;
trimble_time_s = zeros(trimble_row,1);%周内秒
weekNo = zeros(trimble_row,1);%周数

for i = 1 : trimble_row
    [weekNo(i), trimble_time_s(i)] = utc2weeks(2020, 9, 10, floor(trimble_time(i)/10000),  floor(mod(trimble_time(i),10000)/100), mod(trimble_time(i),100));
end

trimble_lat = floor(trimble_gga(:,2)/100) + (trimble_gga(:,2) - floor(trimble_gga(:,2)/100)*100)/60; 
trimble_lon = floor(trimble_gga(:,3)/100) + (trimble_gga(:,3) - floor(trimble_gga(:,3)/100)*100)/60; 
trimble_posType = trimble_gga(:,4);
trimble_solnSvs = trimble_gga(:,5);
trimble_hdop = trimble_gga(:,6);
trimble_height = trimble_gga(:,7) + trimble_gga(:,8);
trimble_age = trimble_gga(:,9);

trimble_gst = xlsread('Gpgst.csv');
trimble_lat_std = trimble_gst(:,6);
trimble_lon_std = trimble_gst(:,7);
trimble_rms = trimble_gst(:,2);
trimble_gst_raw = size(trimble_lat_std, 1);
trimble_gst_t = 1 : trimble_gst_raw;

trimble_avr = xlsread('Avr.csv');
trimble_heading = trimble_avr(:,2) + 180;
trimble_yawType = trimble_avr(:,6);
trimble_avr_raw = size(trimble_heading, 1);
trimble_avr_t = 1 : trimble_avr_raw;

for i = 1 : trimble_avr_raw
    if trimble_heading(i) >= 360
        trimble_heading(i) = trimble_heading(i) - 360;
    end
end

trimble_vtg = xlsread('Gpvtg.csv');
trimble_trk_yaw = trimble_vtg(:,1);
trimble_vtg_raw = size(trimble_trk_yaw, 1);
trimble_vtg_t = 1 : trimble_vtg_raw;
trimble_vel =  trimble_vtg(:,4)/3.6;
trimble_northvel = zeros(trimble_vtg_raw, 1);
trimble_eastvel = zeros(trimble_vtg_raw, 1);
for i = 1 : trimble_vtg_raw
    trimble_northvel(i) = trimble_vel(i) * cos(trimble_trk_yaw(i)/180*pi);
    trimble_eastvel(i) = trimble_vel(i) * sin(trimble_trk_yaw(i)/180*pi);
end

novatel_bestpos = xlsread('Bestpos.csv');
novatel_time = novatel_bestpos(:,1); 
novatel_row = size(novatel_time, 1);
novatel_t = 1 : novatel_row;
novatel_lat = novatel_bestpos(:,4); 
novatel_lon = novatel_bestpos(:,5);  
novatel_posType = novatel_bestpos(:,3); 
novatel_svs = novatel_bestpos(:,11); 
novatel_solnSvs = novatel_bestpos(:,12); 
novatel_lat_std = novatel_bestpos(:,8);
novatel_lon_std = novatel_bestpos(:,9);
novatel_height = novatel_bestpos(:,6) + novatel_bestpos(:,7); 
novatel_age = novatel_bestpos(:,10);

novatel_psrdop = xlsread('Psrdop.csv');
novatel_psrdop_time = novatel_psrdop(:,1); 
novatel_psrdop_row = size(novatel_psrdop_time, 1);
novatel_psrdop_t = 1 : novatel_psrdop_row;
novatel_hdop = novatel_psrdop(:,4); 

novatel_dualant = xlsread('dualantennaHeading.csv');
novatel_dualant_time = novatel_dualant(:,1);
novatel_dualant_row = size(novatel_dualant_time, 1);
novatel_dualant_t = 1 : novatel_dualant_row;
novatel_heading =  novatel_dualant(:,5);
novatel_yawType =  novatel_dualant(:,3);

novatel_bestvel = xlsread('Bestvel.csv');
novatel_bestvel_time = novatel_bestvel(:,1);
novatel_bestvel_row = size(novatel_bestvel_time, 1);
novatel_bestvel_t = 1 : novatel_bestvel_row;
novatel_vel = novatel_bestvel(:,6);
novatel_trk_yaw = novatel_bestvel(:,7);
novatel_northvel = zeros(novatel_bestvel_row,1);
novatel_eastvel = zeros(novatel_bestvel_row,1);

novatel_psrvel = xlsread('Psrvel.csv');
novatel_psrvel_time = novatel_psrvel(:,1);
novatel_psrvel_row = size(novatel_psrvel_time, 1);
novatel_psrvel_t = 1 : novatel_psrvel_row;
novatel_psr_vel = novatel_psrvel(:,6);
novatel_psr_trk_yaw = novatel_psrvel(:,7);
novatel_psr_northvel = zeros(novatel_psrvel_row,1);
novatel_psr_eastvel = zeros(novatel_psrvel_row,1);


for i = 1 : novatel_bestvel_row
    novatel_northvel(i) = novatel_vel(i)*cos(novatel_trk_yaw(i)*pi/180);
    novatel_eastvel(i) = novatel_vel(i)*sin(novatel_trk_yaw(i)*pi/180);
end

for i = 1 : novatel_psrvel_row
    novatel_psr_northvel(i) = novatel_psr_vel(i)*cos(novatel_psr_trk_yaw(i)*pi/180);
    novatel_psr_eastvel(i) = novatel_psr_vel(i)*sin(novatel_psr_trk_yaw(i)*pi/180);
end


figure(1)
plot( novatel_time(2:end), diff(novatel_lat)*1e6, novatel_time, novatel_posType/10, novatel_time, novatel_solnSvs, novatel_time(2:end), diff(novatel_time)*10)
xlabel('周内秒（s）');
legend('纬度差分*1e6(dm级)','posType/10','solnSvs','dif-time*10');
title('Novatel');

figure(2)
plot( novatel_time(2:end), diff(novatel_lon)*1e6, novatel_time, novatel_posType/10, novatel_time, novatel_solnSvs, novatel_time(2:end), diff(novatel_time)*10)
xlabel('周内秒（s）');
legend('经度差分*1e6(dm级)','posType/10','solnSvs','dif-time*10');
title('Novatel');


figure(3)
plot( trimble_time_s(2:end), diff(trimble_lat)*1e6, trimble_time_s, trimble_posType, trimble_time_s, trimble_solnSvs, trimble_time_s(2:end), diff(trimble_time_s))
xlabel('周内秒（s）');
legend('纬度差分*1e6(dm级)','posType','solnSvs','dif-time*10');
title('Trimble');

figure(4)
plot( trimble_time_s(2:end), diff(trimble_lon)*1e6, trimble_time_s, trimble_posType, trimble_time_s, trimble_solnSvs ,  trimble_time_s(2:end), diff(trimble_time_s))
xlabel('周内秒（s）');
legend('经度差分*1e6(dm级)','posType','solnSvs','dif-time*10');
title('Trimble');

figure(5)
plot( ub482_t(2:end), diff(ub482_lat)*1e6, ub482_t, ub482_posType/10, ub482_t, ub482_solnSvs, ub482_t(2:end), diff(ub482_time))
xlabel('采样时间（0.05s）');
legend('纬度差分*1e6(dm级)','posType/10','solnSvs','dif-time*10');
title('UM482');

figure(6)
plot( ub482_t(2:end), diff(ub482_lon)*1e6, ub482_t, ub482_posType/10, ub482_t, ub482_solnSvs, ub482_t(2:end), diff(ub482_time))
xlabel('采样时间（0.05s）');
legend('经度差分*1e6(dm级)','posType/10','solnSvs','dif-time*10');
title('UM482');

% figure(7)
% plot( trimble_time_s - trimble_time_s(1), trimble_lat, novatel_time - novatel_time(1) , novatel_lat, ub482_time - ub482_time(1), ub482_lat)
% xlabel('时间（s）');
% legend('Trimble','Novatel','UM482');
% title('纬度');

figure(7)
plot( trimble_time_s , trimble_lat, novatel_time, novatel_lat)
xlabel('周内秒（s）');
legend('Trimble','Novatel');
title('纬度');

% figure(8)
% plot( trimble_time_s - trimble_time_s(1), trimble_lon, novatel_time - novatel_time(1) , novatel_lon, ub482_time - ub482_time(1), ub482_lon)
% xlabel('时间（s）');
% legend('Trimble','Novatel','UM482');
% title('经度');

figure(8)
plot( trimble_time_s, trimble_lon, novatel_time, novatel_lon)
xlabel('周内秒（s）');
legend('Trimble','Novatel');
title('经度');


% figure(9)
% plot( ub482_t, ub482_lat)
% xlabel('采样时间（0.05s）');
% legend('UM482');
% title('纬度');
% 
% figure(10)
% plot( ub482_t, ub482_lon)
% xlabel('采样时间（0.05s）');
% legend('UM482');
% title('经度');



figure(12)
plot( ub482_t, ub482_northvel, trimble_vtg_t, trimble_northvel, novatel_bestvel_time, novatel_northvel, novatel_psrvel_t, novatel_psr_northvel)
xlabel('周内秒（s）');
legend('UM482','Trimble','Novatel-bestvel','Novatel-psrvel');
title('北向速度');

figure(13)
plot( ub482_t, ub482_eastvel, trimble_vtg_t, trimble_eastvel, novatel_bestvel_t, novatel_eastvel, novatel_psrvel_t, novatel_psr_eastvel)
xlabel('采样时间（0.05s）');
legend('UM482','Trimble','Novatel-bestvel','Novatel-psrvel');
title('东向速度');


figure(14)
plot(ub482_t, ub482_alt, trimble_t, trimble_height, novatel_t, novatel_height)
xlabel('采样时间（0.05s）');
legend('UM482','Trimble','Novatel');
title('WGS84高');

figure(15)
plot(trimble_lon, trimble_lat, novatel_lon, novatel_lat)
xlabel('lon');
ylabel('lat');
legend('Trimble','Novatel');
title('轨迹');

figure(16)
plot(ub482_lon, ub482_lat)
xlabel('lon');
ylabel('lat');
legend('UM482');
title('轨迹');


figure(17)
plot(ub482_t, ub482_hdop, ub482_t, ub482_posType/10)
xlabel('采样时间（0.05s）');
% ylabel('lat');
legend('HDOP','POSTYPE/10');
title('UM482');

figure(18)
plot(trimble_t, trimble_hdop, trimble_t, trimble_posType)
xlabel('采样时间（0.05s）');
% ylabel('lat');
legend('HDOP','POSTYPE');
title('Trimble');

figure(19)
plot(novatel_psrdop_t, novatel_hdop, novatel_t, novatel_posType/10)
xlabel('采样时间（0.05s）');
% ylabel('lat');
legend('HDOP','POSTYPE/10');
title('Novatel');

figure(20)
plot( trimble_avr_t, trimble_heading,  trimble_avr_t, trimble_yawType*10)
xlabel('采样时间（0.05s）');
legend('Trimble', 'Trimble-yawType*10');
title('航向');

figure(21)
plot( novatel_dualant_t, novatel_heading, novatel_dualant_t, novatel_yawType)
xlabel('采样时间（0.05s）');
legend('Novatel','Novatel-yawType');
title('航向');

figure(22)
plot( ub482_t, ub482_heading, ub482_t, ub482_yawType)
xlabel('采样时间（0.05s）');
legend('UM482','UM482-yawType');
title('航向');

figure(23)
plot(novatel_psrdop_t, novatel_hdop, novatel_t, novatel_posType/10, novatel_t, novatel_age)
xlabel('采样时间（0.05s）');
% ylabel('lat');
legend('HDOP','POSTYPE/10','diff-age');
title('Novatel');

figure(24)
plot(trimble_t, trimble_hdop, trimble_t, trimble_posType, trimble_t, trimble_age)
xlabel('采样时间（0.05s）');
% ylabel('lat');
legend('HDOP','POSTYPE','diff-age');
title('Trimble');

disp(['Novatel 718D 位置固定率 =' num2str(sum(novatel_posType==50) / novatel_row)]);
disp(['Trimble BD982 位置固定率 =' num2str(sum(trimble_posType==4) / trimble_row)]);
disp(['unicore UM482 位置固定率 =' num2str(sum(ub482_posType==50) / ub482_row)]);

disp(['Novatel 718D 航向固定率 =' num2str(sum(novatel_yawType==50) / novatel_row)]);
disp(['Trimble BD982 航向固定率 =' num2str(sum(trimble_yawType==3) / trimble_row)]);
disp(['Trimble UM482 航向固定率 =' num2str(sum(ub482_yawType==50) / ub482_row)]);

% figure(12)
% plot( diff(novatel_time))
% 
% figure(13)
% plot( trimble_t, trimble_lat - novatel_lat(5:end))
% xlabel('采样时间（0.05s）');
% legend('Trimble - Novatel');
% title('纬度');
% 
% figure(14)
% plot( trimble_t, trimble_lon - novatel_lon(5:end))
% xlabel('采样时间（0.05s）');
% legend('Trimble - Novatel');
% title('经度');