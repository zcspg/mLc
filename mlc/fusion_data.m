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
ub482_time = roundn(ub482(:,1),-2);%ȡ��λС������ȷ��10ms
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
a = 6378137.0;   %% WGS84������
% e = 1/298.257223565; %%WGS84�������
e = 0.08181919;%��һƫ����

%%%%%%%%%%%%%%%%%%%%WGS84����ϵ��ECEF����ϵ��ת��

%%%%%%ub482����%%%%%%%%%
%%%��ʼ��������
ub482_row = size( ub482_time, 1 );
ub482_N = zeros( ub482_row,1 );
ub482_xt = zeros( ub482_row,1 );
ub482_yt = zeros( ub482_row,1 );
ub482_zt = zeros( ub482_row,1 );
ub482_W_t = zeros( 3,ub482_row );%%������ECEF����ϵ�µ����꼯��
ub482_W_loc = zeros( 3,ub482_row );%�����ھֲ�����loc����ϵ�µ����꼯��
ub482_x_loc = zeros( ub482_row,1 );
ub482_y_loc = zeros( ub482_row,1 );
ub482_z_loc = zeros( ub482_row,1 );
%���������ECEF����ϵ��t��������
for t = 1 : ub482_row
    
    ub482_N(t) = a / sqrt( 1-(e^2) * ( (sin(ub482_lat(t)*pi/180))^2 ) ); %î��Ȧ�뾶
    
    ub482_xt(t) = ( ub482_N(t) + ub482_alt(t) ) * cos( ub482_lat(t)*pi/180 ) * cos( ub482_lon(t)*pi/180 );
    ub482_yt(t) = ( ub482_N(t) + ub482_alt(t) ) * cos( ub482_lat(t)*pi/180 ) * sin( ub482_lon(t)*pi/180 );
    ub482_zt(t) = ( ub482_N(t) * (1-e^2) + ub482_alt(t) ) * sin( ub482_lat(t)*pi/180 );
    
end
%%%ȡ����һ��fix��λ����ʼ��Ϊ�ֲ���������ϵ��ԭ�㣨x0,y0,z0��
ub482_x0 = ub482_xt(fix(1));
ub482_y0 = ub482_yt(fix(1));
ub482_z0 = ub482_zt(fix(1));
ub482_t = [ub482_x0, ub482_y0, ub482_z0]'; %%��Ϊƽ������
%%����ʼ��Ϊԭ��ĵ�������ϵ��ECEF����ϵ����ת��������Ϊ������
ub482_R = [ -sin( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 ),   -sin( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*cos( ub482_lon(fix)*pi/180 );
                  -sin( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 ),    cos( ub482_lon(fix)*pi/180 ) ,  -cos( ub482_lat(fix)*pi/180 )*sin( ub482_lon(fix)*pi/180 );
                              cos( ub482_lat(fix)*pi/180 ) ,                                               0                      ,                 -sin( ub482_lat(fix)*pi/180 )    ];                         
%%��������ڸõ�������ϵ�µľֲ����꣨x_loc, y_loc, z_loc��
for t = 1 : ub482_row
    
    ub482_W_t(:,t) = [ub482_xt(t), ub482_yt(t), ub482_zt(t)]';   
    ub482_W_loc(:,t) = (ub482_R) \ ( ub482_W_t(:,t) - ub482_t ); %%"\"Ϊ�����
    ub482_x_loc(t) = ub482_W_loc(1,t);
    ub482_y_loc(t) = ub482_W_loc(2,t);
    ub482_z_loc(t) = ub482_W_loc(3,t);   
    
end

%%%%%����������ת�������Ϊԭ���NEDϵ
ub482_Pos_x_o = ub482_x_loc;%%����λ�õ�����ֵ
ub482_Pos_y_o = ub482_y_loc;%%����λ�õ�����ֵ
ub482_Pos_z_o = ub482_z_loc;%%����λ�õ�����ֵ

for i = 1 : fix
    ub482_Pos_x_o(i) = 0;
    ub482_Pos_y_o(i) = 0;
    ub482_Pos_z_o(i) = 0;
end

figure(1)
plot( ub482_time, ub482_Pos_x_o);
set(gca,'ygrid','on');
xlabel('���ʱ�䣨s��');
ylabel('NorthPos��m��');
legend('ub482');
title('����λ��','fontsize',13);

 %%%%%%%% ����λ�� %%%%%%%
figure(2)
plot( ub482_time, ub482_Pos_y_o);
set(gca,'ygrid','on');
xlabel('���ʱ�䣨s��');
ylabel('EastPos��m��');
legend('ub482');
title('����λ��','fontsize',13);
 %%%%%%%% ����λ�� %%%%%%%
figure(3)
plot( ub482_time, ub482_Pos_z_o);
set(gca,'ygrid','on');
xlabel('���ʱ�䣨s��');
ylabel('DownPos��m��');
legend('ub482');
title('����λ��','fontsize',13);


 %%%%%%%% ˮƽ�켣 %%%%%%%
figure(4)
plot( ub482_Pos_y_o, ub482_Pos_x_o , fusion_eastpos, fusion_northpos, ub482_Pos_y_o(1),  ub482_Pos_x_o(1),'o', ub482_Pos_y_o(ub482_row),  ub482_Pos_x_o(ub482_row),'o');
set(gca,'ygrid','on');
xlabel('����λ�ƣ�m��');
ylabel('����λ�ƣ�m��');
legend('ub482','�ں�','��ʼ��','��ֹ��');
title('ˮƽ�켣','fontsize',13);
axis equal


dfx_gnss = diff(ub482_Pos_x_o);
dfy_gnss = diff(ub482_Pos_y_o);
figure(5)
plot(ub482_time(2:end),dfx_gnss);
title('���Ǳ���λ�ò��');
xlabel('ʱ�䣨s��');
ylabel('����λ�Ʋ�֣�m��');
figure(6)
plot(ub482_time(2:end),dfy_gnss);
title('���Ƕ���λ�ò��');
xlabel('ʱ�䣨s��');
ylabel('����λ�Ʋ�֣�m��');

% dftime_gnss = diff(ub482_time);
% for i = 1 : fix-1
%     dftime_gnss(i) = 0.05;
% end
% 
% figure(7)
% plot(ub482_time(2:end),dftime_gnss)
% title('����ʱ����');
% 
% figure(8)
% plot(ub482_time,dif_age)
% title('�������');
% 
% figure(9)
% plot(ub482_time, ub482_hdop);
% title('����HDOP');
% xlabel('ʱ�䣨s��');
% ylabel('HDOP');
% 
% figure(10)
% plot(ub482_time, ub482_posType);
% title('����PosType');
% xlabel('ʱ�䣨s��');
% ylabel('PosType');

figure(11)
% scatter(ub482_time, ub482_Pos_x_o);
% hold  on;
% scatter(fusion_time,fusion_northpos);
plot(ub482_time, ub482_Pos_x_o, fusion_time,fusion_northpos);
legend('���Ǳ���λ��','�ںϱ���λ��');

figure(12)
plot(ub482_time, ub482_Pos_y_o, fusion_time,fusion_eastpos);
legend('���Ƕ���λ��','�ں϶���λ��');

d_fusion_northpos = fusion_northpos(1:1:end);
dif_fusion_northpos = diff(d_fusion_northpos);
figure(13)
plot(fusion_time(2:end), dif_fusion_northpos);
legend('�ںϱ���λ�Ʋ��');

d_fusion_eastpos = fusion_eastpos(1:1:end);
dif_fusion_eastpos = diff(d_fusion_eastpos);
figure(14)
plot(dif_fusion_eastpos);
legend('�ں϶���λ�Ʋ��');

figure(15)
plot(fusion_time(2:end),diff(fusion_time))
title('�ں�ʱ����');

figure(16)
scatter(ub482_time, ub482_lat);
hold  on;
scatter(fusion_time, fusion_lat);
plot(ub482_time, ub482_lat, fusion_time, fusion_lat);
legend('UM482','Fusion');
title('γ��');
xlabel('ʱ�䣨s��');
ylabel('γ�ȡ�');

figure(17)
scatter(ub482_time, ub482_lon);
hold  on;
scatter(fusion_time, fusion_lon);
plot(ub482_time, ub482_lon, fusion_time, fusion_lon);
legend('UM482','Fusion');
title('����');
xlabel('ʱ�䣨s��');
ylabel('���ȡ�');

figure(18)
scatter(ub482_time, ub482_alt);
hold  on;
scatter(fusion_time, fusion_height);
plot(ub482_time, ub482_alt, fusion_time, fusion_height);
legend('UM482','Fusion');
title('�߳�');
xlabel('ʱ�䣨s��');
ylabel('�߶ȣ�m��');

figure(19)
plot(fusion_time, fusion_pitch);
title('pitch');
xlabel('ʱ�䣨s��');
ylabel('pitch���㣩');


figure(20)
plot(fusion_time, fusion_northvel);
title('velx');
xlabel('ʱ�䣨s��');
ylabel('velx��m/s��');

figure(21)
plot(fusion_time, fusion_yaw);
title('yaw');
xlabel('ʱ�䣨s��');
ylabel('yaw���㣩');

% diff_px = zeros(ub482_row,1);
% for t = 1 : ub482_row
%    pt = find(fusion_time == ub482_time(t));
%    diff_px(t) = ub482_Pos_x_o(t) - fusion_northpos(pt);
% end
% figure(22)
% plot(ub482_time, diff_px);
% legend('����-�ںϱ���λ��');