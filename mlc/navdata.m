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



ub482_time = roundn(ub482(:,1),-2);%ȡ��λС������ȷ��10ms
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WGS84����γ�ȣ����굽ECEF���꼰�ֲ����������أ������ת�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

pt = 0;

for i = 1 : ub482_row
    if(ub482_fixType(i)==2)
        pt = i;
        break
    end
end

%%%ȡ����ʼ��Ϊ�ֲ���������ϵ��ԭ�㣨x0,y0,z0��
ub482_x0 = ub482_xt(pt);
ub482_y0 = ub482_yt(pt);
ub482_z0 = ub482_zt(pt);
ub482_t = [ub482_x0, ub482_y0, ub482_z0]'; %%��Ϊƽ������
%%����ʼ��Ϊԭ��ĵ�������ϵ��ECEF����ϵ����ת��������Ϊ������
ub482_R = [ -sin( ub482_lat(pt)*pi/180 )*cos( ub482_lon(pt)*pi/180 ),   -sin( ub482_lon(pt)*pi/180 ) ,  -cos( ub482_lat(pt)*pi/180 )*cos( ub482_lon(pt)*pi/180 );
                  -sin( ub482_lat(pt)*pi/180 )*sin( ub482_lon(pt)*pi/180 ),    cos( ub482_lon(pt)*pi/180 ) ,  -cos( ub482_lat(pt)*pi/180 )*sin( ub482_lon(pt)*pi/180 );
                              cos( ub482_lat(pt)*pi/180 ) ,                                               0                      ,                 -sin( ub482_lat(pt)*pi/180 )    ];
                         
%%��������ڸõ�������ϵ�µľֲ����꣨x_loc, y_loc, z_loc��
for t = 1 : ub482_row
    
    ub482_W_t(:,t) = [ub482_xt(t), ub482_yt(t), ub482_zt(t)]';   
    ub482_W_loc(:,t) = (ub482_R) \ ( ub482_W_t(:,t) - ub482_t ); %%"\"Ϊ�����
    ub482_x_loc(t) = ub482_W_loc(1,t);
    ub482_y_loc(t) = ub482_W_loc(2,t);
    ub482_z_loc(t) = ub482_W_loc(3,t);   
    
end

%%%%%%%%%%%%%%% KF����

%%%%%����������ת�������Ϊԭ���NEDϵ
ub482_Pos_x_o = ub482_x_loc;%%����λ�õ�����ֵ
ub482_Pos_y_o = ub482_y_loc;%%����λ�õ�����ֵ
ub482_Pos_z_o = ub482_z_loc;%%����λ�õ�����ֵ

figure(1)
% subplot(3,1,1)
% plot(nav_time, nav_yaw, vio_time, vio_yaw);
% xlabel('ʱ��(t/s)');
% ylabel('���� (��)');
% legend('Ƕ��ʽʵʱ����','VIO���');
% title('yaw');
% grid;
subplot(3,1,1)
plot(nav_time, nav_yaw);
xlabel('ʱ��(t/s)');
ylabel('���� (��)');
legend('Ƕ��ʽʵʱ����');
title('yaw');
grid;


subplot(3,1,2)
plot(nav_time, nav_pitch);
xlabel('ʱ��(t/s)');
ylabel('���� (��)');
legend('Ƕ��ʽʵʱ����');
title('pitch');
grid;

subplot(3,1,3)
plot(nav_time, nav_roll);
xlabel('ʱ��(t/s)');
ylabel('��� (��)');
legend('Ƕ��ʽʵʱ����');
title('roll');
grid;


% figure(2)
% subplot(3,1,1)
% plot(nav_time, nav_veln, vio_time, vio_velx);
% xlabel('ʱ��(t/s)');
% ylabel('�����ٶ�(m/s)');
% legend('Ƕ��ʽʵʱ����','VIOƽ���ٶ�');
% title('vel-n');
% grid;
% subplot(3,1,1)
% plot(nav_time, nav_veln, vio_time, vio_velx);
% xlabel('ʱ��(t/s)');
% ylabel('�����ٶ�(m/s)');
% legend('Ƕ��ʽʵʱ����','VIOƽ���ٶ�');
% title('vel-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_vele, vio_time, vio_vely);
% xlabel('ʱ��(t/s)');
% ylabel('�����ٶ�(m/s)');
% legend('Ƕ��ʽʵʱ����','VIOƽ���ٶ�');
% title('vel-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_veld, vio_time, vio_velz);
% xlabel('ʱ��(t/s)');
% ylabel('�����ٶ�(m/s)');
% legend('Ƕ��ʽʵʱ����','VIOƽ���ٶ�');
% title('vel-d');
% grid;
% 
% 
% figure(3)
% subplot(3,1,1)
% plot(nav_time, nav_posn, vio_time, vio_posx);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','VIO���');
% title('pos-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_pose, vio_time, vio_posy);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','VIO���');
% title('pos-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_posd, vio_time, vio_posz);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','VIO���');
% title('pos-d');
% grid;

figure(2)
subplot(3,1,1)
plot(nav_time, nav_veln);
xlabel('ʱ��(t/s)');
ylabel('�����ٶ�(m/s)');
legend('Ƕ��ʽʵʱ����');
title('vel-n');
grid;
subplot(3,1,1)
plot(nav_time, nav_veln);
xlabel('ʱ��(t/s)');
ylabel('�����ٶ�(m/s)');
legend('Ƕ��ʽʵʱ����');
title('vel-n');
grid;


subplot(3,1,2)
plot(nav_time, nav_vele);
xlabel('ʱ��(t/s)');
ylabel('�����ٶ�(m/s)');
legend('Ƕ��ʽʵʱ����');
title('vel-e');
grid;


subplot(3,1,3)
plot(nav_time, nav_veld);
xlabel('ʱ��(t/s)');
ylabel('�����ٶ�(m/s)');
legend('Ƕ��ʽʵʱ����');
title('vel-d');
grid;


% figure(3)
% subplot(3,1,1)
% plot(nav_time, nav_posn, ub482_time, ub482_Pos_x_o);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','UM482λ��');
% title('pos-n');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_pose, ub482_time, ub482_Pos_y_o);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','UM482λ��');
% title('pos-e');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_posd, ub482_time, ub482_Pos_z_o);
% xlabel('ʱ��(t/s)');
% ylabel('����λ��(m)');
% legend('Ƕ��ʽʵʱ����','UM482λ��');
% title('pos-d');
% grid;
figure(3)
subplot(3,1,1)
plot(nav_time, nav_posn);
xlabel('ʱ��(t/s)');
ylabel('����λ��(m)');
legend('Ƕ��ʽʵʱ����');
title('pos-n');
grid;


subplot(3,1,2)
plot(nav_time, nav_pose);
xlabel('ʱ��(t/s)');
ylabel('����λ��(m)');
legend('Ƕ��ʽʵʱ����');
title('pos-e');
grid;


subplot(3,1,3)
plot(nav_time, nav_posd);
xlabel('ʱ��(t/s)');
ylabel('����λ��(m)');
legend('Ƕ��ʽʵʱ����');
title('pos-d');
grid;
% figure(4)
% subplot(3,1,1)
% plot(nav_time, nav_lat, ub482_time, ub482_lat);
% xlabel('ʱ��(t/s)');
% ylabel('γ��(��)');
% legend('Ƕ��ʽʵʱ����','UM482');
% title('latitude');
% grid;
% 
% 
% subplot(3,1,2)
% plot(nav_time, nav_lon, ub482_time, ub482_lon);
% xlabel('ʱ��(t/s)');
% ylabel('����(��)');
% legend('Ƕ��ʽʵʱ����','UM482');
% title('longitude');
% grid;
% 
% 
% subplot(3,1,3)
% plot(nav_time, nav_alt, ub482_time, ub482_alt);
% xlabel('ʱ��(t/s)');
% ylabel('WGS84��(m)');
% legend('Ƕ��ʽʵʱ����','UM482');
% title('height');
% grid;
figure(4)
subplot(3,1,1)
plot(nav_time, nav_lat);
xlabel('ʱ��(t/s)');
ylabel('γ��(��)');
legend('Ƕ��ʽʵʱ����');
title('latitude');
grid;


subplot(3,1,2)
plot(nav_time, nav_lon);
xlabel('ʱ��(t/s)');
ylabel('����(��)');
legend('Ƕ��ʽʵʱ����');
title('longitude');
grid;


subplot(3,1,3)
plot(nav_time, nav_alt);
xlabel('ʱ��(t/s)');
ylabel('WGS84��(m)');
legend('Ƕ��ʽʵʱ����');
title('height');
grid;


% figure(5)
% plot(nav_pose, nav_posn,vio_posy, vio_posx,'LineWidth', 2);
% xlabel('����λ��(m)');
% ylabel('ǰ��λ��(m)');
% title('ˮƽ�˶��켣');
% legend('Ƕ��ʽʵʱ����','VIO���');
% grid;
figure(5)
plot(nav_pose, nav_posn, ub482_Pos_y_o, ub482_Pos_x_o, 'LineWidth', 2);
xlabel('����λ��(m)');
ylabel('ǰ��λ��(m)');
title('ˮƽ�˶��켣');
legend('Ƕ��ʽʵʱ����','UM482');
grid;
% figure(5)
% plot(nav_pose, nav_posn,'LineWidth', 2);
% xlabel('����λ��(m)');
% ylabel('ǰ��λ��(m)');
% title('ˮƽ�˶��켣');
% legend('Ƕ��ʽʵʱ����');
% grid;