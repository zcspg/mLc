clear all
buf_cnt = [];
close all
%text = 'D:\导航融合项目\outdoor\adjust\矩形行驶动态\NAV.txt';
%text = 'D:\导航融合项目\outdoor\adjust\室外静止\NAV.txt';
text = 'D:\导航融合项目\outdoor\adjust\航向滤波仿真数据\矩形1\NAV.txt';
A = importdata(text);

% len =100 , pa = 1.3  Pz
% len = 50, pa = 2.0  Vz
aori  = A.data(:,2);    %  2  7  10  linspace(200,100,1000) + rand(1,1000);
for i = 1:length(aori)
   [a(i),buf_cnt] =  filter(aori(i), buf_cnt);
   %plot(a)
end
figure
plot(aori,'b');hold on
plot(a,'r','LineWidth',0.5)

function [out,buf_cnt] = filter(data_in, buf_cnt)

    persistent SLIDE_WINDOW_LEN ave 
    persistent weight
	if isempty(buf_cnt)
        SLIDE_WINDOW_LEN = 20;
        buf_cnt = 0;
        ave = data_in;
        weight = 0;
    end
    
    if(buf_cnt < SLIDE_WINDOW_LEN)	 
		buf_cnt = buf_cnt+1;
        weight = 1 / buf_cnt;       
        if(buf_cnt > SLIDE_WINDOW_LEN * 0.5)
            weight = weight*1.3; 
        end
    end
    ave = ave * (1-weight) + data_in * weight;
 	out = ave;
end



% float filter_moving_average_single(float data_in, uint8_t buf_len, float add_weight)
% {
% 	static uint8_t  buf_cnt = 0;
% 	static float  buf_ave = 0;	
%     static float  weight = 1.0f;
% 	
%     if(buf_cnt < buf_len)
% 	{   
% 		buf_cnt ++;
%         weight = 1.0f / buf_cnt;        
%         if (buf_cnt > buf_len*0.5)   
%         {        
%             weight *= add_weight;
%         }
% 	}
%     
%     buf_ave = buf_ave * (1-weight) + data_in * weight;       
%     float result = buf_ave;
% 	return result;
% }
