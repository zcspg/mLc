%function:自适应一阶低通滤波
clear; clc
close all;
filepath = 'D:\导航融合项目\outdoor\adjust\航向滤波仿真数据\定点旋转\';%矩形1\矩形2
text = [filepath , 'NAV.txt'];
gps  = [filepath , 'GPS2.txt'];
A = importdata(text);
B = importdata(gps);
X = A.data(:,2);
gyaw = B.data(:,9);

% one order diff yaw
dyaw = diff(gyaw);
syaw = ones(length(dyaw), 1);
for i = 1:length(dyaw)
    if dyaw(i) > 300 
         dyaw(i) =  dyaw(i) -360;
    end
    if dyaw(i) < -300 
         dyaw(i) =  dyaw(i) +360;
    end
    if i>2
        dyaw(i) = dyaw(i)*0.55 + dyaw(i-1)*0.45;
    end
    if dyaw(i) < 0
        syaw(i) = -1;
    end
end
time = A.data(:,1);
time2 = B.data(:,1);
row = length(X);
% cut = out(1530:end);
% dd = max(cut) - min(cut)
% xstd = std(cut) 


% two order diff yaw (one order for yaw sign)
figure
ddyaw = diff(syaw);hold on
ddyaw = [0; 0; ddyaw];
plot(time2, ddyaw);
isused =  zeros(length(ddyaw), 1);

figure
num_threshold = 0.0005; count_threshold=6; weight_0=0.02; weight_acc=0.1;
[out, wei] = adaptive_low_pas(X, row, num_threshold, count_threshold, weight_0, weight_acc, time, time2, ddyaw, isused);
plot(time, X);hold on;
plot(time, out)
figure
plot(wei);

function [Y,weight] = adaptive_low_pas(X, row, num_threshold, count_threshold, weight_0, weight_acc, time, time2, ddyaw, isused)%各参数分别为滤波前数据、数据个数、数值绝对值阈值、数值连续同向变化程度衡量、初始新值权重、权重递增量

    num_count = 0;
%     old_flag = 0;
    old_num = X(1);
    weight = weight_0*ones(length(X), 1);
  
    lim = 300;
    j=1;
    for i = 2 : row
      if(j<length(time2)-1)
          while(time2(j+1) < time(i))
             j = j+1; 
          end
      end
      
      % adjust to the same region
      if X(i) - X(i-1) > lim
           X(i) =  X(i)-360;
      elseif  X(i) - X(i-1) < -lim
           X(i) =  X(i)+360;
      else
          
      end
      
  
      %judge = abs(new_num - old_num) > num_threshold;       % change obviously
      judge = 1;
      if (judge && isused(j)==0)
          if ( ddyaw(j) == 0)
              new_flag = 0;
              isused(j) = 1;
               now = 1;
          else
              new_flag = 1;
              isused(j) = 1;
               now = 1;
          end
      else
           now = 0;
      end
      if( now ==1)
          if (new_flag == 0) 
              num_count = num_count  + 1;  
              if (num_count >= count_threshold)
                  weight(i) = weight(i-1) + weight_acc;
                  if (weight(i) > 0.9)
                      weight(i) = 0.9;
                  end
              end
          elseif (new_flag ~= 0) 
              num_count = 0;
              weight(i) = weight_0;
          end
      else
           weight(i) = weight(i-1);           
      end
    %  old_flag = new_flag;
%       old_num = new_num;
      X(i) = X(i-1) * (1-weight(i)) + weight(i) * X(i);
     if X(i) > 360
           X(i) =  X(i)-360;
      elseif  X(i) < 0
           X(i) =  X(i)+360;
     else  
         
     end
    end
    Y = X;
end