%function:自适应一阶低通滤波

% text = 'D:\导航融合项目\outdoor\adjust\航向滤波仿真数据\静态\NAV.txt';%矩形1静态
% A = importdata(text);
% X = A.data(:,2);
% time = A.data(:,1);
% row = length(X);
% 
% num_threshold = 0.1; count_threshold=6; weight_0=0.02; weight_acc=0.1;
% out = adaptive_low_pas(X, row, num_threshold, count_threshold, weight_0, weight_acc);
% plot( X);hold on;
% plot( out)
% 
% cut = out(1530:end);
% dd = max(cut) - min(cut)
% xstd = std(cut) 

function Y = adaptive_low_pass(X, row, num_threshold, count_threshold, weight_0, weight_acc)%各参数分别为滤波前数据、数据个数、数值绝对值阈值、数值连续同向变化程度衡量、初始新值权重、权重递增量

    num_count = 0;
%     inver_count = 0;
    old_flag = 0;
    old_num = X(1);
    weight = weight_0;
    lim = 300;
    for i = 2 : row
      if X(i) - X(i-1) > lim
           X(i) =  X(i)-360;
      elseif  X(i) - X(i-1) < -lim
           X(i) =  X(i)+360; 
      end
%       if X(i) - X(i-1) > lim
%            X(i-1) =  X(i-1)+360;
%       elseif  X(i) - X(i-1) < -lim
%            X(i-1) =  X(i-1)-360; 
%       end
      new_num = X(i);
      
      if ((new_num - old_num)>0)
          new_flag = 1;
      else
          new_flag = 0;
      end

      if (new_flag == old_flag) 
          num_count = num_count  + 1;  
          if (abs(new_num - old_num) > num_threshold)
              num_count = num_count  + 1;
          end
          if (num_count >= count_threshold)
              weight = weight + weight_acc;
              if (weight > 0.9)
                  weight = 0.9;
              end
          end
      else
%           inver_count = inver_count + 1;
%           if (inver_count  
          num_count = 0;
          weight = weight_0;
      end
      old_flag = new_flag;
      old_num = new_num;
      X(i) = X(i-1) * (1-weight) + weight * X(i);
     if X(i) >= 360
           X(i) =  X(i)-360;
      elseif  X(i) < 0
           X(i) =  X(i)+360;
     end
    end
    Y = X;
end