%function：计算Sigma点均值
%chuzhiwei
%2019.09.12
function Xhat = sigma_mean_value(X,W)
   Xhat =  W(1) * X(:,1) + W(2) * X(:,2) + W(3) * X(:,3) + W(4) * X(:,4) + W(5) * X(:,5) + W(6) * X(:,6) + W(7) * X(:,7);
   yaw_max = max(X(1,:));
   yaw_min = min(X(1,:));
%    if ((yaw_max - yaw_min) > 180)
%        Xhat(1) = 0;
%    end
   
   count = 0;
   sum_yaw = 0;
   maxval = 5;
   minval = 3;
    if((yaw_max-yaw_min)>maxval)
        count = 1;
        sum_yaw = X(1,1);
        for i = 2 : 7
            if(abs(X(1,1)-X(1,i))<minval)
                count = count + 1;
                sum_yaw = sum_yaw + X(1,i);
            end
        end
        if(count==1)
            sum_yaw = X(1,2);
            for i = 3 : 7
                if(abs(X(1,2)-X(1,i))<minval)
                    count = count + 1;
                    sum_yaw = sum_yaw + X(1,i);
                end
            end
        end
        if(count==1)
            sum_yaw = X(1,3);
            for i = 4 : 7
                if(abs(X(1,3)-X(1,i))<minval)
                    count = count + 1;
                    sum_yaw = sum_yaw + X(1,i);
                end
            end
        end
        if(count==1)
            sum_yaw = X(1,4);
            for i = 5 : 7
                if(abs(X(1,4)-X(1,i))<minval)
                    count = count + 1;
                    sum_yaw = sum_yaw + X(1,i);
                end
            end
        end
        if(count==1)
            sum_yaw = X(1,5);
            for i = 6 : 7
                if(abs(X(1,5)-X(1,i))<minval)
                    count = count + 1;
                    sum_yaw = sum_yaw + X(1,i);
                end
            end
        end
        if(count==1)
            sum_yaw = X(1,6);
                if(abs(X(1,6)-X(1,7))<minval)
                    count = count + 1;
                    sum_yaw = sum_yaw + X(1,7);
                end
        end
         Xhat(1) = sum_yaw / count;
    end
        
    