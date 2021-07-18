%%滑动平均滤波
function [after_filter] = moving_average_filter(data, winSize)  %data滤波前数据，winSize滑动窗口大小

    row  = length(data);
    filter = zeros(row,1);
    W = zeros(winSize,1);
    k = 0;
    for i = 1 : winSize
        for j = 1 : i
            W(j) = data(j);
        end
        filter(i) = mean(W);
    end
    for i = winSize + 1 : row
        for j = i - winSize + 1 : i
            k = k + 1;
            W(k) = data(j);
        end
        filter(i) = mean(W);
        k = 0;
    end
    after_filter = filter;
    
end

