%funcion:实现sigma点先验预估值求解
%chuzhiwei
%2019.09.12
function X = sigma_predict(X, Gyro, T)
    X = X + T * [0 (sin(X(3)*pi/180))/(cos(X(2)*pi/180)) (cos(X(3)*pi/180))/(cos(X(2)*pi/180))
                 0         cos(X(3)*pi/180)                          -sin(X(3)*pi/180)
                 1 (tan(X(2)*pi/180))*(sin(X(3)*pi/180)) (tan(X(2)*pi/180))*(cos(X(3)*pi/180))] * Gyro'; 
    if (X(1) > 360)
        X(1) = X(1) - 360;
    elseif (X(1) < 1)
        X(1) = X(1) + 360;
    end
    
    if (X(2) > 90)
        X(2) = 180 - X(2);
    elseif (X(2) < -90)
        X(2) = -180 - X(2);
     end
     
     if (X(3) > 180)
        X(3) = X(3) - 360;
    elseif (X(2) < -180)
        X(3) = X(3) + 360;
    end
