function Y = outliers_eliminating(X, row, limit) %Ұֵ�޳�

 for i = 2 : row
      
      if (abs(X(i) - X(i-1)) > limit)
          X(i) = X(i-1);
      end
 end
 Y = X;