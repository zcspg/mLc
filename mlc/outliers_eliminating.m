function Y = outliers_eliminating(X, row, limit) %Ò°ÖµÌŞ³ı

 for i = 2 : row
      
      if (abs(X(i) - X(i-1)) > limit)
          X(i) = X(i-1);
      end
 end
 Y = X;