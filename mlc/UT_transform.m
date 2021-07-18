%funcion:实现UT变换中的sigma点求取
%chuzhiwei
%2019.09.12
function sigma = UT_transform(Xhat,sp)

     X0 = Xhat';
     X1 = Xhat' - sp(:,1);
     X2 = Xhat' - sp(:,2);
     X3 = Xhat' - sp(:,3);
     X4 = Xhat' + sp(:,1);
     X5 = Xhat' + sp(:,2); 
     X6 = Xhat' + sp(:,3); 

     sigma = [X0, X1, X2, X3, X4, X5, X6];
