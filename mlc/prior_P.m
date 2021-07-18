%function:计算先验误差协方差矩阵
%chuzhiwei
%2019.09.12
function P = prior_P(W, X, Xhat, Q)
   P = W(1) * ( X(:,1)-Xhat' ) * ( X(:,1)-Xhat' )' +  W(2) * ( X(:,2)-Xhat' ) * ( X(:,2)-Xhat' )' +  W(3) * ( X(:,3)-Xhat' ) * ( X(:,3)-Xhat' )'   +  W(4) * ( X(:,4)-Xhat' ) * ( X(:,4)-Xhat' )'+  W(5) * ( X(:,5)-Xhat' ) * ( X(:,5)-Xhat' )' +  W(6) * ( X(:,6)-Xhat' ) * ( X(:,6)-Xhat' )'  +  W(7) * ( X(:,7)-Xhat' ) * ( X(:,7)-Xhat' )' + Q;