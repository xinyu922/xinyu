function [ R, t ] = se3DataAlignment( X, Y )
%SO3DATAALIGNMENT �˴���ʾ�йش˺�����ժҪ ��һ��R,tʹ��Y=RX+t
%   �˴���ʾ��ϸ˵��ssss
n = size(X,2);
mu_X = sum(X,2)/n;  % mean of X
mu_Y = sum(Y,2)/n;  % mean of Y
x=X-repmat(mu_X,1,n);
y=Y-repmat(mu_Y,1,n);
W=zeros(3,3);
    for i=1:size(X,2)
        %W=W+x(:,i)*y(:,i)';����д���ǲ��Ե�
        W=W+y(:,i)*x(:,i)';
    end
[u,~,v]=svd(W);
R=u*v';
%t=mu_X-R*mu_Y;����д���ǲ��Ե�
t=mu_Y-R*mu_X;
end

