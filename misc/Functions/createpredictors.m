function [H,S]=createpredictors(F,G,N)
% Create H and S in the prediction X = Hx(k)+SU
% System dimension
[n,m] = size(G);
% H (F_pred)
H = zeros(n*N,n);
for j=1:N
    H(1+n*(j-1):n*j,:)=F^(j-1);
end;
% S (G_pred)
S = zeros(N*n,N*m);
for j=2:N
    for k=1:j-1
        S(1+n*(j-1):n*j,1+m*(k-1):k*m)=F^(j-k-1)*G;
    end;
end;
