function X=blockrepeat(A,N)
% Creates a block diagonal matix with the same block repeated N times
X = kron(eye(N),A);