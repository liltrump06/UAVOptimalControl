function mpcoutput = solveMPC(mpcinput)
global quadrotorParameter
m = quadrotorParameter.m;
g = quadrotorParameter.g;
c = quadrotorParameter.c;
b = quadrotorParameter.b;
pd = mpcinput.pd;
vd = mpcinput.vd;
ad = mpcinput.ad;
N = mpcinput.N;
x = mpcinput.x;

Fd = Fd_gene(vd,ad);
F = F_gene(x);
Q = eye(9)*10;
P = eye(3);
A22 = [0,0,0;0,0,0;0,0,-g(3)*c];
% A 9x9, B 9x3 
A = [zeros([3,3]),eye(3),zeros([3,3]);zeros([3,3]),A22,1/m*eye(3);zeros([3,3]),zeros([3,3]),-b*eye(3)];
B = zeros([9,3]);
B(7:9,:) = eye(3);
zd = [pd;vd;Fd];
z0 = [x(1:6);F];
% cvx_begin quiet
%     variable u(3,N+1);
%     variable z(9,N+1);
%     ze = z - zd;
%     sums = 0;
%     for i = 1:6
%         sums = sums+ ze(:,i)'*Q*ze(:,i)+u (:,i)'*P* u(:,i);
%     end
%     % minimize sum(diag(ze'*Q*ze))+sum(diag(u'*P*u))
%     minimize sums
%     subject to  
%         z(:,1) == A*z0+B*u(:,1);
%         for i = 2 : N+1
%             z(:,i) == A*z(:,i-1)+B*u(:,i);
%         end
% cvx_end
%%
MM = [];
LL = [];
LL0 = zeros(9,3*(N-1));
for i = 1:N-1
   LL0(:,4:3*(N-1)) = LL0(:,1:3*(N-1)-3);
   LL0(:,1:3) = A^(i-1)*B;
   MM = [MM;A^i];
   LL = [LL;LL0];
end
Qhat = Q;
Phat = P;
zdnew = [];
for i = 1:N-2
    Qhat = blkdiag(Qhat,Q);
    Phat = blkdiag(Phat,P);
    zdnew = [zdnew;zd(:,i)];
end
zdnew = [zdnew;zd(:,N-1)];
HH = LL'*Qhat*LL+Phat;
SS = LL'*Qhat*MM;
RR = LL'*Qhat;

u = HH\(RR*zdnew-SS*z0);
uopt = u(1:3,:);
% uopt = u(:,1);
mpcoutput.uopt = uopt;
mpcoutput.F = F;




