global quadrotorParameter
quadrotorParameter.Ts = 0.001;


quadrotorParameter.m = 1;
quadrotorParameter.c = 0.01;
quadrotorParameter.g = 9.81;
quadrotorParameter.I = diag([0.001,0.001,0.002]);
quadrotorParameter.Ir = 0.0001;
quadrotorParameter.e3 = [0,0,1]';
%% calculate tau thrust from rpm
% one motor rpm to thrust coeff 
quadrotorParameter.C0 = 1e-3;
quadrotorParameter.C1 = 1e-6;

% distance between motor center and center of mass
quadrotorParameter.l = 0.1;

% some coeff to compute tau
quadrotorParameter.k = 1e-5;

%% parameter about mpc 

Q11 = diag([150,150,1300]);
Q12 = diag([200,200,200]);
Q13 = diag([1,1,1]);
Q33 = diag([150,150,1300]);
% Q = [Q11,Q12,Q13;Q12,zeros(3,3),zeros(3,3);Q13,zeros(3,3),zeros(3,3)];
Q = diag([1500,1500,3500,100,100,100,1000,1000,3500]);
P = eye(3);

quadrotorParameter.Q = Q;
quadrotorParameter.P = P;




% coeff beta in paper
quadrotorParameter.b = 0.1;





%% high gain control of omega
% controller gain
quadrotorParameter.Kw = diag([0.3,0.3,0.05]); 
quadrotorParameter.Kf = eye(3)*1; 




