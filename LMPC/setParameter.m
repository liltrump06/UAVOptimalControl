global quadrotorParameter
quadrotorParameter.Ts = 0.01;


quadrotorParameter.m = 1;
quadrotorParameter.c = 0.01;
quadrotorParameter.g = [0,0,9.8]';
quadrotorParameter.I = diag([0.01,0.01,0.02]);
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
% coeff beta in paper
quadrotorParameter.b = 0.1;

%% high gain control of omega
% controller gain
quadrotorParameter.Kw = diag([0.2,0.2,0.05]); 




