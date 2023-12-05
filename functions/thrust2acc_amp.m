function [f_norm,amp,flag]= thrust2acc_amp(af_norm,thrust)
%THRUST2ACC_AMP()  Summary of this function goes here
flag = true;
a = 1;
b = 2*9.81*af_norm(3);
c = 9.81^2-thrust^2;
amp = max(roots([a,b,c]));
if imag(amp)~=0
    flag = false;
end
fa = af_norm(1)*amp/thrust;
fb = af_norm(2)*amp/thrust;
fc = sqrt(1-fa^2-fb^2);
f_norm = [fa,fb,fc]';
end

