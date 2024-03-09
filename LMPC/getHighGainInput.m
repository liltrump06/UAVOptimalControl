function x = getHighGainInput(T,wd,wd_dot,x)
global quadrotorParameter

I = quadrotorParameter.I;
Ir = quadrotorParameter.Ir;
Kw = quadrotorParameter.Kw;
e3 = quadrotorParameter.e3;
w = x(16:18);
r= x(23:26);
Omega = [0 -wd(3) wd(2);wd(3) 0 -wd(1);-wd(2) wd(1) 0];

Ga = -(-r(1)*Ir*(cross(wd,e3))+r(2)*Ir*(cross(wd,e3))-r(3)*Ir*(cross(wd,e3))+r(4)*Ir*(cross(wd,e3)));
tau_d = I*wd_dot+Omega*I*wd-Ga;
w_error = w-wd;
tau = tau_d-Kw*w_error;
newr = force2rpm(T,tau);
x(23:26) = newr;
x(19) = T;
x(20:22) = tau;
