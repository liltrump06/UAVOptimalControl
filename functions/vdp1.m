function dxdt =vdp1(t,x,D,g)
dxdt=zeros(6,1);
dxdt(1) = -D*x(1);
dxdt(2) = -D*x(2);
dxdt(3) = -D*x(3)+g;
dxdt(4) = x(1);
dxdt(5) = x(2);
dxdt(6) = x(3);
end

