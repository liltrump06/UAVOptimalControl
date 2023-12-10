function flag = check_feasibility_lite(constrain_profile,thrust,omegalist,jtraj)
%CHECK_FEASIBILITY Summary of this function goes here
% Since angular velocity and thrust timeseries are already solved, there is
% no need to apply the fast but complex algorithm to check the feasibility;
fmax = constrain_profile.fmax;
fmin = constrain_profile.fmin;
wmax = constrain_profile.wmax;
flag = true;
if(max(thrust)>fmax || min(thrust)<fmin)
    flag = false;
end
w1m = max(abs(omegalist(1,:)));
w2m = max(abs(omegalist(2,:)));
if w1m^2+w2m^2 > wmax^2
    flag = false;
end
end

