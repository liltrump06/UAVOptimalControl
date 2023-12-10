function flag = check_feasibility(alpha,beta,gamma,T0,Tf,a0,constrain_profile)
gz=-9.81;
fmax = constrain_profile.fmax;
fmin = constrain_profile.fmin;
wmax = constrain_profile.wmax;
PASS = 2;
NOTSURE = 1;
FAIL = 0;
min_interval = 0.1;
flag = recursive_check(T0,Tf);

function flag = recursive_check(T0,Tf)
    if (Tf-T0)<min_interval
        flag = FAIL;
        return
    end
    flag = NOTSURE;
    flagw = NOTSURE;
    flaga = NOTSURE;% 
    amax2list = [];
    amin2list = [];
    jmax2list = [];
    for i=1:3
        root = roots([alpha(i)/2,beta(i),gamma(i)]);
        at0 = get_acc(alpha(i),beta(i),gamma(i),a0(i),T0);
        atf = get_acc(alpha(i),beta(i),gamma(i),a0(i),Tf);
        if(sum(imag(root)) ==0 && size(root,1)==2)
            ar1= get_acc(alpha(i),beta(i),gamma(i),a0(i),root(1));
            ar2 = get_acc(alpha(i),beta(i),gamma(i),a0(i),root(2));
            if(i==3)
                at0 = at0 - gz;
                atf = atf - gz;
                ar1 = ar1 - gz;
                ar2 = ar2 - gz;
            end
            amax2 = max([at0^2,atf^2,ar1^2,ar2^2]);
            amin2 = min([at0^2,atf^2,ar1^2,ar2^2]);
        else
            if(i==3)
                at0 = at0 - gz;
                atf = atf - gz;
            end
            amax2 = max([at0^2,atf^2]);
            amin2 = max([at0^2,atf^2]);
        end
        
        if amax2 > fmax^2
            flag = FAIL;
        end
        
        jt0 = get_jerk(alpha(i),beta(i),gamma(i),T0);
        jtf = get_jerk(alpha(i),beta(i),gamma(i),Tf);
        if(alpha(i)>=0)
           jmax2 =  max([jt0^2,jtf^2]);
        else
            jr = get_jerk(alpha(i),beta(i),gamma(i),-beta(i)/alpha(i));
            jmax2 =  max([jt0^2,jtf^2,jr^2]);
        end
        amax2list = [amax2list,amax2];
        amin2list = [amin2list,amin2];
        jmax2list = [jmax2list,jmax2];
    end
    
    wbar2 = sum(jmax2list)/sum(amin2list);
    amaxbar2 = sum(amax2list);
    aminbar2 = sum(amin2list);
    if wbar2<=wmax^2
        flagw = PASS;
    end
    
    if amaxbar2<=fmax^2 && aminbar2>=fmin^2
        flaga = PASS;
    end
    if(flaga == PASS && flagw == PASS && flag ~= FAIL)
        flag = PASS;
    end
    if flag == NOTSURE
        Thalf = (Tf+T0)/2;
        flag1 = recursive_check(T0,Thalf);
        flag2 = recursive_check(Thalf,Tf);
        if flag1 == PASS && flag2 == PASS
            flag = PASS;
        else 
            flag = FAIL;
        end
    end
end
end

