function p_v = virtual_target(p_c,v,t,rotation,dt)
if(isnan(rotation))
    p_v = p_c;
    return
end
if norm(logm(rotation))>0.0001
    for i = 1:t/dt
        v = rotation*v;
        p_c = p_c+v*dt;
    end
    p_c = p_c + v*(t/dt-i);
else
    p_c = p_c + v*t;
end
if(p_c(3)<10)
    p_c(3) = 10;
end
p_v = [p_c(1),p_c(2),p_c(3)]';
end