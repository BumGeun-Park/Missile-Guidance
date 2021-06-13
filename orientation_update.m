function new_orientation = orientation_update(target,current,orientation,dt,AngleOfAttack,v,L)
unit_vector2target = (target-current)/norm(target-current);  % 현재위치에서 타겟으로의 단위벡터
rot_vector = skew(orientation)*unit_vector2target; % 현재 방향과 타겟으로의 단위벡터를 외적하여 회전벡터를 구한다.

if(norm(rot_vector)/dt > (v/L)*tan(AngleOfAttack))
    rot_vector = rot_vector/norm(rot_vector)*dt*(v/L)*tan(AngleOfAttack);
end
if(norm(rot_vector)/dt < -(v/L)*tan(AngleOfAttack))
    rot_vector = -rot_vector/norm(rot_vector)*dt*(v/L)*tan(AngleOfAttack);
end
Rotation = expm(skew(rot_vector)*dt); % 회전행렬을 구한다.
new_orientation = Rotation*orientation; % 회전행렬을 이용하여 새로운 방향으로 수정한다.
end