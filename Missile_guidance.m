clear,clc,close all
%% Setting
view(3);
% time derivative
dt = 0.01;
video_on = 0;

% Hardware spec
missle_speed = 200;
target_speed = 100;
missle_length = 2;
AngleOfAttack = 15*pi/180;

% Downtown's location
Downtown = [8200 1100 0]';

% Detect area
detect_area = 10;

% Target's initial location
Target_initial_location = ...
    [9000 9000 0]';
n = size(Target_initial_location);

% missle's initial location & orientation
Initial_location_missle = zeros(n);
Initial_orientation_missle = ones(n)/sqrt(3);

Downtown_location = zeros(n);
Initial_target_orientation = zeros(n);
Target = zeros(n(1),n(2),2);
Target_orientation = zeros(n);
current = zeros(n(1),n(2),2);
current_orientation = zeros(n);

for i = 1:n(2)
    Downtown_location(:,i) = Downtown + [randn(2,1)*50;0]; % target + noise + disturbance
    Initial_target_orientation(:,i) = [0 0 1]';
end

Target(:,:,1) = Target_initial_location;
Target_orientation(:,:) = Initial_target_orientation;
current_orientation(:,:) = Initial_orientation_missle;
target_vel_vector = zeros(n);
prev_vel_vector = zeros(n);
v_target = zeros(n);
check = zeros(n(2),1);

i = 1;
j = 1;
if(video_on==1)
    video = VideoWriter('Missle_guidance.avi');
    open(video);
end
%% calculation
while(i<8000)
    for t = 1:n(2)
        Target(:,t,i+1) = Target(:,t,i) + Target_orientation(:,t)*target_speed*dt;
        target_vel_vector(:,t) = Target_orientation(:,t)*target_speed;
        
        %방향 업데이트(타겟위치,현재위치,현재방향,dt,받음각,속도,길이)
        Target_orientation(:,t) = orientation_update(Downtown_location(:,t),Target(:,t,i+1),Target_orientation(:,t),dt,AngleOfAttack,target_speed,missle_length);
        
        if (Target(3,t,i+1) > detect_area)
            check(t) = 1;
        end
        
        % distance
        d(t) = norm(Target(:,t,end)-current(:,t,end));
        
        % target pursuit
        if(check(t) == 1)
            target_vel_vector(:,t) = (Target(:,t,end) - Target(:,t,end-1))/dt;
            prev_vel_vector(:,t) = (Target(:,t,end-1) - Target(:,t,end-2))/dt;
            alpha = acos(current_orientation(:,t)'*(Target(:,t,end)-current(:,t,end))/norm(Target(:,t,end)-current(:,t,end)));
            time = d(t)/(norm(target_vel_vector(:,t)-missle_speed*current_orientation(:,t))*(1-(alpha/(2*pi))^2)^1);
            rot = expm(skew(skew(prev_vel_vector(:,t)/norm(prev_vel_vector(:,t)))*target_vel_vector(:,t)/norm(target_vel_vector(:,t))));
            v_target(:,t) = virtual_target(Target(:,t,end),target_vel_vector(:,t),time,rot,dt);
            
            virtual_T(j,:) = v_target(:,t);
            current(:,t,j+1) = current(:,t,j) + current_orientation(:,t)*missle_speed*dt;
            current_orientation(:,t) = orientation_update(v_target(:,t),current(:,t,end),current_orientation(:,t),dt,AngleOfAttack,missle_speed,missle_length);
            
            % video
            if(video_on==1)
                view(3)
                figure(1);
                hold on
                x = squeeze(Target(1,:,end));
                y = squeeze(Target(2,:,end));
                z = squeeze(Target(3,:,end));
                plot3(x,y,z,'*r')
                
                x = squeeze(Target(1,:,:));
                y = squeeze(Target(2,:,:));
                z = squeeze(Target(3,:,:));
                plot3(x,y,z,'r')
                
                x = squeeze(current(1,:,end));
                y = squeeze(current(2,:,end));
                z = squeeze(current(3,:,end));
                plot3(x,y,z,'*b')
                
                x = squeeze(current(1,:,:));
                y = squeeze(current(2,:,:));
                z = squeeze(current(3,:,:));
                plot3(x,y,z,'b')
                if(norm(Target(:,t,end)-current(:,t,end))<5)
                    x = squeeze(Target(1,:,end));
                    y = squeeze(Target(2,:,end));
                    z = squeeze(Target(3,:,end));
                    plot3(x,y,z,'*r','markersize',20,'linewidth',20)
                end
                
                
                plot3(virtual_T(end,1),virtual_T(end,2),virtual_T(end,3),'*g')
                axis([-100 6000 -100 6000 0 2000])
                xlabel X
                ylabel Y
                zlabel Z
                grid on
                legend('Enemy','Enemy Trajectory','Missile','Missile Trajectory','Virtual target')
                
                frame = getframe(gcf);
                writeVideo(video,frame);
            end
            clf
            hold off
        end
        
        dist(i,t) = norm(Target(:,t,end)-current(:,t,end));
        
        % 지면에 도착
        if (Target(3,t,i+1) < 0)
            Target_orientation(:,t) = 0;
            current_orientation(:,t) = 0;
            continue;
        end
        
        % 격추
        if (norm(Target(:,t,end)-current(:,t,end))<5)
            Target_orientation(:,t) = 0;
            current_orientation(:,t) = 0;
            if(video_on==1)
                close(video);
            end
            break
        end
        
    end
    
    i = i+1;
    
    % 적의 미사일이 10 m 이상 올라오면 탐지되어 요격미사일이 발사된다.
    if(check(t) == 1)
        j = j+1;
    end
end
if(video_on==1)
    close(video);
end

%% plot
figure(1);
hold on
x = squeeze(Target(1,:,:));
y = squeeze(Target(2,:,:));
z = squeeze(Target(3,:,:));
plot3(x,y,z,'.r')

x = squeeze(current(1,:,:));
y = squeeze(current(2,:,:));
z = squeeze(current(3,:,:));
plot3(x,y,z,'.b')

%plot3(q(end,1),q(end,2),q(end,3),'.g')
plot3(virtual_T(:,1),virtual_T(:,2),virtual_T(:,3),'.g')
axis([-100 10000 -100 10000 0 2000])
grid on
legend('Enemy','Missile','Virtual target')
%% DownTown plot
figure(1)
hold on
target_zone_x = linspace(Downtown(1)-250,Downtown(1)+250);
target_zone_y = linspace(Downtown(2)-250,Downtown(2)+250);
[t_x,t_y] = meshgrid(target_zone_x,target_zone_y);
plot3(t_x,t_y,zeros(length(t_x)),'.g','markersize',0.1);
hold off

%% Distance plot
figure(2)
plot(dist)
xlabel ('Sampling time')
ylabel ('Distance')
grid on
[min_dist,T]=min(dist);
% 가장 근접했을 때의 거리
min_dist

% 소요시간
DurationOfTime=T*dt