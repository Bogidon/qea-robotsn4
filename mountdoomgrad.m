rosinit('10.7.28.107',11311)

%%
vel_pub = rospublisher('/raw_vel');
vel_msg = rosmessage(vel_pub);

%% set up timers
dt = 0.5;
global accelData
accelData = [];

rotatet = timer;
forwardt = timer;

forwardt.StartDelay = .45;
forwardt.StartFcn = @(~,~) sendVel(.13,.13,vel_pub,vel_msg);
forwardt.TimerFcn = @(~,~) start(rotatet);

rotatet.ExecutionMode = 'fixedDelay';
rotatet.Period = dt;
rotatet.StartDelay = 0.1;
rotatet.TasksToExecute = inf;
rotatet.StartFcn = @(~,~) sendVel(0.0,0.0,vel_pub,vel_msg);
rotatet.TimerFcn = @(mytimer,~) rotate(mytimer, vel_pub, vel_msg);
rotatet.StopFcn = @(~,~) start(forwardt);
start(rotatet)

axis equal
%% e-brake
stop(forwardt)
stop(rotatet)
sendVel(0.0,0.0,vel_pub,vel_msg);

%% functions
function rotate(mytimer, pub, msg)
    accel = rostopic('echo', '/accel');
    
    v = accel.Data;
    r = rotmatrix(v);
    vv = r'*v;
    h = r'*[1 ; 0 ; 0];
    
    if abs(h(3)) < .168
        sendVel(0.0,0.0,pub,msg);
        return
    end
    
    angle = atand(h(2)/h(1))
    if abs(angle) < 5
        stop(mytimer)
    elseif angle < 0
        sendVel(-.03,.03, pub, msg);
    else
        sendVel(.03, -.03, pub, msg);
    end
end

function out = rotmatrix(w)
    wyaw = atan(w(2)/w(1));
    wryaw = [cos(wyaw) -sin(wyaw) 0; sin(wyaw) cos(wyaw) 0; 0 0 1];
    w = wryaw'*w;

    wpitch = atan(w(1)/w(3));
    wrpitch = [cos(wpitch) 0 sin(wpitch); 0 1 0; -sin(wpitch) 0 cos(wpitch)];
    w = wrpitch'*w;

    wroll = atan(w(3)/w(2));
    wrroll = [1 0 0; 0 cos(wroll) -sin(wroll); 0 sin(wroll) cos(wroll)];
    w = wrroll'*w;

    wr = wryaw * wrpitch;
    out = wr;
end

function sendVel(vl, vr, pub, msg)
    sprintf("VL: %.4d, VR: %.4d", vl, vr);
    msg.Data = [vl vr];
    send(pub, msg);
end
