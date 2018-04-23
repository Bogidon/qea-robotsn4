rosinit('10.7.24.77',11311)

%%
vel_pub = rospublisher('/raw_vel');
vel_msg = rosmessage(vel_pub);
% accel_sub = rossubscriber('/accel');

%%
dt = 0.01;
time = 0:dt:15;
global accelData
accelData = [];

global rotateDir
rotateDir = 0.5;

rotatet = timer;
forwardt = timer;


forwardt.StartDelay = 1;
forwardt.StartFcn = @(~,~) forward(vel_pub,vel_msg);
forwardt.TimerFcn = @(~,~) start(rotatet);


rotatet.ExecutionMode = 'fixedDelay';
rotatet.Period = dt;
rotatet.StartDelay = 0;
rotatet.TasksToExecute = inf;
rotatet.StartFcn = @(~,~) rotate2(vel_pub, vel_msg);
rotatet.TimerFcn = @(mytimer,~) rotate(mytimer);
rotatet.StopFcn = @(~,~) start(forwardt);
start(rotatet)

% accelData(4,:) = [0 diff(accelData(3,:))];
% accelData(5,:) = [0 diff(accelData(2,:))];
%%
stop(forwardt)
stop(rotatet)
sendVel(0.0,0.0,vel_pub,vel_msg);

%%

accelData(4,:) = [0 diff(accelData(3,:))];
accelData(5,:) = [0 diff(abs(accelData(2,:)))];
accelData(6,:) = [0 0 diff(accelData(5,2:end))];
accelData(7,:) = [abs(accelData(2,:))];
clf
subplot(2,1,1)
hold on
plot(accelData(4,:), "g-*")
plot(accelData(5,:),"r-*")
plot(accelData(6,:),"k-*")
hold off
subplot(2,1,2)
hold on
plot(accelData(3,:), "g--")
plot(accelData(2,:), "r--*")
plot(accelData(7,:), "r:")
hold off
    
%% functions
function rotate(mytimer)
    global accelData
    accel = rostopic('echo', '/accel');
    accelData = [accelData accel.Data];
    
    if size(accelData,2) > 4
        firstD = diff(abs(accelData(2, :)));
        secondD = diff(firstD);
        secondD(end)
        if secondD(end) > 0.01
             stop(mytimer);
        end
    end
end

function rotate2(pub, msg)
    global rotateDir
    vl = rotateDir*(-0.03)
    vr = rotateDir*0.03
    sendVel(vl,vr,pub,msg)
end

function sendVel(vl, vr, pub, msg)
    sprintf("VL: %.4d, VR: %.4d", vl, vr);
    msg.Data = [vl vr];
    send(pub, msg);
end
function forward(pub,msg)
    global rotateDir
    rotateDir = rotateDir * -1
    sendVel(.15,.15,pub,msg);
end
