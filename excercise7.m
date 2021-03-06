%To initialize run the following line to set up ros's connection to matlab
% rosinit('10.0.75.2',11311,'NodeHost','10.0.75.1')
%-------------------------------------------------------------------------%
pub = rospublisher('/raw_vel');         %set up publisher for velocity to be written to later
msg = rosmessage(pub);                  %establishes message object containing publisher
d = 0.24765;                            %distance between wheels in meters
w = pi/5;                               %angular velocities
v = d*w;                                %linear velocity
vR = w*((v/w)+(d/2));                   %right wheel velocity
vL = w*((v/w)-(d/2));                   %left wheel velocity
endTime = 2*pi/w;                       %time it theoretically takes to complete one circle
msg.Data = [vL, vR];                    %writes velocities to message data object
send(pub, msg);                         %sends velocities to wheels
sub_enc = rossubscriber('/encoders');   %set up subscriber for encoders to be written to later
tStart = tic();                         %creating start time
elapsed = 0;
endTime = 10;                           %run time
pos = [];
sampleTime = .1;                        %time step to change velocities at
while elapsed <= endTime
    for e = 1:(endTime*(1/sampleTime))
        a = tic();
        pos = [pos, sub_enc.LatestMessage.Data]; %creating matrix of positions
        pause(sampleTime-(toc(a)));
    end
    elapsed = toc(tStart);
end
msg.Data = [0,0];                       %stopping robot after end time
send(pub, msg);
%-------------------------------------------------------------------------%
%POST PROCESSING
vR = pos(1,:);                          %
vL = pos(2,:);
w = (pos(1,:)-pos(2,:))./0.24765;
dD = [];
for i = 2:(size(vR, 2))
    dD = [dD, pos(:,i)-pos(:,i-1)]; %difference of positions
end
v = dD./sampleTime;
x = [];
y = [];
oldTurn = 0;
for i = 1:size(v, 2)
    newTurn = w(i)*sampleTime + oldTurn;    %calculating new theta
    x = [x, v(:,i).*cos(newTurn)];          %calculating new position x
    y = [y, v(:,i).*sin(newTurn)];          %calculating new position y
    oldTurn = newTurn;
end
clf
plot(x(2,:),y(2,:), '.')                    %plotting right wheel
hold on
plot(x(1,:),y(1,:), '.')                    %plotting left wheel
%rosshutdown