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
tStart = tic();                         %establishes start time of while loop
elapsed = 0;                            %establish elapsed
while elapsed <= endTime
    elapsed = toc(tStart);              %check to determine whether to stop neto
end
msg.Data = [0,0];                       %writes zero velocities to data message object neto
send(pub, msg);                         %sends zero velocities to robot
%rosshutdown