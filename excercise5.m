% rosinit('10.0.75.2',11311,'NodeHost','10.0.75.1')
pub = rospublisher('/raw_vel');
sub_bump = rossubscriber('/bump');
msg = rosmessage(pub);
d = 0.24765;                              %m
w = pi/5;
v = d*w;                                  %theoretical R of d
vR = w*((v/w)+(d/2));
vL = w*((v/w)-(d/2));
endTime = 2*pi/w;
msg.Data = [vL, vR];
send(pub, msg);
tStart = tic();
elapsed = 0;
while elapsed <= endTime
    elapsed = toc(tStart);
end
msg.Data = [0,0];
send(pub, msg);
%rosshutdown