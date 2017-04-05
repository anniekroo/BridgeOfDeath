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
sub_enc = rossubscriber('/encoders');
%TODO figure out how to have echo not take over your program
%TODO write velocities to matrix
tStart = tic();
elapsed = 0;
endTime = 10;
pos = [];
sampleTime = .1;
while elapsed <= endTime
    for e = 1:(endTime*(1/sampleTime))
        a = tic();
        pos = [pos, sub_enc.LatestMessage.Data];
        pause(sampleTime-(toc(a)));
    end
    elapsed = toc(tStart);
end
msg.Data = [0,0];
send(pub, msg);

vR = pos(1,:);
vL = pos(2,:);
w = (pos(1,:)-pos(2,:))./0.24765;
dD = [];
for i = 2:(size(vR, 2))
    dD = [dD, pos(:,i)-pos(:,i-1)];
end
v = dD./sampleTime;
x = [];
y = [];
oldTurn = 0;
for i = 1:size(v, 2)
    newTurn = w(i)*sampleTime + oldTurn;
    x = [x, v(:,i).*cos(newTurn)];
    y = [y, v(:,i).*sin(newTurn)];
    oldTurn = newTurn;
end
clf
plot(x(2,:),y(2,:), '.')
hold on
plot(x(1,:),y(1,:), '.')
%rosshutdown