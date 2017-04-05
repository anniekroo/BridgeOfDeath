% rosinit('10.0.75.2',11311,'NodeHost','10.0.75.1')
syms u;
% encode the fact that u is a real number (allows simplifications)
assume(u,'real');
a = .4;
l = .4;
% create a symbolic expression for an ellipse
R = sym([.5*cos(u), .75*sin(u), 0]);

% compute the tangent vector
T = diff(R);
% compute That.  Simplify will make sure things are in a sane form.
That = simplify(T ./ norm(T));
N = simplify(diff(That));
Bhat = simplify(cross(That, N));
%-------------------------------------------------------------------------%
pub = rospublisher('/raw_vel');
sub_bump = rossubscriber('/bump');
msg = rosmessage(pub);
d = 0.24765;                              %m
w = Bhat(3);
v = simplify(norm(T));                                  %theoretical R of d
vR = w.*((v./w)+(d./2));
vL = w.*((v./w)-(d./2));
endTime = 20;
sub_enc = rossubscriber('/encoders');
%TODO figure out how to have echo not take over your program
%TODO write velocities to matrix
tStart = tic();
elapsed = 0;
pos = [];
sampleTime = .1;
timeStep = .1;
while elapsed <= endTime
    for i = 0:timeStep:endTime
        startLoopTime = tic();
        u = i/3;
        instVR = (double (subs(vR)))/3
        instVL = (double (subs(vL)))/3
        msg.Data = [instVL, instVR];
        send(pub, msg);
        pause(timeStep - toc(startLoopTime))
    end
    elapsed = toc(tStart);
end
msg.Data = [0,0];
send(pub, msg);
%-------------------------------------------------------------------------%
%rosshutdown