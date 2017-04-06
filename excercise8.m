%To initialize run the following line to set up ros's connection to matlab
% rosinit('10.0.75.2',11311,'NodeHost','10.0.75.1')
%-------------------------------------------------------------------------%
syms u;
% encode the fact that u is a real number (allows simplifications)
assume(u,'real');
% create a symbolic expression for ellipse
R = sym([.5*cos(u), .75*sin(u), 0]);

% compute the tangent vector
T = diff(R);
% compute That.  Simplify will make sure things are in a sane form.
That = simplify(T ./ norm(T));
N = simplify(diff(That));
Bhat = simplify(cross(That, N));
%-------------------------------------------------------------------------%
pub = rospublisher('/raw_vel'); %set up publisher for velocity to be written to later
msg = rosmessage(pub);          %set up message of publisher for velocity
d = 0.24765;                    %m
w = Bhat(3);                    %equating Bhat to angular velocity
v = simplify(norm(T));          %theoretical R of d

vR = w.*((v./w)+(d./2));        % Equations for the wheels of the neto robot 
vL = w.*((v./w)-(d./2));        % given angular velocity and linear velocity
endTime = 17;                   % stops robot after set amount of time
timeStep = .1;                  % determines how often we change our velocities

tStart = tic();                 % starts timer counting how long the program has been running
while elapsed <= endTime        % checks that the program has been running for less than endTime
    for i = 0:timeStep:endTime  % runs through each time step and writes to neto
        startLoopTime = tic();  % establishes start time for loop to check how long it takes to run loop
        u = i/4.5;              % creates a u value to be substituted into symbolic function
        instVR = (double (subs(vR)))/4.5;   % creates instantaneous velocity for right wheel
        instVL = (double (subs(vL)))/4.5;   % creates instantaneous velocity for right wheel
        msg.Data = [instVL, instVR];        % writes data to data object in message
        send(pub, msg);                     % sends message to neto
        pause(timeStep - toc(startLoopTime))% delays re-running the loop for timeStep seconds from the start of the loop
    end
    elapsed = toc(tStart);                  %recreates elapsed to check whether or not the while loop should end
end

msg.Data = [0,0];               % writes 0 velocities to data object of message
send(pub, msg);                 % sends 0 velocities to neto
%rosshutdown