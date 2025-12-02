

%
% Project01
% EML 4312
% Rick Lind
%

%--------------------------------------------------------------------
%   Define parameters for simulation
%--------------------------------------------------------------------


%
% Define time vector
%
Tstep = 0.01;                         % define time step 
Tstop = 5.0;                         % define time to stop
t = [0:Tstep:Tstop]';                % define time vector

%
% Define commands
%
Xcommand = zeros(1,length(t));       % initialize command to be 0
Xcommand(2/Tstep:length(t)) = 22;    % step command starts at 2 seconds
Ycommand = zeros(1,length(t));       % initialize command to be 0
Ycommand(4/Tstep:length(t)) = 12;    % step command starts at 4 seconds

%
% Velocity
%
Vo = 70*1.467;                        % velocity is 70 mph (convert to fps)


%--------------------------------------------------------------
% Make the x-position autopilot
%--------------------------------------------------------------

%
% Define the plant model
%
P = tf(5,[1 5 0]);                    % plant is P

%
% Make a root locus for Kx
%
figure(1)                             % open new figure
rlocus(P)                             % plot root locus
title('Figure 1: Root Locus for X-Position Controller (Kx)')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

%
% STUDENT : Choose control gains
%
Kp = 2.189;                           % proportional controller
Ki = 0.078;                           % integral controller
Kd = 0.157;                           % derivative controller

%   
% Define PID controller
%
KP = Kp;                              % define proportional controller
KD = tf([Kd 0],[1]);                  % define derivative controller
KI = tf([Ki],[1 0]);                  % define integral controller
Kx = parallel(KP,KI);                 % define PI 
Kx = parallel(Kx,KD);                 % define PID

%
% Make the closed-loop system
%
PK = series(Kx,P);                    % loop gain
S = feedback(PK,1);                   % close the loop

%
% Command increase of 22 feet
%
figure(2)                             % open new figure
opt = stepDataOptions;                % define options for step command
opt.StepAmplitude = 22;               % change magnitude to 22
x = step(S,t,opt);                    % compute step response
plot(t,x);                            % plot data
title('Figure 2: X-Position Step Response (22 ft command at t=2s)')
xlabel('Time (seconds)')
ylabel('X Position (feet)')
grid on

%
% Save the system                      
%
Xsystem = S;                          % rename as Xsystem



%--------------------------------------------------------------
% Make the y-position autopilot
%--------------------------------------------------------------


%
% Make the y-position autopilot
%
P = tf(100,[1 100]);                  % plant is P CHANGED THE PLANT 
I = tf(Vo,[1 0]);                     % integrator is I

%
% Make a root locus for Kphi
%
figure(3)                             % open new figure
rlocus(P);                            % plot root locus
title('Figure 3: Root Locus for Turn Angle Controller (Kphi)')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

%
% STUDENT : Choose control gains
%
Kp = 0.560;                           % proportional controller
Ki = 0.0;                             % integral controller
Kd = 0.0;                             % derivative controller


%
% Define PID controller
%
KP = Kp;                              % define proportional controller
KD = tf([Kd 0],[1]);                  % define derivative controller
KI = tf([Ki],[1 0]);                  % define integral controller
Kphi = parallel(KP,KI);               % define PI
Kphi = parallel(Kphi,KD);             % define PID

%
% Make closed-loop system with Kphi
%
PK = series(Kphi,P);                   % loop gain
T = feedback(PK,1);                    % close the lop with Kphi

%
% Plot step response to turn angle
%
figure(4)                              % open new figure
step(T)                                % plot step response
title('Figure 4: Turn Angle (Phi) Step Response')
xlabel('Time (seconds)')
ylabel('Turn Angle (radians)')
grid on

%
% Include integrator to convert velocity to position
%
TI = series(T,I);                      % include integrator with loop gain

%
% Make root locus for Ky
%
figure(5)                              % open new figure
rlocus(TI)                             % plot root locus
title('Figure 5: Root Locus for Y-Position Controller (Ky) with Integrator')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

%
% STUDENT : Choose a control gain
%
Kp = 0.0236;                          % proportional controller
Ki = 0.0;                             % integral controller
Kd = 0.0;                             % derivative controller

%
% Define PID controller
%
KP = Kp;                               % define proportional controller
KD = tf([Kd 0],[1]);                   % define derivative controller
KI = tf([Ki],[1 0]);                   % define integral controller
Ky = parallel(KP,KI);                  % define PI
Ky = parallel(Ky,KD);                  % define PID

%
% Make closed-loop system with Ky
%
TK = series(Ky,TI);                    % include Ky with loop gain
S = feedback(TK,1);                    % close the loop with Ky

%
% Command increase of 12 feet
%
figure(6)                              % open new figure
opt = stepDataOptions;                 % variables for step command
opt.StepAmplitude = 12;                % set magnitude to 12
z = step(S,t,opt);                     % compute step esponse

%
% Plot response
%
% We would not start the lane change until after x acceleration
%   so delay the step response in y for 2 second
%
for i=1:length(t)
    if i <= 200
        y(i)=0;
    else
        y(i)=z(i-200);
    end
end
plot(t,y);
title('Figure 6: Y-Position Step Response (12 ft command at t=4s)')
xlabel('Time (seconds)')
ylabel('Y Position (feet)')
grid on

%
% Save the system
%
Ysystem = S;

%--------------------------------------------------------------------
%   Make movie of car driving
%--------------------------------------------------------------------
[M,t,Xus,Yus,Xother1,Yother1,Xother2,Yother2] = ...
        project01_movie(Xsystem,Ysystem);


%
% Play the movie
% movie(M,1,5)



