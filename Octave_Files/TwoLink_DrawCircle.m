% Description : Implements independent joint control of a two-link manipulator as
% it traces a circular trajectory
% Author: Luis A. Rodriguez
% Email: rodriguez@msoe.edu
% Date: 1/15/2021
clear all; clc

% Set up parameters for Two-Link(in SI units)
L1 = 0.25; r1 = L1/2; % meters
L2 = L1;   r2 = L2/2; % meters
m1 = 0.5;  m2 = m1;   % kg
b1 = 1e-1; b2 = b1; % Nm-s/rad, Damping
g = 9.81;  % m/s^2, Gravity

% Create an instance of a Two-Link Robot System
RR = TwoLink(L1,r1,L2,r2,m1,m2,g,b1,b2);

% Simulation and solver options
tinit = 0; tfinal = 10; dt = 0.01; %Sample Time
%time  = t0:dt:tf;           % simulation time vector
state0 = [pi/2 0 0 0]; % Robot initial Condition

opts = odeset('RelTol',1e-3,'AbsTol',1e-6);% Solver default values

% Path Trajectory Properties
tdwell  = 4;    % Robot dwells for tcircle-tdwell seconds
tcircle = 5;    % robot starts drawing circle after tcircle seconds
r = 0.1;        % meters, circle radius
xc = 0.2; yc = 0.2; % circle center
xCirc0 = xc + r; yCirc0 = yc; % First point on the circle
[xe0, ye0] = RR.fwdK(state0(1:2)); % End-effertor position at t=0

T = 4;       % circle sample period
omega = 2*pi/T; % rad/s, angular velocity of circle 

% Controller Properties with saturation limits (PID with saturation)
kp = [20 0;0 20]; ki = [40 0;0 40]; kd = [2 0; 0 .1]; 
uMin = -10; uMax = 10 ;
uOffset = 0; % Integrator initial condition and controller offset
tNextControl = 0.0; % starting control time

theta1_Cntrl = PIDController(kp(1,1), ki(1,1), kd(1,1), dt, uMin, uMax, uOffset);
theta2_Cntrl = PIDController(kp(2,2), ki(2,2), kd(2,2), dt, uMin, uMax, uOffset);


% Allocate Memory
npnts = floor((tfinal-tinit)/dt)+1;
time = inf(npnts,1); state = inf(npnts,4);
thd  = inf(npnts-1,2);
torques = inf(npnts-1,2);
% Initialize simulation time vector and state 
time(1) = tinit; state(1,:) = state0;

for i = 1:numel(time)-1
    % Current time and state
    t = time(i); x = state(i,:);
    tNext = t + dt; %Next simulation time or end of interval
    
    % Define Aliases
    th = x(1:2);
    
    % Define desired trajectory
    if (t <= tdwell)
      xe_d(i) = (xCirc0 - xe0)/tdwell*t + xe0;
      ye_d(i) = (yCirc0 - ye0)/tdwell*t + ye0;
    elseif (t <= tcircle) % Dwell phase
        xe_d(i) = xCirc0; ye_d(i) = yCirc0;
    else % start generating circle trajectory
        xe_d(i) = xc + r*cos(omega*(t-tcircle)); ye_d(i) = yc + r*sin(omega*(t-tcircle));
    end
    
    % Use Elbow Down Solution and define the desired joint trajectories
    elbow = 1; % Elbow down Sol
    thd(i,:) = invK(RR,xe_d(i), ye_d(i),elbow);
    
    % Find the control manipulation for joint 1 and joint 2
    tau1 = theta1_Cntrl.PIDStep(th(1), thd(i,1));
    tau2 = theta2_Cntrl.PIDStep(th(2), thd(i,2));
    
    torque = [tau1;tau2]; % Apply torque to robot
    
    [~,state_new] = ode45(@(t,x)RR.model(t,x,torque),[t tNext],x);
    
    if(mod(i,50)==0), t,end; % display some times to show progress
    
    % save time, state, and robot joint torques
    torques(i,:)   = torque;
    time(i+1)      = tNext;
    state(i+1,:)   = state_new(end,1:4); %save the state at the end of the simulation time
    
end%while


%Create Plots
figure(1)
% Joint Angles
subplot(211)
plot(time,state(:,1),time(1:end-1),thd(:,1)); 
legend('\theta_l Traj.','\theta_1 Ref. Traj.','location','northwest')
xlabel('time, [s]');ylabel('\theta_1(t), [rad]'); title(['Damping = ' num2str(b1)])

subplot(212)
plot(time,state(:,2),time(1:end-1),thd(:,2)); 
legend('\theta_2 Traj.','\theta_2 Ref. Traj.','location','southeast')
xlabel('time, [s]');ylabel('\theta_2(t), [rad]')

% Torque Input
figure(2)
stairs(time(1:end-1),torques); legend('\tau_1','\tau_2')
xlabel('time, [s]');ylabel('Control Input, [N-m]');

%X-Y Trajectory
figure(3)
[xe,ye] = RR.fwdK(state(:,1:2));
j = round(tcircle/dt);
plot(xe_d,ye_d,'--',xe,ye,xe_d(j),ye_d(j),'g+',xe_d(end),ye_d(end),'+r',xe(j),ye(j),'g+',xe(end),ye(end),'+r')
axis([0 .5 0 .5])
axis square
xlabel('x-coordinate, [m]');ylabel('y-coordinate, [m]');

% Joint Angular Velocities
figure(4)
subplot(211)
plot(time,state(:,3)); 
xlabel('time, [s]');ylabel('\omega_1(t), [rad/s]');
subplot(212)
plot(time,state(:,4)); 
xlabel('time, [s]');ylabel('\omega_2(t), [rad/s]')


% Final Errors
%disp('Final Errors')
%disp(['theta_1 error = ' num2str(theta1_Cntrl.err,2) ', theta_2 error = ' num2str(theta2_Cntrl.err,2)])
