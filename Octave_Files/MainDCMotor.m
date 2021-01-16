% Description : Simulates a PM DC Motor with a constant armature voltage
% Author: Luis A. Rodriguez
% Email: rodriguez@msoe.edu
% Date: 1/15/2021

clear all; clc
J = 0.0026;   %kg.m^2; moment of inertia of the rotor     
b = 0.01;     %N.m.s motor viscous friction constant    
kb = 0.66;    %V/rad/sec;   electromotive force constant       
kt = 0.66;    % N.m/Amp;   motor torque constant              
R  = 2.62;    %Ohm  electric resistance                
L = 0.05;     %H   electric inductance   
Va = 24;      % Armature voltage             

motor = DCMotor(J,b,kb,L,R,kt);
t0 = 0; tf = 0.5; dt = 0.01;
time = 0:dt:tf;
x0 = [0 0 0]; 

%Refine not yet implemented
options = odeset('AbsTol',1e-6,'Reltol',1e-3,'Refine',4);
      
v = Va;
[~,X] = ode45(@(t,x)motor.model(t,x,v),time,x0,options);

figure(1)
plot(time,X(:,1),time,X(:,2),time,X(:,3)); legend('angle', 'ang. speed','current')

figure(2)
plot(time,X(:,2)*60/2/pi); legend('ang. speed')
