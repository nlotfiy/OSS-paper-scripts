clc
close all
clear

% model parameters
J = 0.0026;
b = 0.01;
Kt = 0.66;
Kb = 0.66;
R = 2.62;
L = 0.05;

A = [0, 1, 0; 0, -(b/J), (Kt/J);0, -(Kb/L), -(R/L)];
B = [0;0;1/L];
C = eye(3);
D = 0;

ssModel = ss(A,B,C,D);

% simulation time specifications
dt = 0.001;
t0 = 0;
tf = 0.5;
tspan = t0:dt:tf;

% initial conditions
x0 = [0; 0; 0];
v0 = 24;

u = v0;

[x, t] = step(u*ssModel, tspan);

figure('name','System States')
set(gcf,'Units','inches')
set(gcf,'Position', [2 2 8 6])
subplot(3,1,1)
plot(t, x(:,1), 'linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
ylabel('$\theta(t)$ [rad]','Interpreter','latex','FontName','Arial','Fontsize',18) 
subplot(3,1,2)
plot(t, x(:,2), 'linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
ylabel('$\omega(t)$ [rad/s]','Interpreter','latex','FontName','Arial','Fontsize',18) 
subplot(3,1,3)
plot(t, x(:,3), 'linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
ylabel('$i(t)$ [A]','Interpreter','latex','FontName','Arial','Fontsize',18) 
xlabel('Time [s]','FontName','Arial','Fontsize',18) 