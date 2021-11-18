clc
close all
clear

% simulation time specifications
dt = 0.01;
t_init = 0;
t_final = 10;
time = t_init:dt:t_final;
th1 = 4;
t_dwell = 1;
th2 = th1 + t_dwell;

% initial conditions for the joint angles and the end-effector position
th_initial = [pi/2;0];
thdot_initial = [0;0];
state0 = [th_initial;thdot_initial];
state = state0;
th = state(1:2,1);
thdot = state(3:4,1);

[xe0, ye0] = FK_fun(th(1,1),th(2,1));
xe_act = xe0;
ye_act = ye0;

% the initial desired trajectory and the desired joint angles
xo = 0.1;
yo = 0.2;
R = 0.1;
T = 4;
[xc0, yc0] = circ_traj(xo, yo, R, T, 0);

xe_d = xe0;
ye_d = ye0;
[th1d, th2d] = IK_fun(xe_d(1),ye_d(1));
thd = [th1d; th2d];


% controller parameters and initial control input
Kp = [20 0;0 20];
Ki = [40 0;0 40];
Kd = [2 0;0 0.1];

old_e = thd(:,1) - th(:,1);
E = 0;
tau = zeros(2,1);
tau_max = 10;
tau_min = -10;



for i = 1:(length(time)-1)
    
   % calculating the new system states by integrating the system dynamics
   t = time(i);
   tNext = t + dt;
   t_span = t:dt:tNext;
   state0 = state(:,i);
   [t_int, state_new] = ode45(@(t,x) model(t, x, tau(:,i)), t_span, state0);
   state(:,i+1) = state_new(end,:)';
   th(:,i+1) = state(1:2,i+1);
   thdot(:,i+1) = state(3:4,i+1);
   
   [xe_act(i+1), ye_act(i+1)] = FK_fun(th(1,i+1),th(2,i+1));
   
   % calculating the desired joint angles based on the desired trajectory
   if time(i) < th1
        xe_d(i+1) = (time(i)/th1)*(xc0-xe0)+xe0;
        ye_d(i+1) = (time(i)/th1)*(yc0-ye0)+ye0;
   elseif time(i) >= th1 && time(i) < th2
        xe_d(i+1) = xc0;
        ye_d(i+1) = yc0;
   else
        [xe_d(i+1), ye_d(i+1)] = circ_traj(xo, yo, R, T, time(i)-th2);       
   end
   
   [th1d, th2d] = IK_fun(xe_d(i+1),ye_d(i+1));
   thd(:,i+1) = [th1d;th2d];
   
   %  calculating the control input based on the tracking error
   e = thd(:,i+1) - th(:,i+1);
   E = E+ e*dt;
   edot = (e-old_e)/dt;
   tau_new = Kp*e + Kd*edot + Ki*E;
   tau_new(tau_new>tau_max) = tau_max;
   tau_new(tau_new<tau_min) = tau_min;
   
   tau(:,i+1) = tau_new;
   old_e = e;
   
end


figure('name','Joint Angles')
set(gcf,'Units','inches')
set(gcf,'Position', [2 2 8 6])
subplot(2,1,1)
plot(time, thd(1,:),'r:',time, th(1,:),'b-','linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
h_legend = legend('Desired','Actual');
set(h_legend,'FontName','Arial','Fontsize',18,'color','w', 'location','best','orientation','vertical')
ylabel('$\theta_1(t)$ [rad]','Interpreter','latex','FontName','Arial','Fontsize',18) 
subplot(2,1,2)
plot(time, thd(2,:),'r:',time, th(2,:),'b-','linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
h_legend = legend('Desired','Actual');
set(h_legend,'FontName','Arial','Fontsize',18,'color','w', 'location','best','orientation','vertical')
ylabel('$\theta_2(t)$ [rad]','Interpreter','latex','FontName','Arial','Fontsize',18) 
xlabel('Time [s]','FontName','Arial','Fontsize',18) 

figure('name','Control Input')
set(gcf,'Units','inches')
set(gcf,'Position', [2 2 8 6])
plot(time, tau(1,:),'b:',time, tau(2,:),'r--','linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
h_legend = legend('u_1','u_2');
set(h_legend,'FontName','Arial','Fontsize',18,'color','w', 'location','best','orientation','vertical')
ylabel('Control Input [N.m]','FontName','Arial','Fontsize',18) 
xlabel('Time [s]','FontName','Arial','Fontsize',18) 

figure('name','End-effector Trajectory')
set(gcf,'Units','inches')
set(gcf,'Position', [2 2 8 6])
plot(xe_d, ye_d,'r--',xe_act, ye_act,'b-','linewidth', 2)
set(gca,'FontName','Arial','Fontsize',18,'Fontweight','Bold')
grid on; 
h_legend = legend('Desired','Actual');
set(h_legend,'FontName','Arial','Fontsize',18,'color','w', 'location','best','orientation','vertical')
ylabel('y','FontName','Arial','Fontsize',18) 
xlabel('x','FontName','Arial','Fontsize',18) 


function dxdt = model(t, x, u)
    % model parameters
    L1 = 0.25;
    L2 = 0.25;
    r1 = L1/2;
    r2 = L2/2;
    m1 = 0.5;
    m2 = 0.5;
    g = 9.81;
    I1 = m1*L1^2/12;
    I2 = m1*L1^2/12;
    b1 = 1e-1;
    b2 = 1e-1;
    
    th = [x(1);x(2)];
    thdot = [x(3);x(4)];
    
    thddot = M_mat(th)\(u - C_mat(th, thdot)*thdot - g_mat(th));
    
    dxdt = [x(3); x(4); thddot(1); thddot(2)];
    
    function M = M_mat(th)
        m11 = I1 + I2 + m1*r1^2 + m2*(L1^2+r2^2)+2*m2*L1*r2*cos(th(2));
        m12 = I2 + m2*r2^2 + m2*L1*r2*cos(th(2));
        m21 = I2 + m2*r2^2 + m2*L1*r2*cos(th(2));
        m22 = I2 + m2*r2^2;
        M = [m11 m12;m21 m22];
    end

    function C = C_mat(th, thdot)
        c11 = -m2*L1*r2*sin(th(2))*thdot(2)+b1;
        c12 = -m2*L1*r2*sin(th(2))*(thdot(1)+thdot(2));
        c21 = m2*L1*r2*sin(th(2))*thdot(1);
        c22 = b2;
        C = [c11 c12;c21 c22];
    end

    function G = g_mat(th)
        g1 = (m1*r1+m2*L1)*g*cos(th(1))+m2*r2*g*cos(th(1)+th(2));
        g2 = m2*r2*g*cos(th(1)+th(2));
        G = [g1;g2];        
    end
end

function [th1, th2] = IK_fun(xe, ye)
    
    L1 = 0.25;
    L2 = 0.25;
    elbow = +1;
%     elbow = -1;
    theta = atan2(ye, xe);
    if theta < 0
        theta = theta + 2*pi;
    end
    D = (xe^2+ye^2-L1^2-L2^2)/(2*L1*L2);
    th2 = atan2(elbow*sqrt(1-D^2),D);
    th1 = theta-elbow*atan2(L2*sin(th2),L1+L2*cos(th2));
end

function [xe, ye] = FK_fun(th1, th2)
    
    L1 = 0.25;
    L2 = 0.25;

    xe = L1*cos(th1) + L2*cos(th1+th2);
    ye = L1*sin(th1) + L2*sin(th1+th2);
end

function [x,y] = circ_traj(xo, yo, R, T, time)
    
    x = xo + R*cos((2*pi/T)*time);
    y = yo + R*sin((2*pi/T)*time);

end