classdef TwoLink < handle
    
    properties
        % Physical Parameters (in SI units)
        L1, r1;    % meters
        L2, r2;    % meters
        m1,  m2;       % kg
        b1, b2;  % Nm-s/rad, Damping on joint 1 and joint 2
        I1, I2; % Mass moment of inertia of link 1 and 2
        alph, bet, delt; % constants
        g = 9.81;       % m/s^2, Gravity
        
    end%properties
    
    methods
        function obj = TwoLink(L1,r1,L2,r2,m1,m2,g,b1,b2)
            if nargin>0
                obj.L1 = L1; obj.r1 = r1; obj.L2 = L2; obj.r2=r2;
                obj.m1 = m1; obj.m2 = m2; obj.g = g; obj.b1 = b1;obj.b2 = b2;
                
                % Calculate moment of inertia for long, slender rod
                obj.I1 = obj.m1*obj.L1^2/12; obj.I2 = obj.m2*obj.L2^2/12;
                obj.alph = obj.I1 + obj.I2 + obj.m1*obj.r1^2 + obj.m2*(obj.L1^2 + obj.r2^2);
                obj.bet = obj.m2*obj.L1*obj.r2;
                obj.delt = obj.I2 + obj.m2*obj.r2^2;
                
            end
        end
        
        
        function [xe,ye] = fwdK(obj,th)
          th1 = th(:,1);th2 = th(:,2);
  
            xe = obj.L1*cos(th1)+obj.L2*cos(th1+th2);
            ye = obj.L1*sin(th1)+obj.L2*sin(th1+th2);
            
        end
        function [th]= invK(obj,xe, ye, elbow)
            
            %elbow = 1; elbow down
            %elbow = -1; elbow up;
            phi = atan2(ye,xe);
            if phi < -pi/2, phi = phi + 2*pi; end
            
            D = (xe^2+ye^2-obj.L1^2-obj.L2^2)/(2*obj.L1*obj.L2);
            th2 = elbow*atan2(sqrt(1-D^2),D);
            th1 = phi-atan2(obj.L2*sin(th2),obj.L1+obj.L2*cos(th2));
            th = [th1 th2];
            
        end
        
        function G = g_mat(obj,th)
            th1 = th(1);th2 = th(2);
            % Gravity terms
            g1 = (obj.m1*obj.r1 + obj.m2*obj.L1)*obj.g*cos(th1) + obj.m2*obj.r2*obj.g*cos(th1 + th2);
            g2 = obj.m2 * obj.r2 * obj.g * cos(th1 + th2);
            G = [g1; g2];
        end%function
        function M = M_mat(obj,th)
            % Calculate the M matrix
            c2 = cos(th(2));       s2 = sin(th(2));
            m11 = obj.alph + 2*obj.bet*c2;  m12 = obj.delt + obj.bet*c2;
            m21 = m12;                      m22 = obj.delt;
            M = [m11 m12; m21 m22];
        end
        function C = C_mat(obj,th,thdot)
            % Calculate the Christoffel matrix
            k = obj.bet*sin(th(2));
            C11 = -k*thdot(2)  + obj.b1;  C12 = -k*(thdot(1) + thdot(2));
            C21 =  k*thdot(1);            C22 =  obj.b2;
            
            C = [C11 C12;C21 C22];
        end
        
        function xdot = model(obj,t,x,u)
           % Alias
            th = x(1:2); thdot = x(3:4); torque = u;

            M = M_mat(obj,th);
            C = C_mat(obj,th,thdot);
            % Gravity terms
            G = g_mat(obj,th);
            
            % Calculate the generalized accelerations
            thddot = inv(M) * (torque - C * thdot - G);
            % Generate the change in the state vector
            xdot = [thdot;thddot];
            
        end%function
        
        function obj = setTorque(obj,torque)
            obj.torque = torque;
        end
    end%methods
    
end%classdef