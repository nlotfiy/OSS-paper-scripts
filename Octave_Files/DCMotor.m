classdef DCMotor < handle
    properties
      
      %The physical parameters for our example are:
      J;     %moment of inertia of the rotor     0.01 kg.m^2
      b;     %motor viscous friction constant    0.1 N.m.s
      Kb;    %electromotive force constant       0.01 V/rad/sec
      Kt;    %motor torque constant              0.01 N.m/Amp
      R;     %electric resistance                1 Ohm
      L;     %electric inductance                0.5 H
     % V = 0;     % armature voltage
    endproperties
    
    methods
  
     function obj = DCMotor(J,b,Kb,L,R,Kt);
        if (nargin >0)
        obj.J = J; obj.b = b; obj.Kb = Kb;
        obj.L = L; obj.R = R; obj.Kt = Kt;
        end
     end%function
    
    function xdot = model(obj,t,x,u) 
            %// ODE: d angle / dt = omega; d omega/dt = (torque - b omega)/J
            %// States: 0 - angle; 1 - omega
            theta = x(1); omega = x(2); i = x(3); 
            v = u; % armature voltage
            
            xdot = [omega, (obj.Kt*i-obj.b*omega)/obj.J, ... 
                    (v-obj.Kb*omega-obj.R*i)/obj.L];
    end%function 
    function [xdot] = model2(obj,t,x,u)
     
     A = [0 1 0; 0 -obj.b/obj.J   obj.Kt/obj.J;0 -obj.Kb/obj.L   -obj.R/obj.L]; 
      B = [0;0;1/obj.L];
      C = [1 0 0;0 1 0;0 0 1]; D = 0;
      
      xdot = A*x+B*u;
    end%function
   
      function [A,B,C,D] = statespace(obj)
      A = [0 1 0; 0 -obj.b/obj.J   obj.kt/obj.J;0 -obj.kb/obj.L   -obj.R/obj.L]; 
      B = [0;0;1/obj.L];
      C = [1 0 0;0 1 0;0 0 1]; D = 0;
    end%function
    
    %}
    
    endmethods
        
end