classdef PIDController < handle
    properties
        kp, ki, kd;
        pTerm, iTerm, dTerm;
        iTerm0 = 0;
        uMin, uMax, uOffset;
        err, old_err = 0;     % current and previous error
        dt;                   % sample time
    end%properties
    
    methods
        
        function obj = PIDController(kp, ki, kd, dt, uMin, uMax, uOffset)
            
            obj.kp = kp; obj.ki = ki; obj.kd = kd; % Controller Gains
            obj.uMin = uMin; obj.uMax = uMax; % Manipulation Min and Max
            obj.uOffset = uOffset;
            obj.dt = dt;
       
        end%function
        
        function u = PIDStep(obj,procVar, setPoint)
            
            % Calculate error, error integral, and error derivative
            obj.err   = setPoint - procVar;
            Err       = obj.err * obj.dt;
            errDot = (obj.err - obj.old_err) / obj.dt;
            
            % Compute P, I and D contributions
            obj.pTerm = obj.kp * obj.err;
            obj.iTerm = obj.iTerm0 + obj.ki*Err;
            obj.dTerm = obj.kd * errDot;
            % Control Law
            u = obj.pTerm + obj.iTerm + obj.dTerm + obj.uOffset;
            
            % Saturate the controller output value if exceeds limits
            if((u > obj.uMax) || (u < obj.uMin))
                if (u > obj.uMax), u = obj.uMax;
                else, u = obj.uMin; end %(uTrial < obj.uMin)
            end
            
            % update the integrator initial value
            obj.iTerm0 = obj.iTerm; %obj.iTerm0 + iTermAdd;
            obj.old_err = obj.err;  % Update previous error value
            
        end%function
        
    end%methods
    
end%classdef