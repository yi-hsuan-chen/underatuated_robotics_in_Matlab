classdef CartPole < nonlinearSys
    methods
        function [X,U,f] = defineSystem(obj,paras)
            % system parameters
            mc      = paras.mc;
            mp      = paras.mp;
            l       = paras.l;
            g       = paras.g;           
            
            syms x theta dx dtheta fx real
            cT      = cos(theta);
            sT      = sin(theta);
            X       = [x; theta; dx; dtheta];
            ddx     = (fx + mp*sT*(l*dtheta^2+g*cT))/(mc+mp*sT^2);
            ddtheta = (-fx*cT-mp*l*dtheta^2*cT*sT-(mc+mp)*g*sT)/(l*(mc+mp*sT^2));
            f       = [dx; dtheta; ddx; ddtheta];
            U       = fx;
        end
    end
    
end