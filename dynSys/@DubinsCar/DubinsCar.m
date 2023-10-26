classdef DubinsCar < handle
    properties
        xdim    % state dimension
        udim    % control input dimension
        paras   % system-dependent parameters
        
        % control-affine dynamics: xdot = f(x) + g(x)u
        f       % Drift term as a function handle
        g       % vector field as a function handle
        fpoly   % polynomial dynamics

        % linearized dynamics: xdot = A(t)x + B(t)u
        A       % xdim-by-xdim state matrix as a function handle
        B       % xdim-by-udim input-to-state as a function handle
    end
    
    methods
        function [x,f,g] = defineSystem(obj,paras)
            syms p_x p_y theta
            x   = [p_x; p_y; theta];
            f   = [paras.v*sin(theta); paras.v*cos(theta); 0];
            g   = [0;0;1];
        end
    end
end