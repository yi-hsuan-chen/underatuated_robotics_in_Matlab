% Adopted from Jason Choi https://github.com/HybridRobotics/CBF-CLF-Helper
% Edited by Yi-Hsuan Chen (yhchen91@umd.edu) 09/14/2023

classdef nonlinearSys < handle
    properties
        xdim    % state dimension
        udim    % control input dimension
        paras   % system-dependent parameters
        xeq     % equailibrium
        
        f       % system dynamics: xdot = f(x,u)
        
        % linearized dynamics: xdot = A(t)x + B(t)u
        A       % xdim-by-xdim state matrix as a function handle
        B       % xdim-by-udim input-to-state as a function handle
    end

    methods
        function obj = nonlinearSys(paras)
            if nargin < 1
                obj.paras = [];
            else
                obj.paras = paras;
            end
            [x,u,f] = defineSystem(obj,paras);
            obj.init(x,u,f);
        end

        function [x,u,f] = defineSystem(obj,paras)
            % nonlinear sys: xdot = f(x)
            % Outputs:  x -- state vector
            %           u -- input vector
            %           f -- state vector derivative, expresed symbolically wrt x and u  
            x = []; u = []; f = []; 
        end

        function dx = dynamics(obj,t,x,u)
            % Inputs:   t -- time, x -- state, u -- control
            % Outputs:  dx -- time derivative of state as a function handle
            dx  = obj.f(x,u);
        end

        function dx = polynomialdyn(obj,t,x,u)
            % Inputs:   t -- time, x -- state, u -- control
            % Outputs:  dx_poly -- polynomial dynamics as a function handle
            dx  = obj.fpoly(x,u);
        end

        function [A,B] = linearize(obj,x0,u0)
            % Inputs:   x0 -- nominal state traj., u -- nominal control traj.
            % Outputs:  [A,B] -- system matrices as function handles
            A  = obj.A(x0,u0);
            B  = obj.B(x0,u0);
        end

    end
end