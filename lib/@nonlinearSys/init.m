function init(obj, symbolic_x, symbolic_u, symbolic_f)
    if isempty(symbolic_x) || isempty(symbolic_f) || isempty(symbolic_u)
        error('x,f,u is empty. Create a class funtcion defineSystem and define your dynamics symbolically.')
    end
    
    if ~isa(symbolic_f, 'sym')
        f_ = sym(symbolic_f);
    else
        f_ = symbolic_f;
    end

    X           = symbolic_x;
    U           = symbolic_u;
    obj.xdim    = size(X,1);
    obj.udim    = size(U,1);
    obj.f       = matlabFunction(f_,'vars',{X,U});

    % Linearize system matrices
    dx          = obj.f(X,U);
    obj.A       = matlabFunction(jacobian(dx,X), 'vars', {X,U});
    obj.B       = matlabFunction(jacobian(dx,U), 'vars', {X,U});
end