classdef LineConstraint < LinearConstraint
  % Line Constraint. x = x0 + u*dx
  methods
    function obj = LineConstraint(x0, dx)
      A = null(dx')';
      obj = obj@LinearConstraint(A*x0, A*x0, A);
    end
  end
end
