classdef LineConstraint < LinearConstraint
  methods
    function obj = LineConstraint(dims, x0, dx)
      if nargin == 2
        dx = x0;
        x0 = dims;
        dims = 1:length(x0);
      end
      A = zeros(length(dims), length(x0));
      A(:,dims) = dx*dx' - eye(length(dims));
      zero_rows = all(A==0, 2);
      A(zero_rows,:) = [];
      obj = obj@LinearConstraint(A*x0, A*x0, A);
    end
  end
end
