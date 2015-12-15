function [ xtraj, utraj ] = findFastestTraj( p, xf, naive)
N = 31;
x0 = zeros(6, 1);

tf0 = norm(xf(1:2)) / 4;
	
prog = DircolTrajectoryOptimization(p,N,[tf0 / 2, tf0 * 2]);
prog = prog.addInputConstraint(BoundingBoxConstraint(p.umin, p.umax), 1:N);
prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
prog = prog.addStateConstraint(ConstantConstraint(xf), N);

if naive
	% constrain x(1:3) to be simply interpolation
	dir = xf(1:3) - x0(1:3);
	dir = dir / norm(dir);
	prog = prog.addStateConstraint(LineConstraint(x0, dir), 2:N-1, 1:3);
end

function [h,dh] = finalCost(t,x)
	h = t;
	dh = [1,zeros(1,size(x,1))];
end
prog = prog.addFinalCost(@finalCost);

[xtraj,utraj,~,~,info,infeasible_constraint_name] = prog.solveTraj(tf0);
end

