function [varargout] = runTrajDistComparison()
	varargout = runTrajDistComparisonLocus();
end

function [optimals, naives] = runTrajDistComparisonLocus()
	p = HolonomicDrive.plausibleModel(2*pi*[1, 2, 4, 5] / 6);
	
% 	runTrajDistComparisonSingle(p, 0.5, [1;0.1]);
% 	return
	
	reflect2 = @(sig)          [sig [1 0; 0 -1] * fliplr(sig)];
	reflect4 = @(sig) reflect2([sig [-1 0; 0 1] * fliplr(sig)]);
	reflect8 = @(sig) reflect4([sig [ 0 1; 1 0] * fliplr(sig)]);
	
	figure(11);
	hold on;
	axis 'equal';
	legend({'optimal', 'naive'});
	
	optimals = {};
	naives = {};
	tfs = 0.2 * (1:5);
	for tf = tfs

		optimal = [];
		naive = [];
		optimalok = [];
		naiveok = [];

		for theta = linspace(0, pi/2, 10)
			dir = [cos(theta); sin(theta)];
			[ x,  u, ok] = findFurthestTraj(p, tf, dir, false);
			[nx, nu, nok] = findFurthestTraj(p, tf, dir, true);

			optimal = [optimal x.eval(tf)];
			naive   = [naive  nx.eval(tf)];
			optimalok = [optimalok ok];
			naiveok   = [naiveok  nok];
		end



		optimal = optimal(1:2,logical(optimalok));
		naive = naive(1:2,logical(naiveok));
		optimal = reflect4(optimal);
		naive = reflect4(naive);
		
		figure(11);
		plot(optimal(1,:), optimal(2,:), 'g');
		plot(naive(1,:), naive(2,:), 'r');
		
		optimals = {optimals{:} optimal};
		naives = {naives{:} naive};
	end
	
	legend({'optimal', 'naive'});
	xlabel('x / m');
	ylabel('y / m');
end

function runTrajDistComparisonSingle(p, tf, dir)

	[ x,  u] = findFurthestTraj(p, tf, dir, false);
	[nx, nu] = findFurthestTraj(p, tf, dir, true);


	% visualize the trajectory
	v1 = HolonomicDriveVisualizer(p);
	v2 = HolonomicDriveVisualizer(p);
	v1.fade_percent = 0.25;

	mv = MultiVisualizer({v1, v2});

	ts = 0:0.03:tf;
	function draw(t, xcomb)
		xs = x.eval(ts);
		plot(xs(1,:), xs(2,:), 'g');
		xs = nx.eval(ts);
		plot(xs(1,:), xs(2,:), 'b');
		legend({'optimal', 'naive'});

		mv.draw(t, xcomb);
	end
	mwf = FunctionHandleVisualizer(mv.getInputFrame, @draw);

	mwf.playback([nx;x], struct('slider', true));

end

function [ xtraj, utraj, ok ] = findFurthestTraj(p, t, dir, naive)
	N = 31;
	x0 = zeros(6, 1);

	% a matrix such that lineM*x==lineM*x0 implies x is on the line x0 + dir*t
	dir = dir / norm(dir);
	lineM = dir*dir' - eye(2);

	prog = DircolTrajectoryOptimization(p,N,[t, t]);
	prog = prog.addInputConstraint(...
		setName(BoundingBoxConstraint(p.umin, p.umax), 'input_limits'), 1:N);
	prog = prog.addStateConstraint(...
		setName(ConstantConstraint(x0), 'start'), 1);

	on_line = LineConstraint(x0, dir);

	% constrain the final position to lie in $dir from the origin
	prog = prog.addStateConstraint(setName(on_line, 'end_direction'), N, 1:2);

	% constrain all other params to be zero
	A = zeros(4, length(x0));
	A(:,end-3:end) = eye(4);
	prog = prog.addStateConstraint(...
		setName(LinearConstraint(A*x0, A*x0, A), 'constants'), N);

	if naive
		% constrain x(1:2) to be simply interpolation
		prog = prog.addStateConstraint(...
			setName(on_line, 'intermediate_direction'), 2:N-1, 1:2);

		% constrain x(3) to be constant
		A = zeros(1, length(x0));
		A(:,3) = 1;
		prog = prog.addStateConstraint(...
			setName(LinearConstraint(A*x0, A*x0, A), 'constant_orientation'), 2:N-1);
	end

	function [h,dh] = finalCost(t,x)
		h = -sum(dir.*x(1:2));
		dh = [1, zeros(1,size(x,1))];
		dh(2:3) = -dir;
	end
	prog = prog.addFinalCost(@finalCost);
	trajinit = foh([0 t], [x0 x0+[dir;zeros(4,1)]]);
	ok = true;
	[xtraj,utraj,~,~,info,infeasible_constraint_name] = prog.solveTraj(t, trajinit);
	if info ~= 1
		disp(sprintf('Failed to find trajectory - error %d', info))
		ok = false;
	end
end

