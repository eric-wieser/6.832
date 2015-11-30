function runTrajComparison()
	megaclear;

	plant = HolonomicDrive.plausibleModel();

	% find optimal trajectory
	x0 = zeros(6, 1);
	xf = [10; 10; 0; 0; 0; 0];
	function [h,dh] = finalCost(t,x)
		h = t;
		dh = [1,zeros(1,size(x,1))];
	end
	tf0 = 15;

	% now repeat for the naive controller
	xtrajs = {};
	utrajs = {};
	for i = 1:2
		p = plant;

		N = 31;
		prog = DirtranTrajectoryOptimization(p,N,[tf0/2 tf0*2]);
		prog = prog.addInputConstraint(BoundingBoxConstraint(p.umin, p.umax), 1:N);
		prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
		prog = prog.addStateConstraint(ConstantConstraint(xf), N);
		if (i == 1)
			dir = xf(1:3) - x0(1:3);
			dir = dir / norm(dir);
			lineM = dir*dir' - eye(3);

			% constrain x(1:3) to be simply interpolation
			A = [lineM zeros(3)];
			prog = prog.addStateConstraint(LinearConstraint(A*x0, A*x0, A), 2:N-1);
		end
		prog = prog.addFinalCost(@finalCost);

		[xtraj,utraj,~,~,info] = prog.solveTraj(tf0);
		disp(info);
		xtrajs = {xtrajs{:}, xtraj};
		utrajs = {utrajs{:}, utraj};
	end
	tspan = [0 max(utrajs{1}.tspan(2), utrajs{2}.tspan(2))];

	for i=1:2
		xtrajs{i} = trim(trajcat(x0, xtrajs{i}, xf), tspan);
		u0 = zeros(n, 1);
		utrajs{i} = trim(trajcat(u0, utrajs{i}, u0), tspan);
	end
	xtraj = vertcat(xtrajs{:});


	% visualize the trajectory
	v1 = HolonomicDriveVisualizer(plant);
	v2 = HolonomicDriveVisualizer(plant);
	v1.fade_percent = 0.25;

	mv = MultiVisualizer({v1, v2});

	ts = tspan(1):0.1:tspan(2);
	function draw(t, x)
		for xtraj = xtrajs
			xtraj = xtraj{1};
			xs = xtraj.eval(ts);
			plot(xs(1,:), xs(2,:));
		end

		mv.draw(t, x)
	end
	mwf = FunctionHandleVisualizer(mv.getInputFrame, @draw);

	mwf.playback(xtraj, struct('slider', true));

	figure;
	subplot(1, 2, 1);
	plot(ts, utrajs{1}.eval(ts));
	subplot(1, 2, 2);
	plot(ts, utrajs{2}.eval(ts));
end