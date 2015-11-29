function runTrajComparison()
	plant = HolonomicDrive.defaultInstance(3);
	naive_plant = cascade(NaiveController(plant), plant);

	% find optimal trajectory
	x0 = zeros(6, 1);
	xf = [0; 10; 0; 0; 0; 0];
	function [h,dh] = finalCost(t,x)
		h = t;
		dh = [1,zeros(1,size(x,1))];
	end
	tf0 = 15;
	
	% now repeat for the naive controller
	xtrajs = {};
	utrajs = {};
	for p = {naive_plant, plant}
		p = p{1};
		
		N = 21;
		prog = DirtranTrajectoryOptimization(p,N,[tf0/2 tf0*2]);
		prog = prog.addInputConstraint(BoundingBoxConstraint(p.umin, p.umax), 1:N);
		prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
		prog = prog.addStateConstraint(ConstantConstraint(xf), N);
		prog = prog.addFinalCost(@finalCost);

		[xtraj,utraj,~,~,info] = prog.solveTraj(tf0);
		
		xtrajs = {xtrajs{:}, xtraj};
		utrajs = {utrajs{:}, utraj};
	end
	
	xtraj = vertcat(xtrajs{:});
	
	% visualize the trajectory
	v1 = HolonomicDriveVisualizer(plant);
	v2 = HolonomicDriveVisualizer(plant);
	v1.fade_percent = 0.25;
	
	mv = MultiVisualizer({v1, v2});
	function draw(t, x)
		xs = xtraj.eval(xtraj.tspan(1):0.1:t);
		plot(xs(1,:), xs(2,:));

		mv.draw(t, x)
	end
	mwf = FunctionHandleVisualizer(mv.getInputFrame, @draw);
	
	mwf.playback(xtraj, struct('slider', true));
	
	u = utrajs;
end