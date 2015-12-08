function runTrajComparison()
	megaclear;

	plant = HolonomicDrive.plausibleModel();
	n = 4;

	% find optimal trajectory
	[xtraj, utraj]   = findFastestTraj(plant, [2.5; 5; 0; 0; 0; 0], false);
	[nxtraj, nutraj] = findFastestTraj(plant, [2.5; 5; 0; 0; 0; 0], true);

	tspan = [0 max(utraj.tspan(2),nutraj.tspan(2))];

	x0 = zeros(6, 1);
	u0 = zeros(n, 1);
	xtraj  = trim(trajcat(x0,  xtraj,  xtraj.eval( xtraj.tspan(2))), tspan);
	nxtraj = trim(trajcat(x0, nxtraj, nxtraj.eval(nxtraj.tspan(2))), tspan);
	utraj  = trim(trajcat(u0,  utraj,  u0), tspan);
	nutraj = trim(trajcat(u0, nutraj,  u0), tspan);
	
	xtrajs = [nxtraj; xtraj];


	% visualize the trajectory
	v1 = HolonomicDriveVisualizer(plant);
	v2 = HolonomicDriveVisualizer(plant);
	v1.fade_percent = 0.25;

	mv = MultiVisualizer({v1, v2});

	ts = tspan(1):0.03:tspan(2);
	function draw(t, x)
		xs = xtraj.eval(ts);
		plot(xs(1,:), xs(2,:), 'g');
		xs = nxtraj.eval(ts);
		plot(xs(1,:), xs(2,:), 'b');
		legend({'optimal', 'naive'});
		
		mv.draw(t, x)
	end
	mwf = FunctionHandleVisualizer(mv.getInputFrame, @draw);

	mwf.playback(xtrajs, struct('slider', true));

	figure;
	subplot(1, 2, 1);
	plot(ts, utraj.eval(ts));
	subplot(1, 2, 2);
	plot(ts, nutraj.eval(ts));
end