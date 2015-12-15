function runTrajComparison()
	megaclear;

	plant = HolonomicDrive.plausibleModel();
	n = 4;

	% find optimal trajectory
	[xtraj, utraj]   = findFastestTraj(plant, [0.5; 0.5; 0; 0; 0; 0], false);
	[nxtraj, nutraj] = findFastestTraj(plant, [0.5; 0.5; 0; 0; 0; 0], true);
	x0 = zeros(6, 1);
	u0 = zeros(n, 1);
	
	xtraj = simulate(cascade(utraj, plant), utraj.tspan, x0);
	nxtraj = simulate(cascade(nutraj, plant), nutraj.tspan, x0);

	tspan = [0 max(utraj.tspan(2),nutraj.tspan(2))];

	xtraj  = trim(append(xtraj,  xtraj.eval( xtraj.tspan(2))), tspan);
	nxtraj = trim(append(nxtraj, nxtraj.eval(nxtraj.tspan(2))), tspan);
	utraj  = trim(append(utraj,  u0), tspan);
	nutraj = trim(append(nutraj,  u0), tspan);

	doffset = 0.125*[1;-1;0;0;0;0];
	offset = 0.25*[-1;-1;0;0;0;0];
	xtraj = setOutputFrame(xtraj + offset + doffset, plant.getOutputFrame);
	nxtraj = setOutputFrame(nxtraj + offset - doffset, plant.getOutputFrame);
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
		
		mv.draw(t, x);
		xlim([-0.5 0.5]);
		ylim([-0.5 0.5]);
	end
	mwf = FunctionHandleVisualizer(mv.getInputFrame, @draw);

	% mwf.playbackAVI(xtrajs, 'comparison.avi');
	mwf.playback(xtrajs, struct('slider', true));

	figure;
	subplot(1, 2, 1);
	plot(ts, utraj.eval(ts));
	subplot(1, 2, 2);
	plot(ts, nutraj.eval(ts));
	
	combtraj = [appendWheelStates(plant, xtraj); appendWheelStates(plant, nxtraj)];
	combtraj = FunctionHandleTrajectory(@(t) eval(combtraj, t/10), combtraj.dim, combtraj.getBreaks * 10);
	saveTraj(combtraj, '../Presentation/6.832/time-comparison.trj')
end