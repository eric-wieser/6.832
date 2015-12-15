plant = HolonomicDrive.plausibleModel();
sensors = HolonomicDriveWheelIntegrator(plant);

T = 2;

modes = {...
	[1, -1, -1, 1],...
	[1, 1, -1, -1],...
	[1, 1, 1, 1]...
};

amp = 0.05;

results = {};

for i=1:3
	mode = modes{i};

	umovex = amp * FunctionHandleTrajectory(...
		@(t) plant.umax.*mode'*sin(2*pi*t/T),...
		4, [0 T]);
	umovex = setOutputFrame(umovex, plant.getInputFrame);

	xtraj = simulate(cascade(umovex, plant), [0 T], zeros(6, 1));
	wheeltraj = simulate(cascade(xtraj, sensors), [0 T], zeros(4, 1));

	overall_traj = [xtraj; wheeltraj];
	results = {results{:}, overall_traj};
end
%%
ts = 0:1/60:2;
ts = ts(1:end - 1);


for i=1:3
	data = results{i}.eval(ts);
	fileID = fopen(sprintf('simple%d.bin', i),'w');
	fwrite(fileID,data,'double');
	fclose(fileID);
end

