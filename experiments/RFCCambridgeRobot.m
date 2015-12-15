function obj = RFCCambridgeRobot(thetas)
	% construct a HolonomicDrive object based on estimated parameters of the RFC robots

	if nargin == 0
		n = 4;
		thetas = 2*pi * (2*(1:n) -1)/(2*n);
	else
		n = length(thetas)
	end
	% make some random guesses
	wr = 0.025;
	m = 1;

	% taken from the C# code
		robotToWheelRadius = 0.0783; % Distance from robot center to wheel center
		wheelMSPerSpeed = 0.02952;   %Conversion from wheel speed [0,127] -> m/s
		% When changing from the current speed to a new commanded speed, every 1 / (this number of seconds),
		% the difference between the actual wheel speed and the commanded will be multiplied by 1/e
		changeConstlf = 8;

	% now try and decode these params
	%  tau = u - b theta\dot
	%  x\ddot = 1 / m * sum (\tau * dir / wr)
	%  theta\ddot = 1 / (wr^2 m) sum (\tau * dir)
	%             = 2 / (wr^2 m) * (umax - b theta\dot)
	%  changeConstlf = 1 / (2 / (wr^2 m) * b)
	% screw it. This is at least dimensionally correct:
	wb = changeConstlf * (wr^2 * m);

	speed_max = wheelMSPerSpeed * 127;
	omega_max = speed_max / wr;
	tau_max = wb * omega_max;

	%
	r = robotToWheelRadius;

	% also a guess
	I = 1/2 * m * r^2;

	for i = 1:n
		theta = thetas(i);
		wheels(i).pos = [r*cos(theta); r*sin(theta)];

		% first column - active direction
		% second column - passive direction
		wheels(i).driveDir = [-sin(theta); cos(theta)];
		wheels(i).slipDir = [cos(theta); sin(theta)];
		wheels(i).b = wb;
		wheels(i).r = wr;
	end

	obj = HolonomicDrive(wheels, I, m);
	obj = setInputLimits(obj, -tau_max, tau_max);
end