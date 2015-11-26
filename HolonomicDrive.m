classdef HolonomicDrive < SecondOrderSystem
	%HOLONOMICDRIVE Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		m
		I
		wheels
	end
	
	methods
		function obj = HolonomicDrive(wheels, I, m)
			% wheels: struct of
			%   pos:    offset from center of mass
			%   b:      damping factor of motor
			%   r:      wheel radius
			%   driveDir: drive direction, a unit vector
			%   slipDir: slip direction. For mecanum wheels, this is not
			%            orthogonal to the drive direction
			% I: moment of inertia of entire robot about vertical axis
			% m: total mass of robot
			n = length(wheels);

			obj = obj@SecondOrderSystem(3, n, true);
			
			obj.I = I;
			obj.m = m;
			obj.wheels = wheels;
			
			obj = setInputLimits(obj,-1,1);
			
			obj = setInputFrame(obj,CoordinateFrame('HolonomicInput',n,'u',...
				arrayfun(@(i) sprintf('tau%d', i), 1:n, 'Unif', false)));
			obj = setStateFrame(obj,CoordinateFrame('HolonomicState',6,'x',...
				{'1','2','theta', '1_dot','2_dot', 'theta_dot'}));
			obj = setOutputFrame(obj,obj.getStateFrame);
		end
		
		function speeds = rotorSpeeds(obj, vel, omega)
			% get the wheel speeds needed to give the robot a certain
			% velocity in body space
			i = 1;
			for wheel = obj.wheels
				% r cross omega
				rotv = omega * [-wheel.pos(2); wheel.pos(1)];
				totalv = vel + rotv;
				
				% decompose into powered and unpowered direction
				vwheel = [wheel.driveDir wheel.slipDir] \ totalv;

				speeds(i) = vwheel(1) / wheel.r;
				i = i + 1;
			end
		end
		
		function qdd = sodynamics(obj,~,q,qd,u)
			theta = q(3);
			rotation = [[cos(theta); sin(theta)] [-sin(theta); cos(theta)]];
			
			bodyvel = rotation \ qd(1:2);
			
			speeds = obj.rotorSpeeds(bodyvel, qd(3));
			
			n = length(obj.wheels);
			
			total_force = [0; 0];
			total_moment = 0;
			for i = 1:n
				wheel = obj.wheels(i);
				% simplified motor dynamics
				tau = u(i) - wheel.b*speeds(i);
				
				% assign direction to force
				f = tau / wheel.r * wheel.driveDir;
				
				% combine forces
				total_force = total_force + f;
				total_moment = total_moment...
					         + wheel.pos(1) * f(2)...
					         - wheel.pos(2) * f(1);
			end
			
			qdd = [total_force / obj.I; total_moment / obj.m];
		end
	end
	
	methods(Static)
		function obj = defaultInstance()
			r = 1;
			n = 3;
		
			wheels = [];
			for i = 1:n
				theta = 2*pi * (i/n);
				wheels(i).pos = [r*cos(theta); r*sin(theta)];
			
				% first column - active direction
				% second column - passive direction
				wheels(i).driveDir = [-sin(theta); cos(theta)];
				wheels(i).slipDir = [cos(theta); sin(theta)];
				wheels(i).b = 1;
				wheels(i).r = 1;
			end
			
			obj = HolonomicDrive(wheels, 1, 1);
		end
	end
end