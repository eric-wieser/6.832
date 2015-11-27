classdef HolonomicDrive < SecondOrderSystem
	%HOLONOMICDRIVE Summary of this class goes here
	%   Detailed explanation goes here
	
	properties (SetAccess = private, GetAccess = public)
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
			
			generateGradients('sodynamics', 2, 'dynamicsGradients', obj, 0,...
				rand(3, 1), rand(3, 1), rand(n, 1))
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
		
		function [qdd, df, d2f] = sodynamics(obj,~,q,qd,u)
			if nargout > 1
				[df,d2f]= dynamicsGradients(obj,0,q,qd,u,nargout-1);
			end
		
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
			
			% convert force to world space
			total_force = rotation * total_force;
			
			qdd = [total_force / obj.I; total_moment / obj.m];
		end
		
		
		function [utraj,xtraj]=runDircol(p,x0,xf,tf0)
			N = 151;
			prog = DircolTrajectoryOptimization(p,N,[5 25]);
			prog = prog.addInputConstraint(BoundingBoxConstraint(p.umin, p.umax), 1:N);
			prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
			prog = prog.addStateConstraint(ConstantConstraint(xf), N);
			prog = prog.addRunningCost(@cost);
			prog = prog.addFinalCost(@finalCost);

			function [g,dg] = cost(dt,x,u)
				R = 0;
				g = u'*R*u;
				%g = sum((R*u).*u,1);
				%dg = [zeros(1,1+size(x,1)),2*u'*R];
				dg = zeros(1, 1 + size(x,1) + size(u,1));
			end
			
			function [h,dh] = finalCost(t,x)
				h = t;
				dh = [1,zeros(1,size(x,1))];
			end

			traj_init.x = PPTrajectory(foh([0,tf0],[x0,xf]));
			info = 0;
			while (info~=1)
				tic
				[xtraj,utraj,z,F,info] = prog.solveTraj(tf0);
				toc
			end
		end
	end
	
	methods(Static)
		function obj = defaultInstance(n)
			r = 1;
			if ~exist('n', 'var')
				n = 3;
			end
		
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
		
		function runTest()
			plant = HolonomicDrive.defaultInstance();
			v = HolonomicDriveVisualizer(plant);
			
			
			u0 = Point(plant.getInputFrame, [1; -1; 0]);
			x0 = Point(plant.getStateFrame, zeros(6, 1));
			
			sys = cascade(ConstantTrajectory(u0), plant);
			
			xtraj = simulate(sys, [0 10], x0);			
			
			v.playback(xtraj, struct('slider', true));
		end
		
		function trajectory()
			plant = HolonomicDrive.defaultInstance();
			
			x0 = zeros(6, 1);
			xf = [10; 10; pi; 0; 0; 0];
			
			[utraj, xtraj] = plant.runDircol(x0, xf, 15);
			
			t = utraj.tspan(1):0.1:utraj.tspan(2);
			figure
			subplot(1, 2, 1);
			plot(t, utraj.eval(t));
			subplot(1, 2, 2);
			xs = xtraj.eval(t);
			plot(xs(1,:), xs(1,:));
			quiver(xs(1,:), xs(1,:), cos(xs(3,:)), sin(xs(3,:)))
			
			v = HolonomicDriveVisualizer(plant);
			v.playback(xtraj, struct('slider', true));
		end
	end
end