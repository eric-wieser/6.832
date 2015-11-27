classdef HolonomicDriveVisualizer < Visualizer	
	properties
		wheels
		outline
	end
	
	methods
		function obj = HolonomicDriveVisualizer(plant)
			typecheck(plant, 'HolonomicDrive');
			obj = obj@Visualizer(plant.getOutputFrame);
			obj.wheels = plant.wheels;
			
			vertices = [obj.wheels.pos];
			obj.outline = vertices(:, convhull(vertices(1,:), vertices(2,:)));
		end
		
		function draw(obj, ~, x)			
			axis 'equal';
			grid on;
			
			theta = x(3);
			rotation = [[cos(theta); sin(theta)] [-sin(theta); cos(theta)]];
			
			% draw the body
			body = gadd(x(1:2), rotation * obj.outline);
			patch(body(1,:), body(2,:), 'r');
			
			corners = [1 1 -1 -1; 1 -1 -1 1];
			corners = gmultiply(corners, [1; 0.1]);
			
			% draw the wheels
			for wheel = obj.wheels
				% corners of wheels in wheel space
				w_corners = [wheel.driveDir wheel.slipDir] * corners;
				
				% corners of wheels in body space
				b_corners = gadd(wheel.pos, w_corners);
				
				% corners of wheels in global space
				g_corners = gadd(x(1:2), rotation * b_corners);
				
				patch(g_corners(1,:), g_corners(2,:), [0.5 0.5 0.5]);
			end
		end
	end	
end

