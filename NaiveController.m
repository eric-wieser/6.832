classdef NaiveController < DrakeSystem
	properties
		plant
	end
	methods
		function obj = NaiveController(plant)
			obj = obj@DrakeSystem(0,0,3,plant.getNumInputs,true,true);

			obj = setInputFrame(obj,CoordinateFrame('NaiveInput',3,'u', {'v1', 'v2', 'omega'}));
			obj = setOutputFrame(obj,plant.getInputFrame);

			obj.plant = plant;
		end
		
		function xd = dynamics(obj, ~, ~, ~)
			xd = [];
		end

		function y = output(obj, ~, ~, u)
			y = obj.plant.rotorSpeeds(u(1:2), u(3));
		end
	end
end