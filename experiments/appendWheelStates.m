function overall_traj = appendWheelStates(plant, xtraj)

sensors = HolonomicDriveWheelSensors(plant);

wheeltraj = simulate(cascade(xtraj, sensors), xtraj.tspan, zeros(length(plant.wheels), 1));

overall_traj = [xtraj; wheeltraj];