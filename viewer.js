$(function() {

var container = $('#container');

var mgr = new Manager(container);



if(location.search == '?simple') {
	mgr.scene = new Scene();

	robot = new Robot();
	mgr.scene.add(robot);

	var active_traj = null;
	var next_traj_id = 0;
	var all_trajs = null;
	var c = new THREE.Clock();

	Q.all([
		Trajectory.load('simple2.bin',10),
		Trajectory.load('simple1.bin',10),
		Trajectory.load('simple3.bin',10)
	]).then(function(trajs) {
		all_trajs = trajs;
		c.start();
	});

	mgr.addEventListener('update', function() {
		if(all_trajs) {
			var t = c.getElapsedTime();
			if(active_traj == null) {
				active_traj = all_trajs[next_traj_id - 1];
				c.elapsedTime = 0;
			}
			else if(Math.floor(t / active_traj.length) != 0) {
				c.elapsedTime = c.elapsedTime % active_traj.length;
				active_traj = all_trajs[next_traj_id - 1];
			}

			if(active_traj) {
				var x = active_traj.eval(t % active_traj.length);
				robot.setState(x[0], x[1], x[2], x.slice(6));
			}
		}
	});

	if(window.top != window) {
		window.addEventListener("message", function(e) {
			var d = e.data;
			if(d.type == 'step') {
				next_traj_id = d.step;
			}
		}, false);
	}
}

else if(location.search == '?time') {
	mgr.scene = new Scene(20);

	mgr.camera.position.y = 0.60;
	mgr.camera.position.x = 0.25;
	mgr.camera.position.z = 0.25;

	var robot1 = new Robot(0xff0000);
	var robot2 = new Robot(0x0000ff);

	var current_step;

	var clock = new THREE.Clock();
	Trajectory.load('time-comparison.trj',20).then(function(traj) {
		mgr.scene.add(robot1);
		mgr.scene.add(robot2);

		mgr.addEventListener('update', function() {
			var state = traj.eval(clock.getElapsedTime() % (traj.length * 1.1));
			var x1 = state.slice(0, 10);
			var x2 = state.slice(10, 20);

			robot1.visible = current_step > 2;
			robot2.visible = current_step > 0;

			robot1.setState(x1[0], x1[1], x1[2], x1.slice(6));
			robot2.setState(x2[0], x2[1], x2[2], x2.slice(6));
		});
	})

	if(window.top != window) {
		window.addEventListener("message", function(e) {
			var d = e.data;
			if(d.type == 'step') {
				clock.start();
				current_step = d.step;
			}
		}, false);
	}
}

});