$(function() {

var container = $('#container');

var renderer = new THREE.WebGLRenderer({
	alpha: true,
	antialias: true,
	maxLights: 5,
	canvas: container.get(0)
});

var camera = new THREE.PerspectiveCamera(75, container.width() / container.height(), 0.01, 5);
$(window).resize(function() {
    var w = container.width();
    var h = container.height();

    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
});
$(window).resize();

controls = new THREE.OrbitControls( camera, renderer.domElement );
//controls.addEventListener( 'change', render ); // add this only if there is no animation loop (requestAnimationFrame)
controls.enableDamping = true;
controls.dampingFactor = 0.25;

scene = new Scene();

// Insert a cube
robot = new Robot();
scene.add(robot);

camera.position.z = 0.25;
camera.position.y = 0.125;
camera.lookAt(new THREE.Vector3(0,0,0));

var grid = new THREE.GridHelper( 50, 0.1 );
grid.material = new THREE.LineBasicMaterial({color: 0x004000});
scene.add(grid);

var plane = new THREE.Mesh(
	new THREE.PlaneGeometry(50, 50),
	new THREE.MeshLambertMaterial({color: 0x008000, side: THREE.DoubleSide})
);
plane.rotateOnAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);
plane.translateZ(0.001)
scene.add( plane );

var active_traj_id = 0;
var all_trajs = null;

var c = new THREE.Clock();
Q.all([
	Trajectory.load('simple1.bin',10),
	Trajectory.load('simple2.bin',10),
	Trajectory.load('simple3.bin',10)
]).then(function(trajs) {
	all_trajs = trajs;
	c.start();

});

var render = function () {
	requestAnimationFrame( render );

	// robot.rotation.x += 0.01;

	controls.update();

	if(all_trajs) {
		var traj = all_trajs[active_traj_id];
		var t = c.getElapsedTime();
		traj = traj.eval(t % 2);
		robot.setState(traj[0], traj[1], traj[2], traj.slice(6));

		if(t > 2) {
			active_traj_id = (active_traj_id + 1) % 3;
			c = new THREE.Clock(true);
		}
	}

	renderer.render(scene, camera);
};

render();

});