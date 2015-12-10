$(function() {

var container = $('#container');

var renderer = new THREE.WebGLRenderer({
	alpha: true,
	antialias: true,
	maxLights: 5
});
container.append(renderer.domElement);

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
camera.position.x = 0.0625;
camera.lookAt(new THREE.Vector3(0,0,0));

// var grid = new THREE.GridHelper( 50, 0.1 );
// grid.material = new THREE.LineBasicMaterial({color: 0x004000});
// scene.add(grid);

var plane = (function() {
	var ms = [
		new THREE.MeshBasicMaterial({color: 0x008000, side: THREE.DoubleSide, transparent: 1, opacity: 0.25}),
		new THREE.MeshBasicMaterial({color: 0x102000, side: THREE.DoubleSide, transparent: 1, opacity: 0.25})
	];
	n = 8;
	var g = new THREE.PlaneGeometry(n*0.05, n*0.05, n, n);
	for(var i = 0; i < g.faces.length; i++) {
		si = Math.floor(i / 2);
		g.faces[i].materialIndex = ((si % n)+ Math.floor(si / n)) % 2;
	}
	return new THREE.Mesh(g, new THREE.MeshFaceMaterial(ms));
})();

plane.rotateOnAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);
plane.translateZ(0.001)
scene.add( plane );

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

var render = function () {
	requestAnimationFrame( render );

	// robot.rotation.x += 0.01;

	controls.update();

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

	renderer.render(scene, camera);
};

render();

if(window.top != window) {
	window.addEventListener("message", function(e) {
		var d = e.data;
		if(d.type == 'step') {
			next_traj_id = d.step;
		}
	}, false);
}

});