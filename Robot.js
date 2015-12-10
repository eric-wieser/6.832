var Robot = function() {
	THREE.Object3D.call(this);

	var r = 0.078;
	var wr = 0.025;
	var wn = 4;
	var wwr = 0.005;
	var wwn = 8;

	this.base = new THREE.Mesh(
		new THREE.CylinderGeometry(r - wwr, r - wwr, 0.005, 32),
		new THREE.MeshPhongMaterial({color: 0xff0000})
	);
	this.base.position.y += wr;
	this.add(this.base);

	this.wheels = [];

	for(var i = 0; i < wn; i++) {
		var theta = (Math.PI*2) * (i + 0.5)/wn;

		var wheelRef = new THREE.Object3D();
		wheelRef.rotateOnAxis(new THREE.Vector3(0, 1, 0), theta);
		wheelRef.translateOnAxis(new THREE.Vector3(1, 0, 0), r);
		this.add(wheelRef);

		var wheel = new THREE.Object3D();
		wheelRef.add(wheel);
		this.wheels.push(wheel);

		var hub = new THREE.Mesh(
			new THREE.CylinderGeometry(wr - wwr, wr - wwr, 0.005, wwn*3),
			new THREE.MeshPhongMaterial({color: 0x808080})
		);
		hub.rotateOnAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
		wheel.add(hub);
		wheel.position.y += wr;

		for(var j = 0; j < wwn; j++) {
			var phi = (Math.PI*2) * (j + 0.5)/wwn;

			var subwheel = new THREE.Mesh(
				new THREE.CylinderGeometry(wwr, wwr, 0.005, 32),
				new THREE.MeshPhongMaterial({color: 0xc0c0c0})
			);
			subwheel.rotateOnAxis(new THREE.Vector3(0, 1, 0), Math.PI / 2);
			subwheel.rotateOnAxis(new THREE.Vector3(0, 0, 1), phi);
			subwheel.translateX(wr - wwr);
			wheel.add(subwheel);
		}


	}
	this.add(new THREE.AxisHelper(0.1));
}

Robot.prototype = Object.create(THREE.Object3D.prototype);
Robot.prototype.constructor = Robot;

Robot.prototype.setState = function(x, y, theta, wheelAngles) {
	this.position.x = x;
	this.position.z = -y;
	this.rotation.y = theta;

	wheelAngles.forEach(function(phi, i) {
		this.wheels[i].rotation.x = -phi;
	}, this);
}