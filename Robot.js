var Robot = (function() {
	var r = 0.078;
	var wr = 0.025;
	var wwr = 0.0075;
	var wwn = 12;

	var Robot = function(baseColor, wn) {
		THREE.Object3D.call(this);
		this.wheels = [];
		this.base = null;
		// this.add(new THREE.AxisHelper(0.1));
	}

	Robot.makeOmni = function(baseColor, n) {
		var obj = new Robot();

		var wn = n || 4;

		obj.base = new THREE.Mesh(
			new THREE.CylinderGeometry(r - wwr, r - wwr, 0.015, 32),
			new THREE.MeshPhongMaterial({color: baseColor || 0xff0000})
		);
		obj.base.position.y += wr;
		obj.add(obj.base);

		for(var i = 0; i < wn; i++) {
			var theta = (Math.PI*2) * (i + 0.5)/wn;

			var wheelRef = new THREE.Object3D();
			wheelRef.rotateOnAxis(new THREE.Vector3(0, 1, 0), theta);
			wheelRef.translateOnAxis(new THREE.Vector3(1, 0, 0), r);
			obj.add(wheelRef);

			var wheel = Robot.makeWheel(0);
			wheelRef.add(wheel);
			obj.wheels.push(wheel);
		}

		return obj;
	}
	Robot.makeMecanum = function(baseColor, n) {
		var obj = new Robot();

		var wangle = Math.PI / 4;
		var wn = n || 4;

		obj.base = new THREE.Mesh(
			new THREE.BoxGeometry (2*(r - wwr), 0.015, 2*(r + wr)),
			new THREE.MeshPhongMaterial({color: baseColor || 0xff0000})
		);
		obj.base.position.y += wr;
		obj.add(obj.base);

		for(var i = 0; i < 2; i++) {
			{
				var wheelRef = new THREE.Object3D();
				wheelRef.rotateOnAxis(new THREE.Vector3(0, 1, 0), Math.PI*i);
				wheelRef.translateOnAxis(new THREE.Vector3(1, 0, 0), r);
				wheelRef.translateOnAxis(new THREE.Vector3(0, 0, 1), r);
				obj.add(wheelRef);

				var wheel = Robot.makeWheel(wangle);
				wheelRef.add(wheel);
				obj.wheels.push(wheel);
			}
			{
				var wheelRef = new THREE.Object3D();
				wheelRef.rotateOnAxis(new THREE.Vector3(0, 1, 0), Math.PI*i);
				wheelRef.translateOnAxis(new THREE.Vector3(1, 0, 0), r);
				wheelRef.translateOnAxis(new THREE.Vector3(0, 0, 1), -r);
				obj.add(wheelRef);

				var wheel = Robot.makeWheel(-wangle);
				wheelRef.add(wheel);
				obj.wheels.push(wheel);
			}
		}

		return obj;
	}

	Robot.makeWheel = function(wangle) {
		wangle = wangle || 0;

		var wheel = new THREE.Object3D();

		var t_hub = 0.005;
		var hub = new THREE.Mesh(
			new THREE.CylinderGeometry(wr - wwr + t_hub/2, wr - wwr+ t_hub/2, t_hub, wwn*3),
			new THREE.MeshPhongMaterial({color: 0x808080})
		);
		hub.rotateOnAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
		wheel.add(hub);
		wheel.position.y += wr;

		for(var j = 0; j < wwn; j++) {
			var phi = (Math.PI*2) * (j + 0.5)/wwn;

			var subwheel = new THREE.Mesh(
				new THREE.CylinderGeometry(wwr, wwr, 0.004, 32),
				new THREE.MeshPhongMaterial({color: 0xc0c0c0})
			);
			subwheel.rotateOnAxis(new THREE.Vector3(0, 1, 0), Math.PI / 2);
			subwheel.rotateOnAxis(new THREE.Vector3(0, 0, 1), phi);
			subwheel.rotateOnAxis(new THREE.Vector3(1, 0, 0), wangle);
			subwheel.translateX(wr - wwr);
			wheel.add(subwheel);
		}

		return wheel;
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

	return Robot;
})()