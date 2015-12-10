var Scene = function(n) {
	THREE.Scene.call(this);
	
	
	// LIGHTS

	dirLightLeft = new THREE.DirectionalLight( 0xffffff, 1 );
	dirLightLeft.color.setHSL( 0.1, 0, 0.95 );
	dirLightLeft.position.set( 0, 0.5, 1 );
	dirLightLeft.position.multiplyScalar( 1 );
	this.add(dirLightLeft);
	
	dirLightRight = new THREE.DirectionalLight( 0xffffff, 1 );
	dirLightRight.color.setHSL( 0.1, 0, 0.95 );
	dirLightRight.position.set( 0, 0.5, -1 );
	dirLightRight.position.multiplyScalar( 1 );
	this.add(dirLightRight);


	var plane = (function() {
		var ms = [
			new THREE.MeshBasicMaterial({color: 0x008000, side: THREE.DoubleSide, transparent: 1, opacity: 0.25}),
			new THREE.MeshBasicMaterial({color: 0x102000, side: THREE.DoubleSide, transparent: 1, opacity: 0.25})
		];
		n = n || 8;
		var g = new THREE.PlaneGeometry(n*0.05, n*0.05, n, n);
		for(var i = 0; i < g.faces.length; i++) {
			si = Math.floor(i / 2);
			g.faces[i].materialIndex = ((si % n)+ Math.floor(si / n)) % 2;
		}
		return new THREE.Mesh(g, new THREE.MeshFaceMaterial(ms));
	})();

	plane.rotateOnAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);
	plane.translateZ(0.001)
	this.add( plane );

}

Scene.prototype = Object.create(THREE.Scene.prototype);
Scene.prototype.constructor = Scene;