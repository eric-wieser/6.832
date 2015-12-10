var Scene = function() {
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

}

Scene.prototype = Object.create(THREE.Scene.prototype);
Scene.prototype.constructor = Scene;