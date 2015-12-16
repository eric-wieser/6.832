Manager = function(container) {
	THREE.EventDispatcher.call(this);

	this.renderer = new THREE.WebGLRenderer({
		alpha: true,
		antialias: true,
		maxLights: 5, preserveDrawingBuffer: true
	});
	container.append(this.renderer.domElement);

	this.camera = new THREE.PerspectiveCamera(75, container.width() / container.height(), 0.01, 5);
	this.camera.position.z = 0.25;
	this.camera.position.y = 0.125;
	this.camera.position.x = 0.0625;
	this.camera.lookAt(new THREE.Vector3(0,0,0));
	$(window).resize(function() {
	    var w = container.width();
	    var h = container.height();

	    this.renderer.setSize(w, h);
	    this.camera.aspect = w / h;
	    this.camera.updateProjectionMatrix();
	}.bind(this));
	$(window).resize();

	this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
	this.controls.enableDamping = true;
	this.controls.dampingFactor = 0.25;

	this.scene = new THREE.Scene();

	var render = function () {
		requestAnimationFrame(render);

		this.controls.update();

		this.dispatchEvent({type: 'update'});

		this.renderer.render(this.scene, this.camera);
	}.bind(this);

	render();
};
THREE.EventDispatcher.prototype.apply(Manager.prototype);
