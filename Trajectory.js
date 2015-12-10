Trajectory = function(data) {
	this.data = data;
	this.length = data.length / 60;
}

Trajectory.prototype.eval = function(t) {
	var i = Math.floor(t * 60);
	if( i >= this.data.length) return this.data[this.data.length-1];
	if( i < 0) return this.data[0];
	return this.data[i];
};

var loadBinary = function(path) {
	var d = Q.defer();

	var req = new XMLHttpRequest();
	req.open("GET", path, true);
	req.responseType = "arraybuffer";

	req.onload = function (oEvent) {
		var arrayBuffer = req.response;
		if (arrayBuffer) {
			d.resolve(arrayBuffer);
		} else {
			d.reject(new Error("No response"))
		}
	};

	req.onerror = d.reject;

	req.send(null);

	return d.promise;
};

Trajectory.load = function(path, w) {
	return loadBinary(path).then(function(raw) {
		raw = new DataView(raw);

		n = raw.byteLength / (8*w);
		if(n != Math.floor(n)) throw Error('Bad width!');

		data = [];
		for(var i = 0; i < n; i++) {
			var row = [];
			for(var j = 0; j < w; j++) {
				row.push(raw.getFloat64(8*(i*w + j), true));
			}
			data.push(row);
		}

		return new Trajectory(data);
	})
}