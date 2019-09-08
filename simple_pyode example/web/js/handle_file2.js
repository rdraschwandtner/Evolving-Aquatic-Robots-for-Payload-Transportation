
// --------------------------------------------------------
// Module: _JSON
// 	Progress bar and file drag and drop
// --------------------------------------------------------
var _JSON = (function () {

	// Variables for reading the JSON file
	var json_reader,
		json_data,
		json_message;

	// Variables for interacting with the DOM
	var drop_zone,
		progress;

	// Handle errors while reading the JSON file
	function errorHandler(evt) {
		switch(evt.target.error.code) {
			case evt.target.error.NOT_FOUND_ERR:
				json_message = 'File Not Found!';
				break;
			case evt.target.error.NOT_READABLE_ERR:
				json_message = 'File is not readable';
				break;
			case evt.target.error.ABORT_ERR:
				break; // noop
			default:
				json_message = 'An error occurred reading this file.';
		};
	}

	// Update the progress bar while reading the JSON file
	function updateProgress(evt) {
		// evt is a ProgressEvent.
		if (evt.lengthComputable) {
			var percentLoaded = Math.round((evt.loaded / evt.total) * 100);
			
			// Increase the progress bar length.
			if (percentLoaded < 100) {
				progress.style.width = percentLoaded + '%';
				progress.textContent = percentLoaded + '%';
			}
		}
	}

	// Handle a file when it is dropped onto the canvas
	function handleFileSelect(evt) {
		evt.stopPropagation();
		evt.preventDefault();

		// File from FileList object.
		var json_file = evt.dataTransfer.files[0];
		json_message = '<strong>' + escape(json_file.name) + '</strong>';

		// Reset progress indicator on new file selection.
		progress.style.width = '0%';
		progress.textContent = '0%';

		// Setup FileReader to read the file
		json_reader = new FileReader();
		json_reader.onerror = errorHandler;
		json_reader.onprogress = updateProgress;
		json_reader.onabort = function(e) {
			console.warn('File read cancelled.');
		};
		json_reader.onloadstart = function(e) {
			document.getElementById('progress_bar').className = 'loading';
		};
		json_reader.onload = function(e) {
			// Ensure that the progress bar displays 100% at the end.
			progress.style.width = '100%';
			progress.textContent = '100%';
			setTimeout('document.getElementById("progress_bar").className="";', 2000);
		};
		json_reader.onloadend = function(e) {
			json_data = JSON.parse(this.result);
			_VISUALIZE.runAnimation();
		};

		// Ensure that the file is JSON data
		//if (json_file.type.match('application/json')) {
		
			// Perform the actual read operation (as text)
			json_reader.readAsText(json_file);

			// files is a FileList of File objects. List some properties.
			json_message += ' (' + (json_file.type || 'n/a') + ') - ';
			json_message += json_file.size + ' bytes, last modified: ';
			json_message += json_file.lastModifiedDate ? json_file.lastModifiedDate.toLocaleDateString() : 'n/a';
		//}
		//else {
		//	json_message += ' is not a JSON file, and will not be loaded.';
		//}
		document.getElementById('output_message').innerHTML = '<p>' + json_message + '</p>';
	}

	// Change the pointer icon to show that this is a copy
	function handleDragOver(evt) {
		evt.stopPropagation();
		evt.preventDefault();

		// Explicitly show this is a copy.
		evt.dataTransfer.dropEffect = 'copy';
	}

	// --------------------------------
	// Function code
	// --------------------------------

	// Check for the various File API support.
	if (window.File && window.FileReader && window.FileList && window.Blob) {
		// Great success! All the File APIs are supported.
	} 
	else {
		json_message = 'The File APIs are not fully supported in this browser.';
	}

	// Grab the progress bar item
	progress = document.querySelector('.percent');

	// Setup the drag-and-drop listeners.
	drop_zone = document.getElementById('container');
	drop_zone.addEventListener('dragover', handleDragOver, false);
	drop_zone.addEventListener('drop', handleFileSelect, false);

	
	// --------------------------------
	// Exported functionality
	// --------------------------------
	return {

		// The abortRead function is used by the "Cancel" button
		abortRead: function () {
			json_reader.abort();
			json_message = 'File read cancelled';
		},

		// Export the JSON data
		getData: function () {
			return json_data;
		},

		// Import JSON data
		setData: function (new_json) {
			json_data = new_json;
		}
	};

}());


// --------------------------------------------------------
// Module: _VISUALIZE
// 	Handle visualization playback
// --------------------------------------------------------
var _VISUALIZE = (function () {

	// Animation data
	var anim_data;

	// Variables for THREE basic objects
	var renderer, 
		scene, 
		camera, 
		cameraControls,
		meshes = [];

	// Variables for playback timing
	var time_cur,
		time_step,
		time_stop,
		time_speed = 1,
		clock = new THREE.Clock();

	// Variables for animation control
	var pause,
		time_control,
		requestId,
		first_pass;

	// HTML elements
	var canvas_container,
		html_canvas;

	// Initialize the canvas
	function init() {
	
		// Get the canvas container and element
		canvas_container = document.getElementById('canvas_container'),
		html_canvas = document.getElementById('canvas_threejs');

		// Setup a WebGLRenderer and add to the DOM
		renderer = new THREE.WebGLRenderer({
			canvas: html_canvas,
			antialias: true,
			alpha: true
		});
		renderer.gammaInput = true;
		renderer.gammaOutput = true;
		renderer.setClearColor(0x0, 1.0);
		renderer.shadowMapEnabled = true;
		
		// Setup a camera
		camera = new THREE.PerspectiveCamera(35, 0, 1, 4000);
		camera.position.set(-657, 620, 1120);

		// Setup camera controls
		cameraControls = new THREE.OrbitControls(camera, renderer.domElement);
		cameraControls.target.set(0, 0, 0);

		// Setup the canvas dimension related properties
		resizeScreen();

		// TODO: Setup Stats (stats.js)

		// Create the scene object and setup the default
		scene = new THREE.Scene();
		scene.fog = new THREE.Fog(0x0, 2000, 4000);

		// Lighting
		scene.add(new THREE.AmbientLight(0x222222));

		var spotLight = new THREE.SpotLight(0xFFFFFF, 1.0);
		// spotLight.position.set(-600, 1200, 300);
		spotLight.position.set(0, 1200, 0);
		// spotLight.angle = 20 * Math.PI / 180;
		// spotLight.exponent = 1;
		// spotLight.target.position.set( 0, 200, 0 );
		spotLight.castShadow = true;
		scene.add(spotLight);

		var light = new THREE.DirectionalLight(0xFFFFFF, 0.8);
		light.position.set(500, 300, 200);
		scene.add(light);

		light = new THREE.DirectionalLight(0xFFFFFF, 0.8);
		light.position.set(-500, 300, -200);
		scene.add(light);



		// Ground plane
		// put grid lines every 10000/100 = 100 units
		// polygonOffset moves the plane back from the eye a bit, so that the 
		// lines on top of the grid do not have z-fighting with the grid
		// Factor == 1 moves it back relative to the slope (more on-edge 
		// means move back farther)
		// Units == 4 is a fixed amount to move back, and 4 is usually a 
		// good value
		var solidGround = new THREE.Mesh(
			new THREE.PlaneGeometry(10000, 10000),
			new THREE.MeshPhongMaterial({
				color: 0xFFFFFF,
				// opacity: 0.3,
				// transparent: true,
				polygonOffset: true,
				polygonOffsetFactor: 1.0,
				polygonOffsetUnits: 4.0
			})
		);	  
		solidGround.rotation.x = -Math.PI / 2;
		solidGround.receiveShadow = true;
		scene.add(solidGround);
		
		scene.add(new THREE.GridHelper(10000, 100));
	}

	// Setup the scene using JSON data
	function resetScene() {

		// Initialize scene timing
		pause = true;
		first_pass = true;
		time_control = false;
		anim_data = _JSON.getData();
		time_cur = anim_data.time_start;
		time_step = anim_data.time_step;
		time_stop = anim_data.time_stop;

		// Update the timing slider
		updateTimeSlider(time_stop, time_cur);

		// Remove and delte all objects from the current scene
		for (var i = meshes.length - 1; i >= 0; i--) {
			scene.remove(meshes[i]);
		};
		meshes.length = 0;

		// Add new objects to the scene
		var geometry, material, mesh;
		for (var i = 0; i < anim_data.primitives.length; i++) {
			
			var cur_geom = anim_data.primitives[i].geometry;
			if (typeof cur_geom === 'undefined' || 
				typeof cur_geom.shape === 'undefined') {
				geometry = new THREE.CubeGeometry(100, 100, 100);
				console.warn('No shape provided.');
			}
			else if (cur_geom.shape === 'cube') {
				geometry = new THREE.CubeGeometry(
					cur_geom.sizeX,
					cur_geom.sizeY,
					cur_geom.sizeZ);
			}
			else if (cur_geom.shape === 'cylinder') {
				geometry = new THREE.CylinderGeometry(
					cur_geom.radiusTop,
					cur_geom.radiusBottom,
					cur_geom.height,
					32);// 32 segments
			}
			else if (cur_geom.shape === 'sphere') {
				geometry = new THREE.SphereGeometry(
					cur_geom.radius, 
					32, 32);// 32, 32 segments
			}
			else if (cur_geom.shape === 'ellipsoid') {
				geometry = new THREE.SphereGeometry(
					cur_geom.sizeX * 0.5, 32, 32);
				var matrix = new THREE.Matrix4().makeScale(
					1.0,
					cur_geom.sizeY/cur_geom.sizeX,
					cur_geom.sizeZ/cur_geom.sizeX);
				geometry.applyMatrix(matrix);
			}
			else {
				geometry = new THREE.CubeGeometry(100, 100, 100);
				console.warn('Invalid shape: ' + cur_geom.shape.toString());
			}

			// Create material based on json data or default
			var cur_mat = anim_data.primitives[i].material;
			if (typeof cur_mat === 'undefined' ||
				typeof cur_mat.type === 'undefined') {
				material = new THREE.MeshPhongMaterial();
				material.color.setHex(0xA167FD)
				console.warn('No material provided.');
			}
			else {
				if (cur_mat.type === 'basic') {
					material = new THREE.MeshBasicMaterial();
				}
				else if (cur_mat.type === 'lambert') {
					material = new THREE.MeshLambertMaterial();
				}
				else {
					material = new THREE.MeshPhongMaterial();
				}
				if (typeof cur_mat.color != 'undefined') {
					material.color.setHex(cur_mat.color);
				}
				if (typeof cur_mat.shininess != 'undefined') {
					material.shininess = cur_mat.shininess;
				}
				if (typeof cur_mat.specular != 'undefined') {
					material.specular.setHex(cur_mat.specular);
				}
				if (typeof cur_mat.wireframe != 'undefined') {
					material.wireframe = true;
				}
				if (cur_mat.type != 'basic') {
					material.ambient.copy(material.color);
				}
			}

			// Create the mesh and add to the scene
			mesh = new THREE.Mesh(geometry, material);
			mesh.castShadow = true;
			mesh.receiveShadow = true;
			scene.add(mesh);
			meshes.push(mesh);
		};
	}

	// Resize the canvas element
	function resizeScreen () {
		var canvas_width = canvas_container.clientWidth,
			canvas_height = canvas_container.clientHeight;

		// Resize the canvas to fill its container
		html_canvas.width = canvas_width;
		html_canvas.height = canvas_height;

		// Update the THREE.js objects
		camera.aspect = canvas_width/canvas_height;
		camera.updateProjectionMatrix();
		renderer.setSize(canvas_width, canvas_height);
	}

	// Animate the scene with a canvas animation loop
	function animate() {
		requestId = requestAnimationFrame(animate);
		render();
	}

	// Render the scene
	function render() {
		var time_delta = clock.getDelta();
		cameraControls.update(time_delta);
		
		// First check if enough time has passed to render a frame
		if ((!pause) || first_pass || time_control) {
			first_pass = false;

			// Animate each of the primitive objects
			for (var i = anim_data.primitives.length - 1; i >= 0; i--) {
				var dynamics = anim_data.primitives[i].dynamics;

				// First check that it is visible this frame
				var show_object = false, start_time;
				if (true) {};
				for (var v = dynamics.visible.length - 1; v >= 0; v--) {
					start_time = dynamics.visible[v][0];
					var stop_time = dynamics.visible[v][1];
					if (time_cur >= start_time && time_cur <= stop_time) {
						show_object = true;
						break;
					}
				};

				// If visible, animate the object
				if (!show_object) {
					meshes[i].visible = false;
				}
				else {
					meshes[i].visible = true;

					// Calculate the time and dynamics indices
					var time_offset = time_cur - start_time;
					var time_idx = Math.round(time_offset/time_step);
					
					var dynamics_idx, position_len = dynamics.position.length;
					if (time_idx >= position_len && dynamics.loop) {
						dynamics_idx = time_idx % position_len;
					}
					else if (time_idx >= position_len) {
						dynamics_idx = position_len-1;
					}
					else { 
						dynamics_idx = time_idx; 
					}
					meshes[i].position.set(
						dynamics.position[dynamics_idx][0],
						dynamics.position[dynamics_idx][1],
						dynamics.position[dynamics_idx][2]);

					var quaternion_len = dynamics.quaternion.length;
					if (time_idx >= quaternion_len && dynamics.loop) {
						dynamics_idx = time_idx % quaternion_len;
					}
					else if (time_idx >= quaternion_len) {
						dynamics_idx = quaternion_len-1;
					}
					else { 
						dynamics_idx = time_idx; 
					}
					meshes[i].quaternion.set(
						dynamics.quaternion[dynamics_idx][0],
						dynamics.quaternion[dynamics_idx][1],
						dynamics.quaternion[dynamics_idx][2],
						dynamics.quaternion[dynamics_idx][3]);
				}
			}

			if (!time_control) {
				time_cur += (time_delta * time_speed);
				if (time_cur > time_stop && time_speed >= 0) { 
					time_cur = 0; 
				}
				else if (time_cur < 0 && time_speed < 0) { 
					time_cur = time_stop; 
				}
				updateTimeBox(time_cur);

				if (isNaN(time_cur) || !isFinite(time_cur)) {
					console.warn(time_cur);
				}
			}			
		}

		renderer.render(scene, camera);
	}

	init();

	return {

		// Run the animation loop
		runAnimation: function () {
			resetScene();
			if (!requestId) { animate(); }
		},

		// Reset the size of the screen
		updateScreen: function () {
			resizeScreen();
		},

		// Pause/play the animation
		togglePause: function () {
			pause = !pause;
		},

		// Set the playback speed
		setSpeed: function (new_speed) {
			time_speed = new_speed;
		},

		// Set the current play time
		setTime: function (new_time) {
			time_cur = new_time;
		},

		// Set timing control variable
		setTimeControl: function (new_control) {
			time_control = new_control;
		}
	}

}());

document.addEventListener('keydown', function(event) {
    if (event.keyCode == 80) {
        // togglePPClass();
        document.getElementById('pp_toggle').onclick()
    }
}, true);



var speed_default = 1,
	speed_max = 4,
	speed_min = -4,
	speed_step = 0.25,
	speed_slider = document.getElementById('speed_slider'),
	speed_label = document.getElementById('speed_label');

speed_slider.value = speed_default;
speed_slider.max = speed_max;
speed_slider.min = speed_min;
speed_slider.step = speed_step;
speed_slider.oninput = function () { speed_label.innerHTML = this.value; };
speed_slider.onchange = function () { _VISUALIZE.setSpeed(this.value); };

// Setup speed_slider labels
var siblings = speed_slider.parentNode.childNodes;
for (var i = siblings.length - 1; i >= 0; i--) {
	if (siblings[i].className === 'left_label') {
		siblings[i].innerHTML = speed_min;
	}
	else if (siblings[i].className === 'right_label') {
		siblings[i].innerHTML = speed_max;
	}
	else if (siblings[i].className === 'label_box') {
		siblings[i].innerHTML = speed_default;
	}
};


var updateTime = function () { 
	time_label.innerHTML = this.value;
	_VISUALIZE.setTimeControl(true);
	_VISUALIZE.setTime(parseFloat(this.value));
};

var updateTimeBox = function (new_time) {
	var rounded_time = Math.round(new_time*10)/10;
	time_slider.value = rounded_time;
	time_label.innerHTML = rounded_time;
}


var updateTimeSlider = function (tmax, tmin) {
	var time_default = 0,
		time_max = tmax,
		time_min = tmin,
		time_step = 0.1,
		time_slider = document.getElementById('time_slider'),
		time_label = document.getElementById('time_label');
		
	time_slider.value = time_default;
	time_slider.max = time_max;
	time_slider.min = time_min;
	time_slider.step = time_step;
	time_slider.oninput = updateTime;
	time_slider.onchange = function () { _VISUALIZE.setTimeControl(false); };

	// Setup time_slider labels
	var siblings = time_slider.parentNode.childNodes;
	for (var i = siblings.length - 1; i >= 0; i--) {
		if (siblings[i].className === 'left_label') {
			siblings[i].innerHTML = time_min;
		}
		else if (siblings[i].className === 'right_label') {
			siblings[i].innerHTML = time_max;
		}
		else if (siblings[i].className === 'label_box') {
			siblings[i].innerHTML = time_default;
		}
	};
}
updateTimeSlider(0, 0);


document.getElementById('example1').onclick = function () {

	// Get the local file
	var request = new XMLHttpRequest();
	request.open("GET", "resources/cuboid.json", false);
	request.send(null);
	var json_data = JSON.parse(request.responseText);

	// Set the json data
	_JSON.setData(json_data);

	// Toggle pause/play and restart the animation
	if (document.getElementById('pp_toggle').className == 'pause') {
		document.getElementById('pp_toggle').onclick();
	}
	_VISUALIZE.runAnimation();
}

document.getElementById('example2').onclick = function () {

	// Get the local file
	var request = new XMLHttpRequest();
	request.open("GET", "resources/fish.json", false);
	request.send(null);
	var json_data = JSON.parse(request.responseText);

	// Set the json data
	_JSON.setData(json_data);

	// Toggle pause/play and restart the animation
	if (document.getElementById('pp_toggle').className == 'pause') {
		document.getElementById('pp_toggle').onclick();
	}
	_VISUALIZE.runAnimation();

// var myCanvas = document.getElementById('canvas_threejs');
// var recorder = new PNGSequence( myCanvas );
// recorder.capture(15);

// // Record 5 seconds
// setTimeout(function(){
//   var thePNGDataURLs = recorder.stop();
// }, 5000 );

}

document.getElementById('example3').onclick = function () {

	// Get the local file
	var request = new XMLHttpRequest();
	request.open("GET", "resources/fish2.json", false);
	request.send(null);
	var json_data = JSON.parse(request.responseText);

	// Set the json data
	_JSON.setData(json_data);

	// Toggle pause/play and restart the animation
	if (document.getElementById('pp_toggle').className == 'pause') {
		document.getElementById('pp_toggle').onclick();
	}
	_VISUALIZE.runAnimation();
}


var togglePPClass = function () {
    var current_class = this.className;
    if (current_class == 'play') { 
        this.className = 'pause';
        _VISUALIZE.togglePause();
    } 
    else { 
    	this.className = 'play';
    	_VISUALIZE.togglePause();
    }
}
document.getElementById('pp_toggle').onclick = togglePPClass;

// _VISUALIZE.runAnimation();


window.addEventListener('resize', _VISUALIZE.updateScreen, false);




function PNGSequence( canvas ){
  this.canvas = canvas;
  this.sequence = [];
};
PNGSequence.prototype.capture = function( fps ){
  var cap = this;
  this.sequence.length=0;
  this.timer = setInterval(function(){
    cap.sequence.push( cap.canvas.toDataURL() );
  },1000/fps);
};
PNGSequence.prototype.stop = function(){
  if (this.timer) clearInterval(this.timer);
  delete this.timer;
  return this.sequence;
};













// // var requestId;

// // function loop() {
// //     ...
// //     // do stuff
// //     ...
// //     requestId = window.requestAnimationFrame(loop, canvas);
// // }

// // function start() {
// //     if (!requestId) {
// //        loop();
// //     }
// // }

// // function stop() {
// //     if (requestId) {
// //        window.cancelAnimationFrame(requestId);
// //        requestId = undefined;
// //     }
// // }

