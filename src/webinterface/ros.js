var ros = new ROSLIB.Ros();
ros.on('error', function(error) {
	console.log(error);
});
ros.on('connection', function() {
	console.log('Connection made!');
});
ros.connect('ws://192.168.2.121:9090');
var add = new ROSLIB.Service({
	ros : ros,
	name : '/add_object',
	serviceType : 'core_object_server/add_object'
});
function addObject(called, t){
	var addreq = new ROSLIB.ServiceRequest({
		name : called,
    time : t
	});
  add.callService(addreq, function(result){
    console.log('Result for service call on ' +
    add.name +
    ': (' + result.success + ') ' + result.info);
  });
}
var del = new ROSLIB.Service({
  ros : ros,
  name : '/delete_object',
  serviceType : 'core_object_server/delete_object'
});
function deleteObject(identification){
  var delreq = new ROSLIB.ServiceRequest({
    id: identification
  });
  del.callService(delreq, function(result){
    console.log('Result for service call on ' +
    delreq.name +
    ': (' + result.success + ') ');
  });
}
var points = new ROSLIB.Service({
  ros : ros,
  name : '/add_points',
  serviceType : 'core_object_server/add_points'
});
function addPoints(identification, t){
  var pointsreq = new ROSLIB.ServiceRequest({
    id : identification,
    time: t
  });
  points.callService(pointsreq, function(result){
    console.log('Result for service call on ' +
    delreq.name +
    ': (' + result.success + ') ' + result.info);
  });
}

function update(){
	var listener = new ROSLIB.Topic({
	ros : ros,
	name : '/info',
	messageType : 'core_object_server/ObjectDigest'
});
	listener.subscribe(function(message){
			console.log('Received message on ' + listener.name);
			console.log('Received message at ' + message.time);
			for (var i = 0; i < message.objects.size(); i++) {
        var str = "<p>(" + message.objects[i].id + ")" +
                    message.objects[i].name + "</p><p>" +
                    "[" + message.objects[i].pos.x + ', ' +
                          message.objects[i].pos.y + ', ' +
                          message.objects[i].pos.z + "]</p><p>" +
                    "[" + message.objects[i].rot.w + ', ' +
                          message.objects[i].rot.x + ', ' +
                          message.objects[i].rot.y + ', ' +
                          message.objects[i].rot.z + "]</p>";
				console.log('Index: ' + message.objects[i].id);
				console.log('Name: ' + message.objects[i].name);
				console.log('Position: ' +
					message.objects[i].pos.x + ', ' +
					message.objects[i].pos.y + ', ' +
					message.objects[i].pos.z);
				console.log('Rotation: ' +
					message.objects[i].rot.w + ', ' +
					message.objects[i].rot.x + ', ' +
					message.objects[i].rot.y + ', ' +
					message.objects[i].rot.z);
				/*Clear the associated id*/
				$('#'+ substr[0] +' .info').html('');
        $('#'+ substr[0] +' .info').append(str);
			}
		listener.unsubscribe();
	});

}
setInterval(update, 1000);