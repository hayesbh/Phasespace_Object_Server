var index = 0;
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
function addObject(called, t, ty){
  alertify.log("Adding Object for "+ t +" seconds", 1, t*1000);
  console.log(t);
  var addreq = new ROSLIB.ServiceRequest({
    name : called,
    time : +t,
    type : ty
  });
  add.callService(addreq, function(result){
    console.log('Result for service call on ' +
    add.name +
    ': (' + result.success + ') ' + result.info);
    if (result.success) {
      //$('#sidebar').append("<div class='object' id='"+index+"'>"+
      //          "<p>(" + index + ') '+ addreq.name + ': </p>'+
      //          "<p class='info'></p></div>");
      index++;
      alertify.success("Object Successfully Added");
    } else {
      alertify.error("Unable to Add Object: "+ result.info, 2500);
    }
  });
}
var del = new ROSLIB.Service({
  ros : ros,
  name : '/delete_object',
  serviceType : 'core_object_server/delete_object'
});
function deleteObject(identification){
  console.log(identification);
  var delreq = new ROSLIB.ServiceRequest({
    id: parseInt(identification, 10)
  });
  del.callService(delreq, function(result){
    console.log('Result for service call on ' +
    delreq.id +
    ': (' + result.success + ') ');
    if(result.success) {
      //$("#"+delreq.id).remove();
      alertify.success("Object Successfully Deleted");
    } else alertify.error("Deletion Failed");
  });
}
var points = new ROSLIB.Service({
  ros : ros,
  name : '/add_points',
  serviceType : 'core_object_server/add_points'
});
function addPoints(identification, t){
  var pointsreq = new ROSLIB.ServiceRequest({
    id : +identification,
    time: +t
  });
  points.callService(pointsreq, function(result){
    console.log('Result for service call on ' +
    pointsreq.name +
    ': (' + result.success + ') ' + result.info);
    if(resulty.success) {
      alertify.success("Points Successfully Added");
    } else alertify.error("Points Failed To Add");
  });
}

function update(){
  console.log("update");
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/objects',
    messageType : 'core_object_server/ObjectDigest'
  });
  listener.subscribe(function(message){
    console.log('Received message on ' + listener.name);
    console.log('Received message at ' + message.time);
    $('.info').html('');
    for (var i = 0; i < message.objects.length; i++) {
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
      console.log('Position: ' + message.objects[i].pos.x + ', ' +
			 message.objects[i].pos.y + ', ' +
		         message.objects[i].pos.z);
			 console.log('Rotation: ' +
			 message.objects[i].rot.w + ', ' +
	 		 message.objects[i].rot.x + ', ' +
			 message.objects[i].rot.y + ', ' +
		         message.objects[i].rot.z);
      $('.info').append(str);
    }
    listener.unsubscribe();
  });
}
