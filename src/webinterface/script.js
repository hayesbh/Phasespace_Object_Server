$(document).ready(function(){
	//FOR ADDING OBJECTS
	//EXPANSION EFFECT
	$("#add").click(function(){
		$("#add_selection").toggle();
	});
	//BUTTON - ROS core_object_server add_object call
	$("#ADD").click(function(){
		addObject($('#AddName').val(), $('#AddTime').val());
		$("#add_selection").hide();
	});
	//CALL THE SAME SERVICE IF ENTER IS CLCIKED
	$("#add_selection").keypress(function(){
		if(event.which == 13){
			addObject($('#AddName').val(), $('#AddTime').val());
			$("#add_selection").hide();
		}
	});
	//IF YOU TRY TO EDIT THE NAME THEN IT WIPES IT FOR YOU
	$('#AddName').click(function(){
		$(this).val('');
	});

	//FOR DELETING OBJECTS
	//EXPANSION EFFECT
	$("#delete").click(function(){
		$("#delete_selection").toggle();
	});
	//DELETE OBJECT ON CLICK
	$("#DELETE").click(function(){
		deleteObject($('#index').val());
		$("#delete_selection").hide();
	});
	//DELETE ON ENTER IF YOU ARE WITHIN THE SECTION
	$("#delete_selection").keypress(function(){
		if(event.which == 13){
			deleteObject($('#index').val());
			$("#delete_selection").hide();
		}
	});
	//FOR THE ADDITION OF POINTS TO AN OBJECT
	//EXPANSION EFFECT
	$('#markers').click(function(){
		$("#markers_section").toggle();
	});
	//CAPTURE MARKERS ON CLICK OF BUTTON
	$('#capture').click(function(){
		addPoints($('#objectindex').val(), $('#time').val());
		$("#markers_section").hide();
	});
	//CAPTURE MARKERS ON ENTER
	$("#markers_section").keypress(function(){
		if(event.which == 13){
			addPoints($('#objectindex').val(), $('#time').val());
			$("markers_section").hide();
		}
	});
});