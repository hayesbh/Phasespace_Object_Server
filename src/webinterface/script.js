$(document).ready(function(){
	var index = 0;
	$("#add").click(function(){
		$("#add_selection").toggle();
	});
	$("#ADD").click(function(){
		$("#add_selection").hide();
		$('#sidebar').append("<div class='object' id='"+index+"'>"+
								"<p>(" + index + ') '+ $('#AddName').val() + ': </p>'+
								"<p class='info'></p></div>");
		addObject($('#AddName').val());
		index++;
	});
	$("#add_selection").keypress(function(){
		if(event.which == 13){
			$("#add_selection").hide();
			$('#sidebar').append("<div class='object' id='"+index+"'>"+
								"<p>(" + index + ') '+ $('#AddName').val() + ': </p>'+
								"<p class='info'></p></div>");
			addObject($('#AddName').val());
			index++;
		}
	});
	$('#AddName').click(function(){
		$(this).val('');
	});


	$("#delete_selection").keypress(function(){
		if(event.which == 13){
			$("#delete_selection").hide();
			$("#"+$('#index').val()).remove();
		}
	});
	$("#delete").click(function(){
		$("#delete_selection").toggle();
	});
	$("#DELETE").click(function(){
		$("#delete_selection").hide();
		$("#"+$('#index').val()).remove();
	});

	$('#markers').click(function(){
		$("#markers_section").toggle();
	});
	$("#markers_section").keypress(function(){
		if(event.which == 13){
			$("markers_section").hide();
		}
	});
	$('#capture').click(function(){
		$("#markers_section").hide();
	});
});