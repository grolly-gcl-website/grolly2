<?php
/*
if ( $http_user_agent ~* (nmap|nikto|wikto|sf|sqlmap|bsqlbf|w3af|acunetix|havij|appscan) ) {
    return 403;
}
*/

session_start();
$_SESSION['cadiweb_version'] = '1.0';

?>

<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Cadi web UI</title>
<!---
<link rel="stylesheet" href="http://code.jquery.com/ui/1.10.3/themes/smoothness/jquery-ui.css">
<script src="http://code.jquery.com/jquery-1.9.1.js"></script>
<script src="http://code.jquery.com/ui/1.10.3/jquery-ui.js"></script>
<link rel="stylesheet" href="/resources/demos/style.css">  -->
<link rel="stylesheet" href="css/resetcss.css">

<link rel="stylesheet" href="js/jquery-ui-1.11.2.custom/jquery-ui.css">
<!--- <link rel="stylesheet" href="css/jquery-ui-timepicker-addon.css"> -->
<script src="js/jquery-1.11.2.min.js"></script>
<script src="js/jquery-ui-1.11.2.custom/jquery-ui.min.js"></script>
<script src="js/jquery-ui-timepicker-addon.js"></script>

<!---
<link rel="stylesheet" href="js/jquery-ui-1.10.3.custom/css/smoothness/jquery-ui-1.10.3.custom.css">
<script src="js/jquery-ui-1.10.3.custom/js/jquery-1.9.1.js"></script>
<script src="js/jquery-ui-1.10.3.custom/js/jquery-ui-1.10.3.custom.js"></script> -->
<link rel="stylesheet" type="text/css" href="css/jquery.svg.css"> 
<script type="text/javascript" src="js/svg/jquery.svg.js"></script>
<script type="text/javascript" src="js/svg/jquery.svganim.js"></script>
<script src="js/raphael-min.js"></script>
<script src="js/justgage.js"></script>

<link rel="stylesheet" href="css/style.css">

<!-- <script type="text/javascript" src="js/date.format.js"></script> -->

<?php
// load tank levels settings
	$row = 1;
	if (($handle = fopen("cm/cadi_settings", "r")) !== FALSE) {
	    while (($rowarr = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$num = count($_SESSION['settings_data'][$row]);
		if (substr($rowarr[0],0,1)!='#') {
			$_SESSION['settings_data'][($rowarr[0])] = $rowarr;
			$row++;
		}
	    }
	    fclose($handle);
	}
	$svg_t3top = $_SESSION['settings_data']['watertank_top_bottom'][1];
	$svg_t3btm  = $_SESSION['settings_data']['watertank_top_bottom'][2];
    	$svg_t4top = $_SESSION['settings_data']['watertank_top_bottom'][3];
    	$svg_t4btm  = $_SESSION['settings_data']['watertank_top_bottom'][4];	
	$psi0psi_ = $_SESSION['settings_data']['psi_sensor'][1];
	$psi32psi_ = $_SESSION['settings_data']['psi_sensor'][2];	
?>

<script>

// replaced by rx_ee_(). Commented out 150809
/*
function rx_ee_bak(addr){	// CSX format data upload
	var csx_data = $('#csxform').serialize();
	alert('updating addr '+addr);
	$.post('cm/cadi_bt_processor.php', {action: 'upload_csx', csx_data:csx_data}, function(data){
		alert(data);
		var number = $('#settings_number').val();	// amount of 16 bit variables to upload to cadi
		$.post('cm/cadi_bt_processor.php', {action: 'rx_ee', addr:addr, number:2}, function(data){});	// 2 vars - HARDCODE	
	});
}





function rx_ee(addr){	// CSX format data upload
	var csx_data = $('#csxform').serialize();
	alert('updating addr '+addr);
	$.post('cm/cadi_bt_processor.php', {action: 'upload_csx', csx_data:csx_data}, function(data){
		alert(data);
		var number = $('#settings_number').val();	// amount of 16 bit variables to upload to cadi
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:115, addr:addr}, function(data){
			alert(data);
		});	// 2 vars - HARDCODE	
	});
}





function rx_ee_this(input){	// semi-direct value upload
	var inputName = input.name;
	var inputData = inputName.split('_');
	var addr = inputData[1];
	var value = input.value;
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:64, addr:addr, value:value}, function(data){
	});
}

*/

function rx_ee_dir(addr, value){	// direct value upload
//	$.post('cm/cadi_bt_processor.php', {action: 'rx_ee_dir', addr:addr, value:value}, function(data){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:64, addr:addr, value:value}, function(data){
		alert(addr+'/'+value);
	});
}





/* accepts the <input>. The 'name' of <input> should be in format:
 aaa_bbb_ccc, where
	- aaa: fields group name (should not intersect with other groups if setting appeared twice)
	- bbb: value address in eeprom
	- ccc: something else
the value is extracted from the 'value' property of <input>
*/
function rx_ee_(input){
	var inputName = input.name;
	var inputData = inputName.split('_');
	var addr = inputData[1];
	var value = input.value;
	// alert('addr='+addr+';val='+value);
	var i=0;
//	for (i=0;i<3;i++) {
		 $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:64, addr:addr, value:value}, function(data){
		//	alert(data);
		});
//	}
}

var g1;
window.onload = function(){
	var g1 = new JustGage({
		id: "psi_gauge",
		value: getRandomInt(0, 100),
		min: 0,
		max: 90,
		title: "Pressure",
		label: "PSI"
	});

	setInterval(function() {
		var gaugeVal = $('#psi_gauge_val').val();
		g1.refresh(gaugeVal);
	}, 500);
}; 





	$(function() {
		$( "#cadi_tabs" ).tabs({
			beforeLoad: function( event, ui ) {
				ui.jqXHR.error(function() {
				ui.panel.html(
				"Couldn't load this tab. We'll try to fix this as soon as possible. " +
				"If this wouldn't be a demo." );
				});
			}
		});
		$('#svg_container').svg({onLoad: drawMapLayer});

		$( ".btn_" ).button();
		$("#system_view_1").hide();
//		get_status_block();
		cadi_list_rfcomms();

		<?php echo 'var psi0psi_ = '.$psi0psi_.';'.PHP_EOL; ?>
		<?php echo 'var psi32psi_ = '.$psi32psi_.';'.PHP_EOL; ?>
		//cadi_status_stream();
	});


$(document).ready(function() {
	cadi_status_stream();
});
	var t3curlvl = 0;
	var t4curlvl = 0;
	function drawMapLayer(svg){


		// draw water levels
		// tank max level in pixels
		tmlp = 135;		// deprecated?
	

 //   		var g = svg.group({stroke: 'black', strokeWidth: 2});

		// draw cadi time 
		svg.text(100, 445, 'Cadi time',{fill: 'red', strokeWidth: 0, fontSize: '29', id:'cadi_time2'});
		svg.text(140, 515, 'Temp',{fill: 'green', strokeWidth: 0, id:'cadi_temp'});
		svg.text(140, 535, 'rH',{fill: 'blue', strokeWidth: 0, id:'cadi_rh'});

		svg.text(140, 565, 'pH',{fill: 'red', strokeWidth: 0, id:'ph1_adc_val'});

		svg.text(635, 120, 'B',{fill: 'green', strokeWidth: 0, id:'blooml'});
		svg.text(635, 200, 'G',{fill: 'green', strokeWidth: 0, id:'growl'});

/*
		svg.text(610, 335, 'TF1',{fill: 'green', strokeWidth: 0, id:'tf1'});
		svg.text(610, 355, 'TF2',{fill: 'green', strokeWidth: 0, id:'tf2'});
		svg.text(610, 385, 'TF3',{fill: 'green', strokeWidth: 0, id:'tf3'});
		svg.text(610, 405, 'TF4',{fill: 'green', strokeWidth: 0, id:'tf4'});

		svg.text(610, 445, 'TSF',{fill: 'black', strokeWidth: 0, id:'tsf'});
		svg.text(610, 465, 'CTSF',{fill: 'black', strokeWidth: 0, id:'ctsf'});
*/
		// draw tank levels in text
		svg.text(150, 385, '2Top',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t3l_txt'});
		svg.text(390, 385, '2Top',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t4l_txt'});

		// display pressure
		svg.text(610, 445, '',{fill: 'red', strokeWidth: 0, fontSize: '30', id:'psi_val'});

		// display CDD status text
		svg.text(140, 475, 'CDD status',{fill: 'red', strokeWidth: 0, id:'cdds'});

		// display auto_flags
		svg.text(140, 495, 'auto_flags:',{fill: 'red', strokeWidth: 0, id:'auto_flags'});

	}

	

	function redraw_svg_layer(){
	
		
		
		$.post('cm/cadi_bt_processor.php', {action: 'get_status_csv'}, function(data){
		//	alert(data);
			if (data.length>42) {
				//$('#csv_string_box').html(data);
				var statusArray = data.split(',');
				// display BTD State
				$('#btd_state').html(statusArray[22]);

				if (statusArray[0]>1000000000) {  // workaround for first lost char of CSV (shared memory)
					var date = new Date(statusArray[0]*1000);
					//var date2 = date.substr(0,16);
					var datestring = date.toString().substr(0,24);
					$('#cadi_time2').html(datestring);
				}
				var temp = parseFloat(statusArray[1]).toFixed(1);
				var rh = parseFloat(statusArray[2]).toFixed(1);
				$('#cadi_temp').html('&nbsp;T: '+temp+'C');
				$('#cadi_rh').html('rH: '+rh+'%');


				
				var ph1_adc_val = statusArray[10];
				$('#ph1_adc_val').html('pH: '+ph1_adc_val);
				$('#sens_ph1_adc_val').html(ph1_adc_val);
				//$('#csv_string').html(data);
				

				/* var wpProgress = statusArray[18];
				$('#tf1').html(wpProgress);
				var auto_failures = statusArray[19];
				$('#tf2').html(auto_failures);
				var runners = statusArray[20];
				$('#tf3').html(runners);

				var tsf = statusArray[3];		// timer state flags
				$('#tsf').html(tsf);
				var ctsf = statusArray[4];		// ctimer state flags
				$('#ctsf').html(ctsf);	*/




				// tank 3 water level redraw
		
			<?php echo 't3top = '.$svg_t3top.';'.PHP_EOL; ?>
			<?php echo 't3btm = '.$svg_t3btm.';'.PHP_EOL; ?>
			t3curlvl = statusArray[8]; 	// provide current level globally
			if (statusArray[8]>t3top && statusArray[8]<t3btm) {
				var lvl = Math.floor((275 * (t3btm-statusArray[8])/(t3btm-t3top)));
				var t3h = 275 - lvl;
				var t3y = 167 + lvl;
			}
			else {
				t3h = 275;
				t3y = 167;
			}
			$('#tank3_water').attr('height',t3h);
			$('#tank3_water').attr('y',t3y);

	
			// $('#ph1_adc_val').html('&nbsp;pH: '+ph1_adc_val);


				// tank 4 water lvl redraw
			<?php echo 't4top = '.$svg_t4top.';'.PHP_EOL; ?>
			<?php echo 't4btm = '.$svg_t4btm.';'.PHP_EOL; ?>

			t4curlvl = statusArray[9];	// provide current level globally
			if (statusArray[9]>t4top && statusArray[9]<t4btm) {
				var lvl2 = Math.floor((250 * (t4btm-statusArray[9])/(t4btm-t4top)));
				var t4h = lvl2;
				var t4y = 200 + 243 - lvl2;
			}
			else {
				t4h = 243;
				t4y = 200;
			}
			$('#tank4_water').attr('height',t4h);
			$('#tank4_water').attr('y',t4y);

				$('#psi_adc_current').html(statusArray[12]); 
				// offset for PSI gauge value
				var psi_offset = -1;
				// draw labels for tanks, displaying current level
				$('#t3l_txt').html('2Top: '+statusArray[8]+'mm');
				$('#t4l_txt').html('2Top: '+statusArray[9]+'mm');
				// var psi_gauge_val = (statusArray[14]*16)+psi_offset;

				var psi_gauge_val = ((statusArray[12]-psi0psi_)/((psi32_psi_ - psi0psi_)/32));
				
				//if (psi_gauge_val) {
					$('#psi_gauge_val').val(psi_gauge_val);
				//}
				// $('#psi_val').html(statusArray[14]+'bar');

				// get valve state circles' colors
				var valves = statusArray[5];
				for (var i=0;i<10;i++) {
					if (valves.charAt(9-i)=="1") {
						$('#valve_'+i).attr('fill', 'green');
					}
					else {
						$('#valve_'+i).attr('fill', 'red');
					}
				}


				// Grolly PSI pump status
				var psi_pump_state = parseInt(statusArray[21]);
				if (psi_pump_state==100) {
					$('#psi_pump').attr('fill', 'red');
				}
				else if (psi_pump_state>1 && psi_pump_state<100) {
					$('#psi_pump').attr('fill', 'green');
				}
				else {
					$('#psi_pump').attr('fill', 'red');
				}

				// Grolly mix pump status
				if (statusArray[16].charAt(3)=="1") {
					$('#mix_pump').attr('fill', 'green');
				}
				else {
					$('#mix_pump').attr('fill', 'red');
				}


				// dosing pumps status squares colors
				for (var i=1;i<5;i++) {
					if (statusArray[16].charAt(4-i)=="1") {
						$('#cdp'+i).attr('fill', 'green');
					}
					else {
						$('#cdp'+i).attr('fill', 'red');
					}
				}

				


				if (statusArray[15]==51){	
					$('#cdds').html('CDD Enabled');
					$('#cdds').attr('fill', 'green');
				}
				else {
					$('#cdds').html('CDD Disabled');
					$('#cdds').attr('fill', 'red');
				}
				var af_bin = statusArray[17].toString(2);
				$('#auto_flags').html(statusArray[17]+' ('+af_bin+')');

				var vs_flag = $('#vs_flag').is(':checked');
				if (vs_flag==1) {
					d = new Date();
				//	alert('vs checked');
					$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
				}
			}
		});
	}

	function get_ip(){
		$.post('cm/cadi_bt_processor.php', {action: 'get_ip'}, function(data){
			alert(data);
		});
	}

	function change_video(){
		var new_video =$("#video_stream").val();
		$.post('cm/cadi_bt_processor.php', {action: 'change_video', new_video:new_video}, function(data){
			alert('new video='+new_video+' ----- '+data);
		});
	}

	function toggleValve(valveId){
		alert(valveId);
	}

	function bt_connect(){
		//alert('start');
		var mac = $('#bind_mac').val();
		var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
		$.post('cm/cadi_bt_processor.php', {action: 'bt_connect', mac:mac, rfcomm: rfcomm}, function(data){
			cadi_list_rfcomms();
			$('#main_output').html(data);
		//	alert(data);
		});
		//alert('connecting');
	}



	function bt_restart(){
		alert('restarting');
		$.post('cm/cadi_bt_processor.php', {action: 'bt_restart'}, function(data){
			alert('restarted');
			cadi_list_rfcomms();
			$('#main_output').html(data);
		});
	}

	function run_watering_pump(){
		var secs = $("#run_watering_secs").val();
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '35', secs:secs}, function(data){
			cadi_list_rfcomms();
		});

	}

	function run_demo(){
		alert("Demo!");
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '34'}, function(data){
			cadi_list_rfcomms();
		});

	}

	function rcmd(cmd){		// packet created directly in packet processor
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: cmd}, function(data){
			$('#main_output').html(data);
		});
	}

	function bt_disconnect(){
		btd_stream_status(0);	// Stop pinging Cadi before disconnect
//		setTimeout(function(){}, 100);
		var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
		$.post('cm/cadi_bt_processor.php', {action: 'bt_disconnect', rfcomm:rfcomm}, function(data){
			alert('disconnected');
			cadi_list_rfcomms();
			$('#main_output').html(data);
	
		});
	}

	function get_status_block(){
		var blocks = $('#status_block_ids').val();
		var block_ids = blocks.split(',');
		for (i=0; i<block_ids.length;i++){
//			$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 7, block_id:block_ids[i]}, function(data){
//			});
			$.post('cm/cadi_bt_processor.php', {action: 'get_status'}, function(data){
				blocks = data.split('---socalledseparator---');
				$('#status_block').html(blocks[0]);
				$('#system_view_2').html(blocks[1]);
				d = new Date();
				var vs_flag = $('#vs_flag').is(':checked');
				if (vs_flag==1) {
					alert('vs checked');
					$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
				}
			});
		}
	}



	function check_plug(){
		alert("checking!");
		$(function(){
			$('#radio_plug1').attr('checked','checked');
			$('#plug1_radio').attr('checked','checked');
			$('#plug1_radio1').attr('checked','true');
			$("radio_").buttonset("refresh");
			$("#radio_plug1").buttonset("refresh");
		});
	}

	function cadi_bt_scan(){
	//	alert('rfcomm scan');
	//	$('#main_output').html('scanning..');
		$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_scan'}, function(data){
			$('#bind_mac').html(data);
	//		alert('Scanning complete!');
	//		alert(data);
	//		cadi_list_rfcomms();
		});  
	}

	function cw_reboot(){	// reboots the machine, running CadiWeb server
		if (confirm('Reboot Cadi server?')) {
			$.post('cm/cadi_bt_processor.php', {action: 'reboot'}, function(data){
			}); 
		}
	}

	function cadi_reset(){
		if (confirm('Reset Cadi?')) {
			rcmd(13);
		}
	}

	function cadi_list_rfcomms(){
	//	alert('listing binds');
		$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_list_binded'}, function(data){
			$('#binded_rfcomms').html(data);
		});
	}

	function stopSerialRead(psid){
	//alert('call'+psid);
		$.post('cm/cadi_bt_processor.php', {action: 'stop_serial_read', process:psid}, function(data){
			cadi_list_rfcomms();
	//		alert('kill -9 sent');
		});  
	}

	function bt_tx_packet() {
		var data = $('#tx_data_packet').val();
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', data: data}, function(data){
			cadi_list_rfcomms();
			alert(data);
		});  
	} 


	function bt_tx() {
		var data = $('#tx_data').val();
		$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd: '1', plug_id:'1', state:data}, function(data){
			cadi_list_rfcomms();
	//		alert('kill -9 sent');
		});  
	}

	function plugStateSet(plug, state){
	//	alert('oga');
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data){
			cadi_list_rfcomms();
	//		alert(data);
		});  
	}

	function bt_setdd() {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
			cadi_list_rfcomms();
	//		alert(data);
		});  
	}


	function cadi_status_stream(){
		var state = $('#flag_status_stream').is(':checked');
		if (state==1) {
//			var interval = setInterval(function(){get_status_block()},1000);	// enables drawing SVG with PHP
			var delay=$("#status_stream_delay").val();
			var interval = setInterval(function(){redraw_svg_layer()},delay);	// enables SVG draw with JS
			$('#status_stream_interval').val(interval);
			//alert(interval+' created');
		}
		if (state==0) {
			var interval = $('#status_stream_interval').val();
			// alert(interval+' cleared');
			clearInterval(interval);
		}
	}

	
function cadi_view(viewId){
	switch (viewId) {
		case 1:
			$("#system_view_1").show();
			$("#system_view_2").hide();
			break;
		case 2:
			$("#system_view_2").show();
			$("#system_view_1").hide();
			break;
	} 

}

function btd_stream_status(newStatus){
	//	alert();
		$.post('cm/cadi_bt_processor.php', {action: 'btd_stream_start', status: newStatus}, function(data){
	//		cadi_list_rfcomms();
	//		alert('Streaming to server cache');
		});  
}

function redraw_update_log(){
		$.post('cm/cadi_bt_processor.php', {action: 'redraw_update_log'}, function(data){
			$('cadi_update_log').html(data);
		});  
}

function cadiweb_update(){
		btd_stream_status(0);
		$.post('cm/cadi_bt_processor.php', {action: 'cadiweb_update'}, function(data){});
		alert("Do not use Cadiweb control panel until server finishes software upgrade with reboot!");
		var log_upd_int_id = setInterval(function(){redraw_update_log},1000);	// enables SVG draw with JS
		$('#log_interval_id').val(log_upd_int_id);
}

function stop_log_stream(){
		var  interval = $('#log_interval_id').val();
		clearInterval(interval);
}

function eeRead() {
	var addr = $('#ee_addr').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:18, addr:addr}, function(data){
		alert('EEPROM value read');
		var out = data.split('seppy');
		$('ee16bit_value').val(out[0]);
		$('ee32bit_value').val(out[1]);
	});  
}

function eeWrite(dataType) {
	var addr = $('#ee_addr').val();
	var cmd=0;
	switch ($dataType) {
		case 1:
			var value = $('#ee16bit_value').val();
			cmd = 15;
			break;
		case 2:
			var value = $('#ee32bit_value').val();
			cmd = 16;
			break;
	}
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', data_type:dataType, value:value, cmd:cmd}, function(data){
		alert('EEPROM value sent');
	}); 
 
}

function enable_dlsettings_overlay(state){
	if (state==1) {
		$('.lcso').css('display', 'block');
	}
	else {
		$('.lcso').css('display', 'none');
	}
}






function download_csx(type){
	enable_dlsettings_overlay(1);
	var interval_csxdl = $('#csxdl_interval').val();
	clearInterval(interval_csxdl);
	setTimeout(function(){ }, 1000);
	$.post('cm/cadi_bt_processor.php', {action: 'dl_settings'}, function(data){
		// alert(data);
		var interval_csxdl = setInterval(function(){csx_dl_proc(type)},1000);	
		$('#csxdl_interval').val(interval_csxdl);	
	});
}




// running during Cadi settings download
function csx_dl_proc(type){
	
	$.post('cm/cadi_bt_processor.php', {action: 'check_dl_set_status'}, function(data){
		
		var tmparr = data.split('v0t0n0');
		var btdState = tmparr[1];
		if (btdState == "1") {				// if BTDaemon is idle
			var interval_csxdl = $('#csxdl_interval').val();
			clearInterval(interval_csxdl); 		//, clear interval for checking status
			if (type==0) {
				$.post('cm/csx_get_table.php', {}, function(data){
					$('#csxform').html(data);	//  and reload settings table
				});
			}
			enable_dlsettings_overlay(0);
		}	
	});

	
}


function mix_solution(){
	var secs = $('#run_watering_secs').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:38, secs:secs}, function(data){
		
	});  

}


</script>
</head>
<body>
<input type="hidden" id="status_stream_interval" value="" />
<div id="cadi_tabs">
<ul>
<li><a href="#tabs-1">Status</a></li>
<li><a href="cm/cadi_timers.php">Timers</a></li>
<li><a href="cm/cadi_watering.php">Watering</a></li>
<li><a href="cm/cadi_fertilization.php">Fertilization</a></li>
<li><a href="cm/cadi_sensors.php">Sensors</a></li>
<li><a href="cm/cadi_advanced.php">Advanced</a></li>
<!--<li><a href="cm/cadi_dd.php">Direct drive</a></li> -->
</ul>

<div id="csv_string"></div>

<div class="ral">
<b id="btd_state">BTD state</b>
<button onClick="download_csx(1)">Reload Cadi Settings</button>
<div class="lcso">Downloading Cadi settings. Please wait ...</div>
<input type="hidden" value="0" id="csxdl_interval" />
<!--
<button class="btn_ fr" onClick="check_plug()">Apply settings</button>
<button class="btn_ fr">Reload settings</button>
-->
</div>
<div id="tabs-1">
<input type="checkbox" checked id="flag_status_stream" onClick="cadi_status_stream()" />Stream STATUS (from server cache)
<input type="checkbox" id="vs_flag" /> also Video
<button class="btn_" id="cadi_view_cam" onClick="cadi_view(1)">Cam</button>
<button class="btn_" id="cadi_view_map" onClick="cadi_view(2)">Map</button>
<br>
<table>
	<tr>
		<td style="vertical-align:top;">
			<div id="system_view_1">
				<div id="status_block" style="float:left;">
					<?php include_once('cm/status_view_1.php'); ?>
				</div>
				<img id="cadi_img" style="float:right;" src="img/curimage.jpeg?" />
			</div>
			<div id="system_view_2" style="border: 0px solid blue;">
				<div style="float:left;">
					<div style="float:left;">




<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:xlink="http://www.w3.org/1999/xlink"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   width="950"
   height="744.09448"
   id="svg2"
   version="1.1"
   inkscape:version="0.91 r13725"
   sodipodi:docname="grolly2_cadiweb_schema.svg"
   style="margin-left:-200px;">
  <defs
     id="defs4">
    <linearGradient
       id="linearGradient4127">
      <stop
         style="stop-color:#00ffcc;stop-opacity:1;"
         offset="0"
         id="stop4129" />
      <stop
         style="stop-color:#00ffcc;stop-opacity:0;"
         offset="1"
         id="stop4131" />
    </linearGradient>
    <inkscape:perspective
       sodipodi:type="inkscape:persp3d"
       inkscape:vp_x="-13.479251 : 207.70134 : 1"
       inkscape:vp_y="0 : 1526.5795 : 0"
       inkscape:vp_z="1385.611 : 207.70134 : 1"
       inkscape:persp3d-origin="686.06588 : 18.381438 : 1"
       id="perspective2995" />
    <linearGradient
       inkscape:collect="always"
       xlink:href="#linearGradient4127"
       id="linearGradient4137"
       x1="-217.82143"
       y1="-429.05859"
       x2="-158.60715"
       y2="-431.91571"
       gradientUnits="userSpaceOnUse" />
  </defs>
  <sodipodi:namedview
     id="base"
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1.0"
     inkscape:pageopacity="0.0"
     inkscape:pageshadow="2"
     inkscape:zoom="1.98"
     inkscape:cx="711.53939"
     inkscape:cy="373.49349"
     inkscape:document-units="px"
     inkscape:current-layer="layer1"
     showgrid="false"
     inkscape:object-paths="false"
     inkscape:snap-nodes="true"
     showguides="true"
     inkscape:guide-bbox="true"
     inkscape:window-width="1280"
     inkscape:window-height="999"
     inkscape:window-x="1362"
     inkscape:window-y="-4"
     inkscape:window-maximized="1">
    <sodipodi:guide
       position="874.28571,297.14286"
       orientation="1,0"
       id="guide4638" />
  </sodipodi:namedview>
  <metadata
     id="metadata7">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title></dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <g
     inkscape:label="Layer 1"
     inkscape:groupmode="layer"
     id="layer1"
     transform="translate(450,-100)">
    <rect
       style="fill:#ffffff;stroke:#2b0000;stroke-width:2.8900001"
       id="rect3025"
       width="170.78143"
       height="278.79703"
       x="-121.50121"
       y="164.93553" />
    <rect
       style="fill:#ffffff;stroke:#2b0000;stroke-width:2.8900001"
       id="rect3027"
       width="160.56375"
       height="243.76497"
       x="105.9947"
       y="199.79051" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.78540301px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -47.207938,503.10101 0,17.53453 c 0,0 4.063494,32.14659 40.6348806,32.14659 l 254.7936774,0"
       id="path3049"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccsc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -26.265727,501.49284 0,10.56855 c 0,0 -2.554422,14.43536 26.274081,14.43536 l 64.955591,0"
       id="path3065"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccsc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 42.967507,526.49675 107.243963,0 c 0,0 26.63899,4.55444 26.63899,-24.27406"
       id="path3073"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 249.54074,525.92532 -28.38681,0 c 0,0 -24.13393,1.52414 -24.63898,-22.75891"
       id="path3073-1"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccc" />

    <circle
       style="stroke:#2b0000;stroke-width:2.8900001"
	fill = "none" 
       id="psi_pump"
       inkscape:transform-center-x="-12.407197"
       cx="274.66299"
       cy="540.29095"
       r="29.193409" />
<!---

<path
       sodipodi:type="arc"
	fill = "none"
       style="stroke:#2b0000"
       id="psi_pump"
       sodipodi:cx="244.66299"
       sodipodi:cy="540.29095"
       sodipodi:rx="10.101525"
       sodipodi:ry="10.101525"
       d="m 142.43151,143.05371 a 10.101525,10.101525 0 1 1 -20.20305,0 10.101525,10.101525 0 1 1 20.20305,0 z"
       transform="matrix(2.89,0,0,2.89,-57.352203,120.93211)"
       inkscape:transform-center-x="-12.407197" />
--->


    <path
       style="fill:none;stroke:#000000;stroke-width:3.35290217px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 340.40898,442.93579 0,120.78544"
       id="path3181"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 313.46277,386.57396 0,-202.16434 c 0,0 1.09477,-21.16522 -19.34063,-21.16522 -20.43539,0 -60.21141,0 -60.21141,0"
       id="path3183"
       inkscape:connector-curvature="0" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 337.18243,386.7805 0,-221.86992 c 0,0 -4.37903,-22.98981 -23.71966,-22.98981 -19.34063,0 -79.18712,-0.72983 -79.18712,-0.72983 l -15.69146,0 c 0,0 -18.24586,1.09475 -18.24586,16.05636 0,14.96163 0,14.96163 0,14.96163 l 26.63896,0.36493 c 0,0 -1.82456,-8.39311 7.29836,-8.75803"
       id="path3185"
       inkscape:connector-curvature="0" />
    <path
	fill = "none"
       style="stroke:#000000;stroke-width:2.3386085px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 208.29633,445.66506 -44.52654,56.26335 44.52654,0 -44.52654,-56.26335 z"
       id="valve_8"
       inkscape:connector-curvature="0" />
    <path
       style="stroke:#000000;stroke-width:2.42191815px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
	fill = "#ababab;"
       d="m 362.46807,552.63975 60.49054,44.41823 0,-44.41823 -60.49054,44.41823 z"
       id="valve_1"
       inkscape:connector-curvature="0" />
    <path
       style="stroke:#000000;stroke-width:2.42043948px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
fill = "#ababab;"
       d="m 362.67329,601.14697 60.40446,44.42721 0,-44.42721 -60.40446,44.42721 z"
       id="valve_2"
       inkscape:connector-curvature="0" />
    <path
	fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.4169898px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 362.68289,650.01252 60.21211,44.44219 0,-44.44219 -60.21211,44.44219 z"
       id="valve_3"
       inkscape:connector-curvature="0" />
    <path
	fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 313.46277,554.53004 0,245.04922"
       id="tl_xxx"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 312.81607,800.96218 c 0,0 4.5337,28.45 14.45,28.45 l 33.54464,0"
       id="path3243"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="csc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 339.18243,565.4979 13.15152,0.002 9.48686,0"
       id="path3245"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.31576,631.63027 -23.62828,0 0,28.05893 23.62828,0"
       id="path3247"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.82081,680.85915 -23.62828,0 0,31.09107 23.62828,0"
       id="path3249"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cccc" />
    <path
       sodipodi:type="arc"
       fil="none"
       style="stroke:#2b0000;stroke-width:1.78571427"
       id="cdp4"
       sodipodi:cx="337.14285"
       sodipodi:cy="-168.40552"
       sodipodi:rx="12.5"
       sodipodi:ry="12.5"
       d="m 349.64285,-168.40552 a 12.5,12.5 0 0 1 -12.5,12.5 12.5,12.5 0 0 1 -12.5,-12.5 12.5,12.5 0 0 1 12.5,-12.5 12.5,12.5 0 0 1 12.5,12.5 z"
       transform="matrix(1.6184,0,0,1.6184,-178.34868,561.55834)" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.592,272.44811 c 4.3232,-1.55115 10.83903,14.57915 10.83903,14.57915 0,0 5.2387,16.74226 0.92231,18.43376 -4.81066,1.88511 -11.12971,-14.49298 -11.12971,-14.49298 0,0 -4.90759,-17.04464 -0.63163,-18.51993 z"
       id="path3267"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 383.62267,283.20064 c 1.55115,4.32324 -14.57915,10.83903 -14.57915,10.83903 0,0 -16.74226,5.23871 -18.43376,0.92229 -1.88511,-4.81061 14.49297,-11.12968 14.49297,-11.12968 0,0 17.04463,-4.90763 18.51994,-0.63164 z"
       id="path3267-5"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
       sodipodi:type="arc"
       fill="none"
       style="stroke:#2b0000;stroke-width:1.78571427"
       id="cdp3"
       sodipodi:cx="337.14285"
       sodipodi:cy="-168.40552"
       sodipodi:rx="12.5"
       sodipodi:ry="12.5"
       d="m 349.64285,-168.40552 a 12.5,12.5 0 0 1 -12.5,12.5 12.5,12.5 0 0 1 -12.5,-12.5 12.5,12.5 0 0 1 12.5,-12.5 12.5,12.5 0 0 1 12.5,12.5 z"
       transform="matrix(1.6184,0,0,1.6184,-178.11234,492.03247)" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.82835,202.92226 c 4.3232,-1.55115 10.83903,14.57914 10.83903,14.57914 0,0 5.2387,16.74227 0.9223,18.43375 -4.81066,1.88513 -11.1297,-14.49297 -11.1297,-14.49297 0,0 -4.90759,-17.04465 -0.63163,-18.51992 z"
       id="path3267-1"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 383.859,213.67479 c 1.55117,4.32324 -14.57914,10.83903 -14.57914,10.83903 0,0 -16.74227,5.2387 -18.43375,0.92228 -1.88511,-4.8106 14.49298,-11.12967 14.49298,-11.12967 0,0 17.0446,-4.90763 18.51991,-0.63164 z"
       id="path3267-5-7"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
       sodipodi:type="arc"
       fill="none"
       style="stroke:#2b0000;stroke-width:1.78571427"
       id="cdp2"
       sodipodi:cx="337.14285"
       sodipodi:cy="-168.40552"
       sodipodi:rx="12.5"
       sodipodi:ry="12.5"
       d="m 349.64285,-168.40552 a 12.5,12.5 0 0 1 -12.5,12.5 12.5,12.5 0 0 1 -12.5,-12.5 12.5,12.5 0 0 1 12.5,-12.5 12.5,12.5 0 0 1 12.5,12.5 z"
       transform="matrix(1.6184,0,0,1.6184,-178.11234,634.84182)" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.82831,345.73161 c 4.32321,-1.55115 10.83903,14.57914 10.83903,14.57914 0,0 5.23871,16.74227 0.92232,18.43375 -4.81068,1.88511 -11.12971,-14.49297 -11.12971,-14.49297 0,0 -4.9076,-17.04465 -0.63164,-18.51992 z"
       id="path3267-1-1"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 383.85899,356.48414 c 1.55115,4.32324 -14.57917,10.83903 -14.57917,10.83903 0,0 -16.74224,5.2387 -18.43374,0.92228 -1.88512,-4.8106 14.49297,-11.12967 14.49297,-11.12967 0,0 17.04461,-4.90763 18.51994,-0.63164 z"
       id="path3267-5-7-5"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="ccscc" />
    <path
	fill = "#505050;"
       style="stroke:#000000;stroke-width:2.33817959px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -14.925188,445.76536 -44.55315,56.20914 44.55315,0 -44.55315,-56.20914 z"
       id="valve_7"		
       inkscape:connector-curvature="0" />
    <path
       onClick="toggleValve(0);"
       fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -126.87678,108.12915 87.216069,43.86607 0,-43.86607 -87.216069,43.86607 z"
       id="valve_0"
       inkscape:connector-curvature="0" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -41.237143,118 49.5710857,0 c 0,0 25.8035673,-4.12856 25.8035673,20.64286 0,24.77143 0,0 0,0 l 0,0 0,0 0,0"
       id="path4206"
       inkscape:connector-curvature="0" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m -41.237143,141.5 48.5389511,0 0,8.25716 26.8357019,0"
       id="path4214"
       inkscape:connector-curvature="0" />
    <rect
       style="fill:#3771cb;stroke:#3771cb"
       id="tank3_water"
       width="166"
       height="0"
       x="-119"
       y="167" />
    <rect
       style="fill:#3771cb;stroke:#3771cb"
       id="tank4_water"
       width="157"
       height="0"
       x="108"
       y="162"
       inkscape:label="#rect4218" />
    <path
	fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.41696525px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 363.34419,700.44418 60.20916,44.44347 0,-44.44347 -60.20916,44.44347 z"
       id="valve_4"
       inkscape:connector-curvature="0" />
    <path
	fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.40726948px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 363.34178,750.43743 59.70893,44.45697 0,-44.45697 -59.70893,44.45697 z"
       id="valve_5"
       inkscape:connector-curvature="0" />
    <path
	fill = "none"
       style="stroke:#000000;stroke-width:2.33817959px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 349.41943,385.98992 -44.55315,56.20914 44.55315,0 -44.55315,-56.20914 z"
       id="valve_9"
       inkscape:connector-curvature="0" />
    <path
	fill = "#ababab;"
       style="stroke:#000000;stroke-width:2.40726233px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 363.34219,800.72297 59.70811,44.45731 0,-44.45731 -59.70811,44.45731 z"
       id="valve_6"
       inkscape:connector-curvature="0" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 362.53664,733.59945 -23.62828,0 0,31.09107 23.62828,0"
       id="path3249-5"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 362.03159,782.0843 -23.62828,0 0,31.09107 23.62828,0"
       id="path3249-3"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:2.8900001px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 361.02149,584.61047 -23.62828,0 0,28.05893 23.62828,0"
       id="path3247-7"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cccc" />
    <path
       style="fill:none;stroke:#000000;stroke-width:3.31575823px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 313.65656,443.29772 0,83.25009"
       id="path3181-3"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cc" />
    <path
       style="fill:none;fill-rule:evenodd;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 313.63636,525.91266 -13.63636,0"
       id="path4691"
       inkscape:connector-curvature="0" />
    <path
       style="fill:none;fill-rule:evenodd;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 313.63636,555.20559 -13.13131,-0.50505"
       id="path4693"
       inkscape:connector-curvature="0" />
    <text
       xml:space="preserve"
       style="font-style:normal;font-weight:normal;font-size:40px;line-height:125%;font-family:sans-serif;letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="263.71722"
       y="554.11469"
       id="text4695"
       sodipodi:linespacing="125%"><tspan
         sodipodi:role="line"
         x="263.71722"
         y="554.11469"
         id="tspan4699">P</tspan></text>
  </g>
</svg>








<div id="csv_string_box">
CSV here
</div>






<div id="psi_gauge" style="width:300px; height:240px; float:left; position: absolute; display: block; top:585px; left:270px;"></div>
<input type="hidden" id="psi_gauge_val" value="0" />


				</div style="width:100%; border: 1px solid red;">
					<div id="svg_container" style="display:block; min-width:60%; min-height:665px; float:left; border:1px solid yrllow; position:absolute;">

					</div>
				</div>
		</td>
		<td>
			<div style="float:right; border: 0px solid blue;">
				<?php include_once('cm/cadi_dd.php'); ?>
			</div>
		</td>
	</tr>	
</table>
	=================================================	
	<br>

<br> 
	<div onClick="cadi_bt_scan();" style="display:inline; border: 1px solid red;">Scan</div>
	<select id="bind_mac" name="bind_mac">
	<option>Scan to get the list</option>
	</select>
	<button onClick="bt_connect();" style="display:inline; border: 1px solid red;">Connect</button>
	RFCOMM NUMBER:
	<input type="text" style="width: 40px;" value="0" id="rfcomm_nr" /><br>
	Binded RFCOMM list (<button onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">refresh</button>):
	<div id="binded_rfcomms">
		
	</div>
	<button class="btn_" onClick="btd_stream_status(1);">BTD status stream ON</button>
	<button class="btn_" onClick="btd_stream_status(0);">BTD status stream OFF</button>
<!--	<div title="use this field to send the data to Cadi while connected" id="tx_form">
		<input type="text" name="tx_data" id="tx_data" />
		<div onClick="bt_tx_packet()">Send</div>	
	</div> 


  	<div title="send ZX2[1],[1],[1]" id="tx_form">
		<input type="text" name="tx_data_packet" id="tx_data_packet" />
		<div onClick="bt_tx()">Send packet</div>	
	</div> -->

<br>

	=================================================	
	<br>
	<br>

<!--  <div onClick="bt_setdd()">Set DD</div>	

<div onClick="plugStateSet('1','1')">Enable P1</div>
<div onClick="plugStateSet('1','0')">Disable P1</div>  -->


<br>
==========================================

<br>


<br>
<button onClick="run_demo()">Run demo</button>

<br>

<button onClick="cw_reboot()">Reboot server</button><br>
<button onClick="cadi_reset()">Cadi reset</button><br>
====================================
<br>
<button onClick="cadiweb_update()">Cadiweb software update</button><br>
<input type="hidden" id="log_interval_id" value="0" />
<div>
	<textarea size="10" id="cadi_update_log"></textarea>
</div>
<button onClick=stop_log_stream();>Stop log stream</button>
====================================
<br>


Status stream delay<input type="text" id="status_stream_delay" value="800"/>
<br>


<input type="text" value="0" id="video_stream" title="this value is N in '/dev/videoN'" /><button onClick="change_video()">Change video</button>
</div>
</div>
</body>
</html>
