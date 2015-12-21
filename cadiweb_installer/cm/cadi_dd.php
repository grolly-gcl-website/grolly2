<script>
$(function() {
	$( ".btn_" ).button();
	$( ".radio_" ).buttonset();
});


// TO BE DELETED
function bt_setdd(state) {
	if (state==1) {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
			cadi_list_rfcomms();
		});
	}
	else {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '3'}, function(data){
				cadi_list_rfcomms();
		});
	}
}

function auto_flags(flags){	// set new auto_flags byte
	if (flags == 256) {
		flags = $('#auto_flags_input').val();
		// alert('Custom flags='+flags+' ('+flags.toString(2)+')');
	}
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 8, flags:flags}, function(data){
		$('#main_output').html(data);
	});
}

function open_valve(valve){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 4, valve:valve}, function(data){
		$('#main_output').html(data);
		//alert(data);
	});
}

function close_valve(valve){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 5, valve:valve}, function(data){
		$('#main_output').html(data);
	});
}
function reset_valve_fails(){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 10}, function(data){
		$('#main_output').html(data);
	});
}
function cadi_txcmd(cmd){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: cmd}, function(data){
		$('#main_output').html(data);
	});
}

function cadi_send_command(){
	var mac = $('#bind_mac').val();
	var command = $('#cadi_command').val();
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:command, mac:mac}, function(data){
		$('#main_output').html(data);
	});
}

function cadi_send_cmd(cmd){
	var mac = $('#bind_mac').val();
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:cmd, mac:mac}, function(data){
		$('#main_output').html(data);
	});
}

function cdd_toggle(){
	var state = $('#cdd_enabled').is(':checked');
	if (state==1) {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
			cadi_list_rfcomms();
			// alert(data);
		});  
	}
	else {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '3'}, function(data){
			cadi_list_rfcomms();
			// alert(data);
		});  
	}
}

function plugStateSet(plug, state){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data){
		cadi_list_rfcomms();
	});  
}



function close_valves(){
	alert('Closing ALL valves');
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '37'}, function(data){}); 
}

// Watering Task: background watering functions calls
function wt_watering(){
	var duration = $('#wt_duration').val();
	var line_id = $('#wt_line_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '102', duration:duration, line_id:line_id}, function(data){
		
	});

}


// Watering Task: Mix solution in MixTank
function wt_mixtank(){
	var duration = $('#wt_mixtank_duration').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '103', duration:duration}, function(data){
		
	});
}

function wt_mix_in(){
	var duration = $('#wt_mixin_duration').val();	// duration of aftermix process
	var volume = $('#wt_mixin_volume').val();	// how much fertilizer to add
	alert(duration+'/'+volume+'/'+'/');
	var doserId = $('#wt_mixin_doserid').val();	// doser id for mixing in the nutrition
	var speed = $('#wt_mixin_speed').val();	// speed for the doser (recommended for Welco: 40-100%)
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '104', duration:duration, volume:volume, doserid:doserId, speed:speed}, function(data){
		alert(data);
	});
}

// wt_mt_reach_level(uint16_t new_level, uint8_t source)
function wt_mt_reach_level(){
	var new_level = $('#wt_mtrl_newlevel').val();
	var src = 1;	// HARDCODE
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '105', new_level:new_level, src:src}, function(data){
		
	});
}

// wt_mt_add_water(uint16_t amount, uint8_t source)
function wt_mt_add_water(){
	var amount = $('#wt_mtaw_amount').val();
	var src = 1;
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '106', amount:amount, src:src}, function(data){
		
	});
}


// void wt_mt_drain2level(uint16_t new_level, uint8_t drain_valve)
function wt_mt_drain2level(){
	var new_level = $('#wt_dtl_newlevel').val();
	var drain_valve = $('#wt_dtl_drain_valve').val();
	var src = 1;	// HARDCODE
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '107', new_level:new_level, drain_valve:drain_valve}, function(data){
		
	});
}


</script>


<div id="cadi_dd_accordion" style="width:100%;">
	<div style="float:right; display:block; width:100%; right:0px;">	
<!---	<input type="checkbox" id="cdd_enabled" onClick="cdd_toggle(this)"> CDD: Cadi Direct Drive enabled  --->
	<table>
	<?php

	$row = 1;
	if (($handle = fopen("cm/cadi_settings", "r")) !== FALSE) {
	    while (($rowarr = fgetcsv($handle, 1000, ",")) !== FALSE) {
		if (substr($rowarr[0],0,1)!='#') {
			$_SESSION['settings_data'][($rowarr[0])] = $rowarr;
			$row++;
		}
	    }
	    fclose($handle);
	}


	$plug_amount = 4;

//	echo '<tr><td><b>LOADS:</b></td></tr>';
/*	for ($i=0; $i<$plug_amount; $i++) {
		echo '
		<tr title="'.$_SESSION['settings_data'][2][$i+1].'">
			<td style="text-align:right; width:15%;">'.$_SESSION['settings_data']['ac_loads'][$i+1].'</td>
			<td>
				<form class="c_inline">
				<div id="radio_plug1" class="radio_">
				<input type="radio" id="plug'.$i.'_radio1" name="plug'.$i.'_radio"><label for="plug'.$i.'_radio1" onClick="plugStateSet('.$i.',1)" >ON</label>
				<input type="radio" id="plug'.$i.'_radio2" name="plug'.$i.'_radio" checked="checked"><label for="plug'.$i.'_radio2" onClick="plugStateSet('.$i.',0)">OFF</label>
				</div>
				</form>
			</td>
			<td></td>
		</tr>';
	}	*/
	?>

<!---	</table>

	<table>   --->
	<?php
	$valves_amount = 14;

	
//	echo '<tr><b>VALVES:</b></tr>';


// display only ordered valves
	$valve_order = $_SESSION['settings_data']['display_valves'];
	for ($i=1;$i<sizeof($valve_order);$i++) {
		echo '
		<tr title="'.$_SESSION['settings_data']['valves_hints'][$i+1].'">
			<td style="text-align:right; width:15%;">'.$valve_order[$i].' '.$_SESSION['settings_data']['valves'][($valve_order[$i]+1)].'</td>
			<td>
				<form class="c_inline">
				<div id="radio_plug1" class="radio_">
				<input type="radio" id="valve'.$valve_order[$i].'_radio1" name="valve'.$valve_order[$i].'_radio"><label for="valve'.$valve_order[$i].'_radio1" onClick="open_valve('.$valve_order[$i].')">Open</label>
				<input type="radio" id="valve'.$valve_order[$i].'_radio2" name="valve'.$valve_order[$i].'_radio" checked="checked"><label for="valve'.$valve_order[$i].'_radio2" onClick="close_valve('.$valve_order[$i].')">Close</label>
				</div>
				</form>
			</td>
			<td></td>
		</tr>';
	}

/*
 // display all the valves
	for ($i=0; $i<$valves_amount; $i++) {
		echo '
		<tr class="valves_all" title="'.$_SESSION['settings_data']['valves_hints'][$i+1].'">
			<td style="text-align:right; width:15%;">'.$_SESSION['settings_data']['valves'][$i+1].'</td>
			<td>
				<form class="c_inline">
				<div id="radio_plug1" class="radio_">
				<input type="radio" id="valve'.$i.'_radio1" name="valve'.$i.'_radio"><label for="valve'.$i.'_radio1" onClick="open_valve('.$i.')">Open</label>
				<input type="radio" id="valve'.$i.'_radio2" name="valve'.$i.'_radio" checked="checked"><label for="valve'.$i.'_radio2" onClick="close_valve('.$i.')">Close</label>
				</div>
				</form>
			</td>
			<td></td>
		</tr>';
	} 
	
*/

	?>
	<tr><td><button onClick="close_valves()">Close valves</button></td></tr>
	
	<tr><td>
		<br>

		...REACH WATER LEVEL...
		<br>
		New level (mms):<input type="text" id="wt_mtrl_newlevel" /><br>
		<button onClick="wt_mt_reach_level()">Reach new level</button>
		<br>
		<br>


		...ADD WATER...
		<br>
		Amount (mms):<input type="text" id="wt_mtaw_amount" /><br>
		<button onClick="wt_mt_add_water()">Add water</button>
		<br>
		<br>

		
		... DRAIN TO THE LEVEL...
		<br>
		New level (mms):<input type="text" id="wt_dtl_newlevel" /><br>
		<select id="wt_dtl_drain_valve">
		<?php 
			for ($i=0; $i<$valves_amount; $i++) {
				echo '<option value="'.$i.'">'.$i.' - '.($_SESSION['settings_data']['valves'][$i+1]).'</option>';
			}
		?>
		</select>
		<br>
		<button onClick="wt_mt_drain2level()">Drain</button>
		<br>
		<br>

		<br>
		...WATERING TASK...
		<br>
		Line id:
		<select id="wt_line_id">
		<?php 
			for ($i=0; $i<$valves_amount; $i++) {
				echo '<option value="'.$i.'">'.$i.' - '.($_SESSION['settings_data']['valves'][$i+1]).'</option>';
			}
		?>
		</select>
		<br>
		Duration (seconds):	<input type="text" id="wt_duration" /><br>
		<button onClick="wt_watering()">Run watering task</button>
		<br>

		<br>
		...MIXTANK TASK...
		<br>
		Duration (seconds):<input type="text" id="wt_mixtank_duration" /><br>
		<button onClick="wt_mixtank()">Mix Tank</button>
		<br>
		<br>

		... MIX-IN TASK...
		<br>

		Duration (seconds):<input type="text" id="wt_mixin_duration" /><br>
		Volume (seconds):<input type="text" id="wt_mixin_volume" /><br>
		doser:
		<?php
			echo '<select id="wt_mixin_doserid">';
			$dosers_amount = 4;
			for ($i=0; $i<$dosers_amount; $i++) {
				echo '
					<option value="'.$i.'" title="'.$_SESSION['settings_data']['dosers_hints'][$i+1].'">'.$i.': '.$_SESSION['settings_data']['dosing_pumps'][$i+1].'</option>';
			}
			echo '</select>';
			for ($i=0; $i<$dosers_amount; $i++) {
				echo '<input type="hidden" id="doser_speed_'.$i.'" value="'.$_SESSION['settings_data'][7][$i+1].'" />';
			}
		?>
		<br>
		Speed (40-100):<input type="text" id="wt_mixin_speed" value="50" /><br>
		<button onClick="wt_mix_in()">Mix-IN</button>

		<br>
		<br>





	</td></tr>

	<tr><td colspan="2">
		<button class="btn_" onClick="reset_valve_fails()">Fails=0</button>
		<button class="btn_" onClick="auto_flags(0)">Auto=0</button><br>
		<button class="btn_" onClick="auto_flags(255)">Auto=255</button>
		<button class="btn_" onClick="cadi_txcmd(11)">Force DOWN</button><br>
		<button class="btn_" onClick="cadi_txcmd(12)">Set time</button>
		<br>

		<input type="text" value="254" id="auto_flags_input" />
		<button onClick=auto_flags(256)>Set auto flags</button>

	</td></tr>
	<tr><td></td></tr>
	<tr><td></td></tr>

	</table>

	</div>
</div>

=====================
