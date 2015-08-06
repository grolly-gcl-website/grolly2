<!-- 
Cadiweb settings and preferences set here

-->

<?php

/*
Cadi settings schema contains all data needed to display user settings memory dump in web-element

cadi_settings_schema.csv
[field] | [description]

address	|	user setting record virtual address in STM32 Emulated EEPROM
type		|	setting's variable size ("2" for 16 bit, "4" - for 32 bits)
description|	text description for this memory field (no commas in this field!!!)
value	|	setting's variable value

*/

function print_settings_schema(){
	$row = 1;
	// cadi_settings_schema.csv contains settings & descriptions for settings memory fields
	if (($handle = fopen("cadi_settings_schema.csv", "r")) !== FALSE) {
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		echo '<tr>
				<td>'.$row.'</td>	
				<td>'.$data[0].'</td>	
				<td>'.$data[1].'</td>
				<td>'.$data[2].'</td>	
				<td>'.$data[3].'</td>	
			</tr>';
		$_SESSION['schema_data'][$row++] = $data;
	    }
	    fclose($handle);
	}
}

// requests and then reads settings memory array from connected Cadi device into binary file on disk
function dl_cadi_settings(){

}

// uploads the local settings binary to connected Cadi device
function ul_cadi_settings(){

}

// converts current schema into Cadi settings binary
function schema2csbin(){
	for ($i=1;$i<sizeof($_SESSION['schema_data']);$i++) {
		// if 16 bit variable met
		if ($_SESSION['schema_data'][$i][1]==2) {
			
		}
		// if 32 bit variable met
		if ($_SESSION['schema_data'][$i][1]==4) {
			
		}
	}

}

// extract Cadi settings binary into schema
// existing schema is being updated and "gaps" are filled with <untagged> descriptor for 16 bit variable type
function csbin2schema(){
	if (($handle = fopen($_SESSION['csbin_filename'], "r")) !== FALSE) {
		$csbin = fgetcsv($handle, 100000, ",")
		for ($i=0;$i<sizeof($csbin);$i++) {
		
		}
	}
}







?>

<script>
function apply_btd_settings(){
	var arr = [$("#cadi_set_pfd").val(),
			$("#cadi_set_csd").val(),
			$("#cadi_set_fsd").val(),
			$("#cadi_set_rpd").val(),
			$("#cadi_set_cn").val(),
			$("#cadi_set_srtrs").val(),
			$("#cadi_set_sppd").val(),
			$("#cadi_set_setfs").val(),
			$("#cadi_set_setsa").val()
];
	settings = arr.join();
//	alert(arr);
	$.post('cm/cadi_bt_processor.php', {action: 'btd_apply_settings', settings:settings}, function(data){
		 alert(data);
	});

	var arr = [
			$("#cadi_set_svg_t3top").val(),
			$("#cadi_set_svg_t3btm").val(),
			$("#cadi_set_svg_t3wx").val(),
			$("#cadi_set_svg_t3wy").val(),
			$("#cadi_set_svg_t4top").val(),
			$("#cadi_set_svg_t4btm").val(),
			$("#cadi_set_svg_t4wx").val(),
			$("#cadi_set_svg_t4wy").val()];
	settings = arr.join();
//	alert(arr);
	$.post('cm/cadi_bt_processor.php', {action: 'svg_apply_settings', settings:settings}, function(data){
		 alert(data);
	});

}

</script>

<?php
	$temp = file_get_contents('btds/btd.conf');
	$temparr = explode(",", $temp);
	$temp = file_get_contents('svg.conf');
	$svg_arr = explode(",", $temp);
?>

<b>Cadiweb preferences tuning</b>
<form action="cm/cadiweb_settings.php" method="POST">
<input type="hidden" id="cmd" value="save_settings" />
<ul>
	<li>Photoshot freq divider, N
		<input
			id="cadi_set_pfd"
			title="make a photoshot every N iteration of Command Scan Cycle (CSC)"
			type="text"
			value="<?php echo $temparr[0]; ?>" />
	</li>
	<li>Command scan delay, us
		<input
			id="cadi_set_csd"
			title="25000=25ms~=40scans per second"
			type="text"
			value="<?php echo $temparr[1]; ?>" />
	</li>
	<li>File size difference, X
		<input
			id="cadi_set_fsd"
			title="attmpt to parse response if filesize increased at least for X bytes"
			type="text"
			value="<?php echo $temparr[2]; ?>" />
	</li>
	<li>Response parser divider, N
		<input
			id="cadi_set_rpd"
			title="attmpt to parse response every N-th CSC"
			type="text"
			value="<?php echo $temparr[3]; ?>" />
	</li>
	<li>/dev/videoX camera number
		<input
			id="cadi_set_cn"
			title="attmpt to parse response every N-th CSC"
			type="text"
			value="<?php echo $temparr[4]; ?>" />
	</li>
	<li>Serial response tail read size
		<input
			id="cadi_set_srtrs"
			title="Data amount to be read from the end "
			type="text"
			value="<?php echo $temparr[5]; ?>" />
	</li>
	<li>Status packet ping divider, N
		<input
			id="cadi_set_sppd"
			title="Send status request packet to Cadi every N iteration of daemon main loop"
			type="text"
			value="<?php echo $temparr[6]; ?>" />
	</li>
	<li>Settings dump file size, bytes
		<input
			id="cadi_set_setfs"
			title="Daemon prelocates settings dump file to this size"
			type="text"
			value="<?php echo $temparr[7]; ?>" />

	</li>
	<li>Settings dump start adress (within STM32 memory) offset, bytes
		<input
			id="cadi_set_setsa"
			title="First byte in settings dump file corresponds to this address in Cadi EEPROM memory"
			type="text"
			value="<?php echo $temparr[8]; ?>" />

	</li>
</ul>



<b>SVG variables</b>

<ul>
	<li>Верхний уровень бака (3), см
		<input
			id="cadi_set_svg_t3top"
			title="The tank 3 water top level, cm / Верхний уровень бака (3), см"
			type="text"
			value="<?php echo $svg_arr[0]; ?>" />
	</li>
	<li>Дно бака (3), см
		<input
			id="cadi_set_svg_t3btm"
			title="Tank 3 water bottom level. cm / Нижний уровень бака (3), см"
			type="text"
			value="<?php echo $svg_arr[1]; ?>" />
	</li>
	<li>Расположение "воды" (бак (3)) на схеме, координата X
		<input
			id="cadi_set_svg_t3wx"
			title="SVG 'water' position on Map view for Tank (3), X axis"
			type="text"
			value="<?php echo $svg_arr[2]; ?>" />
	</li>
	<li>Расположение "воды" (бак (3)) на схеме, координата Y
		<input
			id="cadi_set_svg_t3wy"
			title="SVG 'water' position on Map view for Tank (3), Y axis"
			type="text"
			value="<?php echo $svg_arr[3]; ?>" />
	</li>
	<li>Верхний уровень бака (4), см
		<input
			id="cadi_set_svg_t4top"
			title="The tank 4 water top level, cm"
			type="text"
			value="<?php echo $svg_arr[4]; ?>" />
	</li>
	<li>Дно бака (4), см
		<input
			id="cadi_set_svg_t4btm"
			title="Tank 4 water bottom level. cm / Нижний уровень бака (4), см"
			type="text"
			value="<?php echo $svg_arr[5]; ?>" />
	</li>
	<li>Расположение "воды" (бак (4)) на схеме, координата X
		<input
			id="cadi_set_svg_t4wx"
			title="SVG 'water' position on Map view for Tank (4), X axis"
			type="text"
			value="<?php echo $svg_arr[6]; ?>" />
	</li>
	<li>Расположение "воды" (бак (4)) на схеме, координата Y
		<input
			id="cadi_set_svg_t4wy"
			title="SVG 'water' position on Map view for Tank (4), Y axis"
			type="text"
			value="<?php echo $svg_arr[7]; ?>" />
	</li>
</ul>



<button class="btn_" onClick="apply_btd_settings()">Apply new settings</button>
