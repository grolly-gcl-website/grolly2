
<?php 
session_start();
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
	$minval = $_SESSION["settings_data"]["watertank_minmax_levels"][1];
	$maxval = $_SESSION["settings_data"]["watertank_minmax_levels"][2];


?>

<script script type="text/javascript">





$(function() {
	$( "#cadi_sensors_accordion" ).accordion();
});



$(function() {

<?php
	echo 'var maxval = '.$maxval.';'.PHP_EOL;
	echo 'var minval = '.$minval.';'.PHP_EOL;

?>
 	  
 	
	$( "#t3top_sldr, #t3btm_sldr, #t4top_sldr, #t4btm_sldr" ).slider({
	orientation: "horizontal",
	range: "min",
	max: minval,
	value: 0,
	slide: refreshSliders,
	change: refreshSliders
	});
//	
	$('#t3btm_sldr').slider('value', t3btm);
	$('#t3top_sldr').slider('value', t3top);

	$('#t4btm_sldr').slider('value', t4btm);
	$('#t4top_sldr').slider('value', t4top);
});

function refreshSliders(){
	var t3top = $('#t3top_sldr').slider('value');
	var t3btm = $('#t3btm_sldr').slider('value');
	var t4top = $('#t4top_sldr').slider('value');
	var t4btm = $('#t4btm_sldr').slider('value');
	
	t3prcnt = 100-((t3btm-t3curlvl)/((t3btm - t3top)/100));
	t4prcnt = 100-((t4btm-t4curlvl)/((t4btm - t4top)/100));
	if (t3prcnt<0) {
		t3prcnt = 0;
	}
	if (t3prcnt>100) {
		t3prcnt = 100;
	}
	if (t4prcnt<0) {
		t4prcnt = 0;
	}
	if (t4prcnt>100) {
		t4prcnt = 100;
	}

	$('#t3watersetup').css('height',(t3prcnt+'%'));
	$('#t4watersetup').css('height',(t4prcnt+'%'));

	$('#t3curlvl').html('Current level: '+t3curlvl);
	$('#t4curlvl').html('Current level: '+t4curlvl);
	$('#t3btm_val').html('Bottom: '+t3btm);
	$('#t3top_val').html('Top: '+t3top);
	$('#t4btm_val').html('Bottom: '+t4btm);
	$('#t4top_val').html('Top: '+t4top);
}

function save_tank_settings(){
	var t3top = $('#t3top_sldr').slider('value');
	var t3btm = $('#t3btm_sldr').slider('value');
	var t4top = $('#t4top_sldr').slider('value');
	var t4btm = $('#t4btm_sldr').slider('value');
	$.post('cm/cadi_settings_processor.php', {action: 'tank_settings', t3top:t3top, t3btm:t3btm, t4top:t4top, t4btm:t4btm}, function(data){
		alert('Saved!');
	});
}

</script>



<div id="cadi_sensors_accordion">
<h3>Water Tank Levels</h3>
<div>
	<div id="outer" style="width:100%; text-align:center;">
		<div style="display:block; border:3px solid blue; width: 30%; float:left; background-color:#0099ff; position: relative;">
			<div style="height:250px; display:block;">
			<div id="t3watersetup" style="float:left; width:100%; height:70%; background:#FFFFFF; position: absolute; opacity:0.5; border-bottom:3px solid blue;"></div>
				<div id="t3top_sldr"></div>
				<div style="float:left; position: absolute;" id="t3top_val">Top: </div>
				<br><br>
				Fresh Water Tank level setup
				<br>
				<div style="float:left; position: absolute;" id="t3curlvl">Current level: </div>
			</div>
			<div id="t3btm_val">Bottom: </div>
			<div id="t3btm_sldr"></div>
		</div>

		<br>

		<button style="display:inline-block" onClick="save_tank_settings();">Save tank settings</button>
	
	
		<div style="border:3px solid green; width: 30%; float:right;  background-color:#00cc00; position: relative;"">
			<div style="height:250px; display:block;">
			<div id="t4watersetup" style="float:left; width:100%; height:70%; background:#FFFFFF; position: absolute; opacity:0.5; border-bottom:3px solid green;"></div>
				<div id="t4top_sldr"></div>
				<div style="float:left; position: absolute;" id="t4top_val">Top: </div>
				<br><br>
				Mixing Tank level setup
				<br>
				<div style="float:left; position: absolute;" id="t4curlvl">Current level: </div>
			</div>

			<div id="t4btm_val">Bottom: </div>
			<div id="t4btm_sldr"></div>
	
		</div>

	</div>
</div>

<h3>Water Pressure Sensor</h3>
<div>
	<table>
		<tr>
			<td colspan="2" style="text-align:center;">
				Water pressure sensor levels
			</td>
		</tr>
		<tr>
			<td>Top level</td>
			<td>
				<input 
					onChange="rx_ee_(this)" 
					type="text" 
					value="" 
					name="" 
				/>
			</td>
		</tr>
		<tr>
			<td>Bottom level</td>
			<td>
				<input 
					onChange="rx_ee_(this)" 
					type="text" 
					value="" 
					name="" 
				/>
			</td>
		</tr>
	</table>

</div>

<h3>pH level meter</h3>
<div>
	<table>
		<tr>
			<td colspan="2" style="text-align:center;">
				pH sensor calibration
			</td>
		</tr>
		<tr>
			<td>Current ADC value&nbsp;</td>
			<td>
				<div id="sens_ph1_adc_val"></div>
			</td>
		</tr>
		<tr>
			<td>pH4</td>
			<td>
				<input 
					onChange="rx_ee_(this)" 
					type="text" 
					value="" 
					name="" 
				/>
			</td>
		</tr>
		<tr>
			<td>pH7</td>
			<td>
				<input 
					onChange="rx_ee_(this)" 
					type="text" 
					value="" 
					name="" 
				/>
			</td>
		</tr>
	</table>

</div>
