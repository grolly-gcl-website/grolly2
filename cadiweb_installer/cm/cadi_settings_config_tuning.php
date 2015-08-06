<!-- 
This block gives user ability to tune the Cadi Bluetooth Daemon settings

-->

<script>
$(document).ready(function() {
	$('#csxform').submit(function (e) {
   	  $('#csxform').click();
	     e.preventDefault();
	});
});

function upload_csx(){
	var csx_data = $('#csxform').serialize();
	$.post('cm/cadi_bt_processor.php', {action: 'upload_csx', csx_data:csx_data}, function(data){
		
	});
}





</script>

<br><br>
<input type="hidden" value="0" id="csxdl_interval" />


<b>=== Cadi EEPROM Settings Tuning ===</b>

<button onClick="download_csx(0)">Download settings from Cadi</button>

<form id="csxform" name="csxform" action="">

<?php include('csx_get_table.php'); ?>


</form>
<br><br><br>


