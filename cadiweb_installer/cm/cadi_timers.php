<?php
	session_start();
	include_once('cadi_bt_processor.php');
	$stngs = $_SESSION['cadi_settings'];
?>

<script>
$(function() {
	$( ".btn_cadi_timer_enabled" ).button();
	$( "#cadi_timers_accordion" ).accordion();
});
</script>
<div id="cadi_timers_accordion">
<h3>Daily timers</h3>
<div>
<table>
<?php
	display_timers();

?>
</table>
</div>
<h3>Cyclic timers</h3>
<div>
<table>
<?php
	

?>
</table>
</div>

</div>

