<?php include('gen_fcm.php');  ?>

<script>

$('.dtpckr1').datetimepicker({ dateFormat: 'yy-mm-dd', 
				timeFormat: 'hh:mm:ss',
				changeYear: true
});


$('.dtpckr1').datetimepicker("option", {
  onClose: function(dateText, inst){
		
		var curut = $(this).datepicker('getDate') / 1000;
		var date = new Date(curut*1000);
		// hours part from the timestamp
		if (date.getHours()>9) {
			var hours = date.getHours();
		}
		else {
			var hours = "0" + date.getHours();
		}

		// minutes part from the timestamp
		if (date.getMinutes()>9) {
			var minutes = date.getMinutes();
		}
		else {
			var minutes = "0" + date.getMinutes();
		}

		if (date.getSeconds()>9) {
			var seconds = date.getSeconds();
		}
		else {
			var seconds = "0" + date.getSeconds();
		}

		if (date.getDate()>9) {
			var day = date.getDate();
		}
		else {
			var day = "0" + date.getDate();
		}

		if (date.getMonth()>9) {
			var month = date.getMonth()+1;
		}
		else {
			var month = "0" + (date.getMonth()+1);
		}

		var year = date.getFullYear();


		$(this).datepicker('option', 'dateFormat', 'yy-mm-dd');

		var namearr = this.id.split('_');
		addr = namearr[1];
		$( "input[name='csx_"+addr+"_value']" ).val(curut);
		$( "input[name='csx_"+addr+"_value']" ).change();
	}
});



$(function() {
	$( ".btn_cadi_timer_enabled" ).button();
	$( "#cadi_watering_accordion" ).accordion();
});

function submitWateringData(){
	$("#your_datepicker").datetimepicker("getDate").getTime() / 1000
}

function get_settings(){	// read 32 bit STM32 flash value
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:58, addr:addr}, function(data){
		//	alert('sent settings request');
	});
}

function get_settings_block(){	// read 32 bit STM32 flash value
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:59, addr:addr}, function(data){
		//	alert('sent settings request');
	});
}

// 16 bit
function save_settings(){
	var addr = $('#settings_block_id').val();
	var value = $('#settings_value').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:value}, function(data){
		alert('settings saved');
	});
}

function rx_ee_val(addr, type){
	if (type == 10) {	// 8 bit higher
		var value = $('#'+addr+'_value').val();
		value = Math.floor(value/256);	// get higher 8 bit
		$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:17, addr:addr, value:value}, function(data){
			alert('settings bit higher saved');
		});
	}
	if (type == 11) {	// 8 bit lower
		var value = $('#'+addr+'_value').val();
		value %= 256;	// get lower 8 bit
		$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:18, addr:addr, value:value}, function(data){
			alert('settings bit higher saved');
		});
	}
	if (type == 2) {	// 16 bit
		var value = $('#'+addr+'_value').val();
		value %= 65536;	// remove not needed
		$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:value}, function(data){
			alert('settings bit higher saved');
		});
	}
	if (type == 4) {	// 32 bit
		var value = $('#'+addr+'_value').val();
		var tmp = value % 65536;
		value = Math.floor(value/65536);
		$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:value}, function(data){
			addr++;	// next 16bit cell
			$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:tmp}, function(data){
			
			});
		});
	}
	var value = $('#'+addr+'_value').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:value}, function(data){
		alert('settings saved');
	});
}




function save_settings_block(){
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:16, addr:addr}, function(data){
		alert('sent block of settings to Cadi');
	});
}

function apply_settings(){
	var addr = $('#settings_block_id').val();
	var value = $('#settings_value').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:19}, function(data){});
}

function update_block(addr){
	var arr = '';
	var nval = 0;
	$("[id^=addr_"+addr+"]").each(function( index ) {
		// arr += ' / ';
		if ($( this ).is(':checked')==true) {	// if it is checked
			elid = $( this ).attr('id');	// get its element id
			elidarr = elid.split('_');	// get value addr and bit id
			bit_id = elidarr[3];		// bit value
			nval |= (1<<(bit_id-1));
			//alert(index+' checked / bit_id='+bit_id+' / nval='+nval);
		}
	});
	var oval = $('#val_'+addr).val();	// old value
	//alert(oval+' was nad became '+nval+' at addr='+addr);
	if (nval!=oval) {			// if old and new values does not match, update old one
		rx_ee_dir(addr, nval);
		// alert('Update to '+nval+' (was '+oval+')');
	}
}

function link_fmp(wp_id){
	var fmp_id = $('#wp_'+wp_id+'_link_selector').val();
	alert(fmp_id);
}


</script>
<div id="cadi_watering_accordion">
<?php
	$wp_amount = 8;
	for ($n=1; $n<=$wp_amount;$n++) {
		get_wp_block($n);
	}
?>


</div>

