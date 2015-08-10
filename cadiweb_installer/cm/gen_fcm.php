<?php 

include_once('cadi_settings.php');
// sync_dump2conf();


/*
	$row = 0;
	if (($handle = fopen("cadi_settings", "r")) !== FALSE) {
	    while (($_SESSION['settings_data'][$row] = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$num = count($_SESSION['settings_data'][$row]);
		$row++;
	    }
	    fclose($handle);
	}

*/


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


$wp_block_size = 15;


/*
The blocks of flags are generated based on "Views"
View - is a CSV string, describing the uC EEPROM address bit representation.
Any View has an unique ID called block_id.
There could be multiple Views for one EEPROM value, discovering different
EEPROM value's bits to access.
*/

function gen_flags_block($block_id){
	global $_SESSION;
	global $wp_block_size;
	if ($block_id>100 && $block_id<200) {		// WP_FLAGs
		$strct = '16,';
		$strct .= (1575+$wp_block_size*($block_id-101));
		
		$strct .= '(addr),';
		// $wlines = explode(',',$_SESSION['settings_data']['output_valves']);
		// generim stroku vida:
		// 	',,Line 1, Line 2, Line 3,,Line 7, Line 4, Line X,, Holo'
		// dlja peredachi v generator blokov flagov
		for ($i=1;$i<sizeof($_SESSION['settings_data']['valves']);$i++) {

			if (strlen($_SESSION['settings_data']['valves'][$i])>1) {
			// dobavit' kusochek stroki vida ",valve_id valve_name"
			$vlvid = str_pad(($i-1), 2, "0", STR_PAD_LEFT);
			$strct .= ','.$vlvid.' '.$_SESSION['settings_data']['valves'][$i];

			}
			else {
				$strct .= ',';
			}
		}
		// $strct .= $_SESSION['settings_data']['output_valves'];
	//	$strct .= ',,Line 1, Line 2, Line 3,,Line 7, Line 4, Line X,, Holo';
		gen_flags($strct);
	}
	if ($block_id>1100 && $block_id<1200) {		// TIMERS

	}

	if ($block_id>2100 && $block_id<2200) {		// CYCLIC TIMERS

	}
}


function gen_flags($strct){
	$arr = explode(',', $strct);
	$type = $arr[0];	// val type: 8, 16 or 32 bit
	$addr = (int)$arr[1];	// val addr
	echo '<tr><td>';
	$tmparr = explode(',', get_settings_val($addr));
	echo '</tr></td>';
	$value = $tmparr[1];
	switch ($type) {
		case '1':	// 8 bit
			$valsize = 8;
			break;
		case '2':	// 16 bit
			$valsize = 16;
			break;
		case '3':	// 32 bit
			$valsize = 32;
			break;

	}
	$valsize = 16;		// HARDCODE
	$binstr = str_pad(decbin($value), $valsize, '0', STR_PAD_LEFT);	// get full 16bit representation of 2 bytes
	$flags_amount = strlen($binstr);
	echo '<input type="hidden" id="val_'.$addr.'" value="'.$value.'" />';
	for ($i=0; $i<($flags_amount); $i++) {
		if (strlen($arr[($i+2)])>0) {	// do not display empty named lines
			echo '
				<tr>
					<td><input type="checkbox"';
			if (($binstr[($flags_amount-$i)])==1) {	// if on the 
				echo 'checked';
			}
			echo 		' name="" id="addr_'.$addr.'_bit_'.($i).'" /></td>
					<td>'.$arr[($i+2)].'</td>
				</tr>
			';
		}
	} 
}



/*
$csx comes as an array like:
    [csx_14050_type] => 1
    [csx_14050_value] => 0
    [csx_14050_text] => 8 bit test
    [csx_14051_type] => 1
    [csx_14051_value] => 0
    [csx_14051_text] => 8bit test2
    [csx_1410_type] => 1
    [csx_1410_value] => 0
    [csx_1410_text] => 32bit test
while $sca is an array like
	$sca[n] = 
		[0] = $addr
		[1] = $type
		[2] = $value
		[3] = $description_string
	
*/

// Extract encoded Cadi Settings into PHP array

function csx2sca($csx){
	$curaddr_arr = explode("_", $csx[0]);	// get first csx line address to
	$curaddr = $curaddr_arr[1];		// start foreach properly filling $sca
	$ndx = 0;
	foreach ($csx as $key=>$value) {
		unset($tmparr);
		$tmparr = explode("_", $key);
		$rowaddr = $tmparr[1];
		$subkey = $tmparr[2];
		if ($curaddr != $rowaddr) {	// when new address detected, fill sca row and increase index
			$curaddr = $rowaddr;
			$sca[$ndx][0] = $outarr['addr'];
			if ($outarr['type']>0) {
				$sca[$ndx][1] = $outarr['type'];
			}
			$sca[$ndx][2] = $outarr['value'];
			if (strlen($outarr['text'])>1) {
				$sca[$ndx][3] = $outarr['text'];
			}
			$ndx++;
		}
		$outarr[$subkey] = $value;	// temporary
		$outarr['addr'] = $rowaddr;	// array
	}
	
	return $sca;
}

function get_fert_selector($fert_id){		// fill selector with fertilizer options
	$dosers_amount = 4;
	for ($i=1; $i<$dosers_amount; $i++) {		// skip id=0, if its Mixing Pump
		echo '<option value="'.$i.'"';
		if ($i == $fert_id) {
			echo ' selected';
		}
		echo ' title="'.$_SESSION['settings_data']['dosing_pumps'][$i+1].'">'.$i.': '.$_SESSION['settings_data']['dosers_hints'][$i+1].'</option>';
	}
	echo '</select>';
}

$fmp_amount = 9;

// Get FMPs linked to Watering Program $wp_id
function get_wp_ferts_array($wp_id){
	global $fmp_amount;
	$fmp_size = 5;
	$fmp2wp_link_offset = 1701;	// FMP1 link to WP
	
	for ($i=0;$i<$fmp_amount;$i++) {		// create address table for all the linked FMPs' contents
		$fmpstart_offset = $fmp2wp_link_offset+$i*$fmp_size;
		$fmpaddrs[$fmpstart_offset] = null;	// doser id
		$fmpaddrs[$fmpstart_offset+1] = null;	// dosing time
		$fmpaddrs[$fmpstart_offset+3] = null;	// aftermix time
		$subaddr1 = ($fmpstart_offset+2).'1';	// ... and ...
		$fmpaddrs[$subaddr1] = null;		// N
		$subaddr1 = ($fmpstart_offset+2).'0';	// ... and ...
		$fmpaddrs[$subaddr1] = null;		// link
	}
	$sca_fmps = get_settings_arr($fmpaddrs);	// get linked FMPs' contents
	$m=0;
	$n=0;
	unset($fmp_link_selector);
	unset($fmp);
	for ($i=0;$i<$fmp_amount;$i++) {		// rebuild array with FMP contents
		unset($line);
		$fmpstart_offset = $fmp2wp_link_offset+$i*$fmp_size;
		$line['addr'] = $fmpstart_offset;
		$line['id'] = $i+1;
		$line['doser_id'] = $sca_fmps[$fmpstart_offset][2];	// doser id
		$line['dosing_time'] = $sca_fmps[($fmpstart_offset+1)][2];	// dosing time
		$line['aftermix_time'] = $sca_fmps[($fmpstart_offset+3)][2];	// aftermix time
		$subaddr1 = ($fmpstart_offset+2).'1';
		$line['n'] = $sca_fmps[$subaddr1][2];	// N
		$subaddr2 = ($fmpstart_offset+2).'0';
		$line['link'] = $sca_fmps[$subaddr2][2];	// fmp2wp link
		if ($wp_id==$line['link']) {
			$fmp[$m++] = $line;
		}
		else {
			$fmp_link_selector[$n++] = $line;
		}
	}
	$out['selector'] = $fmp_link_selector;
	$out['fmp'] = $fmp;
	return $out;
}

function get_fmp_selector($args=null){	// prints FMP selector for linking FMPs to WPs
	global $fmp_amount;
	
	for ($i=0;$i<$fmp_amount;$i++) {
		echo '<option value="">
			FMP#'.($i+1).'

		</option>';
	}

}


function get_wp_block($wp_id){
	$wp_start_offset = 1565;	// Watering programs Emulated EEPROM memory start offset
	$wp_size = 15;			// Watering Program settings block size
	$offset_wp_timeout = 0;		// watering timeout
	$offset_wp_volume = 1;		// Rules (higher) and Volume (lower byte)
	$offset_wp_interval = 2;	// WP run interval
	$offset_wp_start = 4;		// General timer for WP: start
	$offset_wp_end = 6;		// General timer for WP: finish
	$offset_wp_prev_run = 8;	// WP's previous run unix time stamp
	$offset_wp_flags = 10;		// WP flags

	$timers_offset = 1498;

	$wp_id--;			// to start from 0
	$addr_wp_volume = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_volume).'1';
	$addr_wp_interval = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_interval);
	$addr_wp_start = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_start);
	$addr_wp_end = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_end);
	$addr_wp_prev_run = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_prev_run);
	$addr_wp_flags = $wp_start_offset + $offset_wp_flags + $wp_size*$wp_id;
	$addr_wp_timeout = $wp_start_offset + $offset_wp_timeout + $wp_size*$wp_id;
	$addr_wp_timer_rules = ($wp_start_offset+$wp_size*$wp_id+$offset_wp_volume).'0';

	$wp_addrs[$addr_wp_timeout] = null;
	$wp_addrs[$addr_wp_volume] = null;
	$wp_addrs[$addr_wp_interval] = null;
	$wp_addrs[$addr_wp_start] = null;
	$wp_addrs[$addr_wp_end] = null;
	$wp_addrs[$addr_wp_prev_run] = null;
	$wp_addrs[$addr_wp_timer_rules] = null;
	
	$wp_sca = get_settings_arr($wp_addrs);
	$wp_timeout = $wp_sca[$addr_wp_timeout][2];
	$wp_volume = $wp_sca[$addr_wp_volume][2];
	$wp_timer_rules = $wp_sca[$addr_wp_timer_rules][2];
	$wp_interval = $wp_sca[$addr_wp_interval][2];
	$wp_start = $wp_sca[$addr_wp_start][2];
	$wp_end = $wp_sca[$addr_wp_end][2];
	$wp_prev_run = $wp_sca[$addr_wp_prev_run][2];
	$wp_start_formatted = date("Y-m-d H:i:s",$wp_sca[$addr_wp_start][2]);
	$wp_end_formatted = date("Y-m-d H:i:s",$wp_sca[$addr_wp_end][2]);
	$wp_prev_run_formatted = date("Y-m-d H:i:s",$wp_sca[$addr_wp_prev_run][2]);

	// process timers, applied in Rules(Volume)
	$timer_rules = str_pad(decbin($wp_timer_rules), 8, "0", STR_PAD_LEFT);

	$n = 0;
	for ($i=0;$i<8;$i++) {
		$flag = substr($timer_rules,(7-$i),1);
		if ($flag==1) {
			$addr = $timers_offset+$i*5;		// get curent timer offset
			$tr_addrs_on[$addr] = null;		// address array for ON times
			$tr_addrs_off[($addr+2)] = null;	// ... and the same for OFFs
			$tr_ids[$n++] = $i;
		}
	}

	$wp_timer_rules_on_sca = get_settings_arr($tr_addrs_on);
	$wp_timer_rules_off_sca = get_settings_arr($tr_addrs_off);
	$i = 0;
	// fill $timers with $timers['timer_id']['on'] = on_unix_time_stamp and same for OFF
	foreach ($tr_addrs_on as $addr=>$empty) {
		$timers[$i]['id'] = $tr_ids[$i];
		$timers[$i]['on'] = $wp_timer_rules_on_sca[$addr][2];
		$timers[$i]['off'] = $wp_timer_rules_off_sca[$addr+2][2];
		$timers[$i]['addr_on'] = $addr;
		$timers[$i]['addr_off'] = $addr+2;
		$i++;
	}

	echo '
	<h3>Программа полива '.($wp_id+1).'</h3>
	<div>
		<table>
		<tr>
			<td title="Максимальное время полива, сек">Таймаут, сек</td>
			<td>
				<input
					onChange="rx_ee_(this)"
					type="text"
					name="csx_'.$addr_wp_timeout.'_value"
					value="'.$wp_timeout.'" 
				/>
			</td>
		</tr>
		<tr>
			<td title="в сантиметрах, уровня воды в баке">Количество воды, см</td>
			<td>
				<input
					onChange="rx_ee_(this)"
					type="text"
					name="csx_'.$addr_wp_volume.'_value"
					value="'.$wp_volume.'" 
				/>
			</td>
		</tr>
		<tr>
		<tr>
			<td>Интервал поливов</td>
			<td><input
				onChange="rx_ee_(this)"
				type="text" 
				name="csx_'.$addr_wp_interval.'_value" 
				value="'.$wp_interval.'" />
			</td>
		</tr>


		<tr>
			<td>Начало</td>
			<td>
				<input
					class="dtpckr1" 
					type="text" 
					id="addr_'.$addr_wp_start.'_dtpckr"
					value="'.$wp_start_formatted.'" 
				/> 
				<input 
					onChange="rx_ee_(this)" 
					type="hidden"
					name="csx_'.$addr_wp_start.'_value"
					value="'.$wp_start.'"
				/>
			</td>
		</tr>
		<tr>
			<td>Конец</td>
			<td>
				<input
					class="dtpckr1" 
					type="text" 
					id="addr_'.$addr_wp_end.'_dtpckr"
					value="'.$wp_end_formatted.'" 
				/> 
				<input 
					onChange="rx_ee_(this)" 
					type="hidden"
					value="'.$wp_end.'"
					name="csx_'.$addr_wp_end.'_value"
				/>
			</td>
		</tr>
		<tr>
			<td>Последний полив</td>
			<td>
				<input
					class="dtpckr1" 
					type="text" 
					id="addr_'.$addr_wp_prev_run.'_dtpckr"
					value="'.$wp_prev_run_formatted.'" 
				/> 
				<input 
					onChange="rx_ee_(this)" 
					type="hidden"
					value="'.$wp_prev_run.'"
					name="csx_'.$addr_wp_prev_run.'_value"
				/>
			</td>
		</tr>
		<tr>

			<td>Поливать учитывая таймеры:</td>
			<td>
				';
			gen_wp_rule_timers($timers);

			echo '
			</td>
		</tr>
		<tr>
			<td>Линии полива</td>
			<td>
				<table 
					onMouseLeave="update_block('.$addr_wp_flags.')" 
					class="flags_blk" id="flags_'.$addr_wp_flags.'"
				>';
					$flgblkid = 101+$wp_id;
					gen_flags_block($flgblkid);
			
		echo '
				</table>
			</td>
		</tr>
		</table>';


		$fmps_arr = get_wp_ferts_array($wp_id);
		$fmps = $fmps_arr['fmp'];
		$fmp_selector = $fmps_arr['selector'];
		echo '	<div id="wp_'.$wp_id.'_fmps">
				<table>';
		echo '<tr>
				<th>id
				<th>fertilizer
				<th>volume
				<th>aftermix
			</tr>';
		foreach($fmps as $fmp_key=>$fmp) {
			$addr_fmp_doserid = $fmp['addr'];
			$addr_fmp_dosingtime = $fmp['addr']+1;
			$addr_fmp_aftermixtime = $fmp['addr']+3;

			echo '<tr>
				<td>FMP #'.($fmp['id']).'</td>
				<td>
					<select 
						onChange="rx_ee_(this)"
						name="csx_'.$addr_fmp_doserid.'_value"

					>'.PHP_EOL;
						get_fert_selector($fmp['doser_id']);	// get fertilizer ids likned to WP
				
				echo PHP_EOL.'	</select>
				</td>
				<td><input 
					onChange="rx_ee_(this)" 
					name="csx_'.$addr_fmp_dosingtime.'_value" 
					type="text" 
					value="'.$fmp['dosing_time'].'" 
				/></td>
				<td><input 
					onChange="rx_ee_(this)" 
					name="csx_'.$addr_fmp_aftermixtime.'_value" 
					type="text" 
					value="'.$fmp['aftermix_time'].'" 
				/></td>
			</tr>';
		}
		
		echo '</table>';
		
		echo '<br>Link fertilizer<br>';
		echo '<select id="wp_'.$wp_id.'_link_selector">';
		foreach($fmp_selector as $fmp_key=>$fmp) {
			echo '<option value="'.($fmp_selector[$fmp_key]['id']).'">
				FMP#'.($fmp_selector[$fmp_key]['id']).' - 
				'.$fmp_selector[$fmp_key]['doser_id'].':
				 '.$_SESSION['settings_data']['dosers_hints'][($fmp_selector[$fmp_key]['doser_id']+1)].' - 
				Vol: '.$fmp_selector[$fmp_key]['dosing_time'].' - 
				Aftermix: '.$fmp_selector[$fmp_key]['aftermix_time'].' - 
				WPLink: '.$fmp_selector[$fmp_key]['link'].' 
			</option>';
		}
		echo '</select>';
		echo '<button onClick="link_fmp('.$wp_id.');">Link</button>';

		


		echo '
	</div>
	</div>

	';	
}
	

// generate HTML for timers display in WaterinProgram
function gen_wp_rule_timers($timers){
	echo '<table>';
	foreach ($timers as $key=>$timerarr) {
		echo '<tr>
			<td><input 
				type="hidden" 
				value="" 
				id="trid_'.$timerarr['id'].'" /></td>
			<td>ON</td>
			<td><input 
				type="text" 
				name="csx_'.$timerarr['addr_on'].'_value" 
				value="'.$timerarr['on'].'" 
				onChange="rx_ee_(this)"  /></td>

			<td>OFF</td></td>
			<td><input 
				type="text" 
				name="csx_'.$timerarr['addr_off'].'_value" 
				value="'.$timerarr['off'].'" 
				onChange="rx_ee_(this)" /></td>

			</tr>';
	}


				

	echo '</table>';
}




/* takes array like 

[addr1]=>null
[addr2]=>null
...
[addrN]=>null
and fills the values with corresponding sca rows
*/
function get_settings_arr($addrs=null){
	if (($handle = fopen("cadi_settings_conf.csv", "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 10000, ",")) !== FALSE) {
		$sca[$row++] = $data;
	    }
	    fclose($handle);
	}
	if ($addrs!=null) {	// if addresses array received, filter these lines only
		foreach($addrs as $addrs_key=>$addr){
			foreach ($sca as $key=>$row) {	// parse array looking for such an address
				if ($row[0]==$addrs_key) {
					$lastkey = $key;  // when address in CSV string match $addr, store string's number
					$lastaddr = $row[0];	// and get variable address
				}
			}
			if ($lastaddr>0) {	// if record in CSV settings file found
				$sca2[$lastaddr] = $sca[$lastkey];
			}
			else {
			} 	
		}
		unset($sca);
		$sca = $sca2;
	}
	return $sca;
}


// returns Cadi EEPROM setting's value based on its address.
// Output format is "x,y" where 'x' - value type (1(8bit), 2(16) or 3(32)), and 'y' is a value
function get_settings_val($addr){
	$addrs[$addr] = null;
	$sca = get_settings_arr($addrs);
	return $sca[$addr][1].','.$sca[$addr][2];
}

// Puts setting back to CSV file 
 function put_settings_val($addr, $value){
	if (($handle = fopen("cadi_settings_conf.csv", "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 10000, ",")) !== FALSE) {
		$sca[$row] = $data;
		// echo "read";
		if ($sca[$row][0]==$addr) {
			$sca[$row][2] = $value;
		}
		$row++;
	    }
	    fclose($handle);
	}
	// recreate new file with new CSV values of dumped binary
	if (($fp = fopen("cadi_settings_conf.csv", "w")) !== FALSE) {
		foreach ($sca as $key=>$row) {
			fputcsv($fp,$row);
		}
	fclose($fp);
	}
}





?>
