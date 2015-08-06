<?php
if (session_status() == PHP_SESSION_NONE) {
    session_start();
}
// cs - for Cadi Settings
$_SESSION['cs'] = array();

$_SESSION['cs']['timers'][0]['description'] = 'default timer description';
$_SESSION['cs']['timers'][0]['on'] = 0;
$_SESSION['cs']['timers'][0]['off'] = 0;
$_SESSION['cs']['timers'][0]['enabled'] = 0;
$_SESSION['cs']['plugs']['0']['description'] = 'default plug description';
$_SESSION['cs']['plugs']['0']['program_link'] = '0';	// timer, ctimer... id

$_SESSION['cs']['valves']['0']['description'] = 'default plug description';
$_SESSION['cs']['valves']['0']['program_link'] = '0';	// timer, ctimer... id
$_SESSION['csbin_filename'] = 'csbin_local';	// cadi settings binary file name
$_SESSION['csblksize'] = 32;	// 16 variables of 16bit each.


$cs_csv_fn = 'cadi_settings_conf.csv';
$cs_dump_fn = 'cadi_settings_dump';

// read dump file settings from btd.conf (shared with BTDaemon)
if (($handle = fopen("btds/btdaemon.conf", "r")) !== FALSE) {
	while (($data = fgetcsv($handle, 1000, "=")) !== FALSE) {
		// print_r($data);
		if (!(substr($data[0],0,1)=='#') && count($data)>1) {
			if (strpos($data[1],'#')>0) {	// if comment after line met, cut it
				$val_arr = explode('#',$data[1]);
				$settings[$data[0]] = $val_arr[0];
			}
			else {
				$settings[$data[0]] = $data[1];
			}
		}
		// echo PHP_EOL.'SUBSTR:'.substr($data[0],0,1).PHP_EOL;
		// echo PHP_EOL.'STRPOS:'.strpos($data[0],'#').PHP_EOL;
	}
	// echo PHP_EOL.'*** btdaemon.conf loading...'.PHP_EOL;
	// print_r($settings);		   
	if (!($settings['srtrs']<40 && $settings['srtrs']>1000)) {
		$settings['srtrs'] = 100;	// force default size if not in range [40..1000] bytes
	}
	fclose($handle);
}




// injects Cadi settings config file into dump file
function sync_conf2dump(){
	global $settings, $cs_csv_fn, $cs_dump_fn;
	if (($handle = fopen($cs_csv_fn, "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$sca[$row++] = $data;
	    }
	    fclose($handle);
	}


	// read dump file into variable
	$curfsize = filesize($cs_dump_fn);	// current response file size
	clearstatcache();				// refresh file size
	$fp = fopen($cs_dump_fn, 'rb'); // open in binary read mode.
	fseek($fp, 0,0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,$curfsize); // read settings dump data
	fclose($fp); // close the file




	// pack corresponding (from conf file adresses and types) values to settings dump file
	foreach ($sca as $key=>$row) {
		if ($row[1]==1) {
			// pack 8 bit value
			$addr = substr($row[0],0,4);		// 21.11.14, 4 instead of 5
			$parity = substr($row[0],4,1);	// 0 - higher byte, 1 - lower byte
			$pointer = ($addr)*2+$parity;
			$settings_dump[$pointer] = chr($sca[$key][2]);

			echo 'packing 8 bit at '.$addr.' (pointer:'.$pointer.') Val='.$sca[$key][2].PHP_EOL;
		}
		if ($row[1]==2) {
			// 16 bit value pack
			$addr = $row[0];
			$pointer = ($addr)*2;
			$settings_dump[$pointer++] = chr(floor($sca[$key][2]/256));
			$settings_dump[$pointer] = chr($sca[$key][2]%256);
			echo 'packing 16 bit ('.(floor($sca[$key][2]/256)).' and '.($sca[$key][2]%256).') at '.$addr.' (pointer: '.$pointer.')'.PHP_EOL;
		}
		if ($row[1]==3) {
			// 32 bit

			$addr = 0;
			$addr = $row[0];
			$val = 0;
			$val = floor($sca[$key][2]/65536);
			echo 'packing 32 (1/2) bit at '.$addr.PHP_EOL;
			$pointer = ($addr)*2;
			$settings_dump[$pointer++] = chr(floor($val/256));
			$settings_dump[$pointer++] = chr($val%256);

			$addr++;
			$val = 0;
			$val = $sca[$key][2]%65536;
			echo 'packing 32 (2/2) bit at '.$addr.PHP_EOL;
			$pointer = ($addr)*2;
			$settings_dump[$pointer++] = chr(floor($val/256));
			$settings_dump[$pointer] = chr($val%256);

		}
	}

	// recreate new dump file
	$fp = fopen($cs_dump_fn, 'w');
	fwrite($fp, $settings_dump);
	fclose($fp); // close the file

	$fp = fopen('cdtest', 'w');
	fwrite($fp, $settings_dump);
	fclose($fp); // close the file

}

// extracts Cadi settings from dump file into config file
function sync_dump2conf(){
	// CSV file contains lines with fields:
	// 0. address
	// 1. type (1 - 8bit, 2 - 16bit, 3 - 32bit)
	// 2. value (plain text integer value)
	// 3. text name, explaining the meaning of the value, kind of comment
	global $settings, $cs_csv_fn, $cs_dump_fn;
	if (($handle = fopen($cs_csv_fn, "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$sca[$row++] = $data;
	    }
	    fclose($handle);
	}


	// TEST BLOCK
	$curfsize = filesize($cs_dump_fn);	// current response file size
	clearstatcache();				// refresh file size
	$fp = fopen($cs_dump_fn, 'rb'); // open in binary read mode.
	fseek($fp, 0,0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,$curfsize); // read settings dump data
	fclose($fp); // close the file
	$settings_file_hex='';
	for ($i=0; $i<strlen($settings_dump);$i++) {
		if (($i%10)==0) {
			$settings_file_hex .= PHP_EOL.'<br>';
			$settings_file_hex .= ' '.($i/10).' ';
		}
		$settings_file_hex .= sprintf("\\x%02x",ord($settings_dump[$i]));
	}
//	echo PHP_EOL."settings file hex (first byte has STM32 EEPROM address ".$settings['cs_start_addr']."): ".PHP_EOL.$settings_file_hex.PHP_EOL;
	// EOF TEST BLOCK


	// extract corresponding (to conf file adresses and types) values from settings dump file
	foreach ($sca as $key=>$row) {
//		echo 'key: '.$key.'<br>';
//		print_r($row);
		if ($row[1]==1) {
			// extract 8 bit value
			$addr = substr($row[0],0,4);
			$parity = substr($row[0],4,1);	// 0 - lower byte, 1 - higher byte
			
			if (($fp = fopen($cs_dump_fn, "rb")) !== FALSE) {
				$seekaddr = ($addr)*2;
				if ($parity == '1'){
					$seekaddr++;
				}
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 1);
				// echo '8B: Addr '.$seekaddr.' has '.(ord($data)).' <br>';
				$sca[$key][2] = ord($data);
				fclose($fp);
			}
		}
		if ($row[1]==2) {
			// 16 bit value extraction

			$addr = $row[0];
			//echo 'Extracting 16 bit val from addr '.$addr.PHP_EOL;
			if (($fp = fopen($cs_dump_fn, "rb")) !== FALSE) {
				$seekaddr = ($addr)*2;
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 2);
				// echo '16B: Addr '.$seekaddr.' has '.(ord($data[0])*256+ord($data[1])).' <br>';
				$sca[$key][2] = ord($data[0])*256+ord($data[1]);
				fclose($fp);
			}
		}
		if ($row[1]==3) {
			// extract 32 bits from dump file
			$addr = substr($row[0],0,5);
			if (($fp = fopen($cs_dump_fn, "rb")) !== FALSE) {
				$seekaddr = ($addr)*2;
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 4);
				// echo '32B: Addr '.$seekaddr.' has '.(ord($data[0])*16777216+ord($data[1])*65536+ord($data[2])*256+ord($data[3])).' <br>';
				$sca[$key][2] = ord($data[0])*16777216+ord($data[1])*65536+ord($data[2])*256+ord($data[3]);
				fclose($fp);
			}
		}
	}

	// recreate new file with new CSV values of dumped binary
	if (($fp = fopen($cs_csv_fn, "w")) !== FALSE) {
		foreach ($sca as $key=>$row) {
			fputcsv($fp,$row);
		}
	fclose($fp);
	}

}

// extract the value with $addr address from Cadi Settings CSV file
function get_csv_value($addr){
	global $cs_csv_fn;
	echo 'get_csv_value()    ';
	if (($handle = fopen($cs_csv_fn, "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$sca[$row++] = $data;
		if ($data[0]==$addr) {
			print_r($data);
			return $data;	// [0] - addr, [1] - type, [2] - value, [3] - description
		}
	    }
	    fclose($handle);
	}
	echo 'returning empty arr';
	return $data;
}


?>
