<?php

ini_set ("display_errors", "1");

error_reporting(E_ALL);

echo PHP_EOL.'=== Cadiweb binary to CSV log parser v0.2 ==='.PHP_EOL;

$filename = 'serialresp.out';
$filename2 = "csvout.csv";
$logfile_size = filesize($filename);
$rxms = 0;	// RX Machine state
$curpntr = 0;		// current binary file pointer
$bblock_size = 500000;	// binary log file block size
$blk = '';
$records_counter = 0;	// number of records processed
$start = time();

$log_blocks_amount = floor($logfile_size / $bblock_size)+1;

	echo PHP_EOL.'Filesize is: '.$logfile_size;


	for ($z = 0; $z < $log_blocks_amount; $z++){
		if (file_exists($filename)) {
			$fp = fopen('serialresp.out', 'rb');
			fseek($fp, $bblock_size*$z); // It needs to be negative (*(-1)) to seek from the file end
			$data = fread($fp, $bblock_size);
			fclose($fp);
			$packets_arr = explode('ZX', $data);
		}
		foreach ($packets_arr as $key => $last_packet) {
			$packet_type = $last_packet[0];
			$packet_size = ord($last_packet[1]);
			if ($packet_type==3 && strlen($last_packet)>37) {		// parsing Cadi status

				$block_id = ord($last_packet[36]);
				if ($block_id==1) {
					$comm_state = ord($last_packet[2]);		// TxBuffer[4] in STM32 firmware
					$timerStateFlags = decbin(ord($last_packet[3]));
					$cTimerStateFlags = decbin(ord($last_packet[4]));
					$wpStateFlags = ord($last_packet[7]);
					$dht[0] = ord($last_packet[8]);
					$dht[1] = ord($last_packet[9]);
					$dht[2] = ord($last_packet[10]);
					$dht[3] = ord($last_packet[11]);
					// Sonar data
					$sonar_read[0] = ord($last_packet[16])+ord($last_packet[17])*265;		// First sonar lower and higher bytes
					$sonar_read[1] = ord($last_packet[18])+ord($last_packet[19])*265;		// First sonar lower and higher bytes

					$cadi_ta[2] = ord($last_packet[12]);		// TxBuffer[14] = (uint8_t)(RTC->CNTH&(0xFF));
					$cadi_ta[3] = ord($last_packet[13]);		// TxBuffer[15] = (uint8_t)((RTC->CNTH>>8)&(0xFF));
					$cadi_ta[0] = ord($last_packet[14]);		// TxBuffer[16] = (uint8_t)(RTC->CNTL&(0xFF));
					$cadi_ta[1] = ord($last_packet[15]);		// TxBuffer[17] = (uint8_t)(((RTC->CNTL)>>8)&(0xFF));
					$cadi_time = $cadi_ta[3]*16777216+$cadi_ta[2]*65536+$cadi_ta[1]*256+$cadi_ta[0];
					$adc_avg[0] = ord($last_packet[20])+ord($last_packet[21])*256;
					$adc_avg[1] = ord($last_packet[22])+ord($last_packet[23])*256;
					$adc_avg[2] = ord($last_packet[24])+ord($last_packet[25])*256;
					$adc_avg[3] = ord($last_packet[26])+ord($last_packet[27])*256;

					$crc = ord($last_packet[($packet_size-3)]);
					$counted_crc = crc_block(2, $last_packet, ($packet_size-3));	// 90 xor 88 = 2



					if ($counted_crc==$crc) {

						$handle = fopen($filename2, "r");
						fseek($handle, -200, SEEK_END);
						$contents = fread($handle, 200);
						fclose($handle);
						$testarr = explode($cadi_time,$contents);
						if (sizeof($testarr)<2){
						
						

							$outstr = '';
							// echo 'CRC OK';

					

							$statarr['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
							$statarr['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
							$statarr['timerStateFlags'] = str_pad(decbin(ord($last_packet[3])), 4, "0", STR_PAD_LEFT);
							$statarr['cTimerStateFlags'] = str_pad(decbin(ord($last_packet[4])), 4, "0", STR_PAD_LEFT);
							$statarr['valves'] = str_pad(decbin(ord($last_packet[5])), 5, "0", STR_PAD_LEFT);
							$statarr['plugs'] = str_pad(decbin(ord($last_packet[6])), 4, "0", STR_PAD_LEFT);
							$statarr['wpStateFlags'] = decbin(ord($last_packet[7]));
							$statarr['dosingPumpsFlags'] = str_pad(decbin(ord($last_packet[32])), 4, "0", STR_PAD_LEFT);
							$statarr['sonar_read'][0] = $sonar_read[0];	// sonar1 distance
							$statarr['sonar_read'][1] = $sonar_read[1];	// sonar2 distance
							$statarr['time'] = $cadi_time;
							$statarr['adc_avg'][0] = $adc_avg[0];	// ADC average value
							$statarr['adc_avg'][1] = $adc_avg[1];	// ADC average value
							$statarr['adc_avg'][2] = $adc_avg[2];	// ADC average value
							$statarr['adc_avg'][3] = $adc_avg[3];	// ADC average value
							$statarr['psi'] = round((($adc_avg[2]-600)/470), 2); 
							$statarr['comm_state'] = ord($last_packet[2]);
							$statarr['auto_flags'] = ord($last_packet[33]);
							$statarr['wpProgress'] = ord($last_packet[34]);
							$statarr['auto_failures'] = ord($last_packet[35]);
							$statarr['runners'] = ord($last_packet[28]);
							$statarr['psi_state'] = ord($last_packet[31]);


							// prepare array for CSV
							$tofile[0] = $cadi_time;
							$tofile[1] = $statarr['dht']['temp'];
							$tofile[2] = $statarr['dht']['rh'];
							$tofile[3] = $statarr['timerStateFlags'];
							$tofile[4] = $statarr['cTimerStateFlags'];
							$tofile[5] = $statarr['valves'];
							$tofile[6] = $statarr['plugs'];
							$tofile[7] = $statarr['wpStateFlags'];
							$tofile[8] = $statarr['sonar_read'][0];
							$tofile[9] = $statarr['sonar_read'][1];
							$tofile[10] = $statarr['adc_avg'][0];
							$tofile[11] = $statarr['adc_avg'][1];
							$tofile[12] = $statarr['adc_avg'][2];
							$tofile[13] = $statarr['adc_avg'][3];
							$tofile[14] = $statarr['psi'];
							$tofile[15] = $statarr['comm_state'];
							$tofile[16] = $statarr['dosingPumpsFlags'];
							$tofile[17] = $statarr['auto_flags'];
							$tofile[18] = $statarr['wpProgress'];
							$tofile[19] = $statarr['auto_failures'];
							$tofile[20] = $statarr['runners'];
							$tofile[21] = $statarr['psi_state'];
				
				
							$csv_string = implode(",", $tofile);
							file_put_contents($filename2, $csv_string, FILE_APPEND);
							file_put_contents($filename2, PHP_EOL, FILE_APPEND);
							$records_counter++;
						}

					}
					else {
						//echo 'CRC broken';
						//echo 'Counted CRC='.$counted_crc.'<br>';
						//echo 'PacketCRCByte='.$crc.'<br>';
					}
				}
		}
	}
}

$secs = time() - $start;
echo 'Finished processing log of '.$logfile_size.' bytes within '.$secs.' seconds.'.PHP_EOL.'Processed '.$records_counter.' records';

function dump_csv($cadi_time, $last_packet){
	global $records_counter;
	$filename = "csvout.csv";
	$handle = fopen($filename, "r");
	fseek($handle, -200, SEEK_END);
	$contents = fread($handle, 200);
	fclose($handle);
	$testarr = explode($cadi_time,$contents);
	if (sizeof($testarr)<2){

		

		// echo $records_counter.PHP_EOL;
		file_put_contents($filename, $str, FILE_APPEND);
		file_put_contents($filename, PHP_EOL, FILE_APPEND);
		$records_counter++;
	}
	else {
		// echo '... Already seen'.PHP_EOL;
	}
}














function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
} 

?>
