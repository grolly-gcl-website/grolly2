<?php

$settings['fsd'] = 5;
$settings['srtrs'] = 150;

$fp2 = fopen('cadi_status.csv', 'w');
$fp = fopen('serialresp.out', 'rb');

stream_set_blocking($fp2, 0);
stream_set_blocking($fp, 0);


while (1) {
	$ping_packet = "\x5a\x58\x32\x04\x33\x01\x06\x00";
	$command = "/bin/echo -e '";
	for ($i=0; $i<strlen($ping_packet);$i++) {
		$command .= sprintf("\\x%02x",ord($ping_packet[$i]));
	}
	$command .= "' >> /dev/cadi";
	echo $ping_packet.PHP_EOL.$command.PHP_EOL;
	exec($command.' &');
	usleep(25000);


	$curespfs = filesize('serialresp.out');	// current response file size
	clearstatcache();				// refresh file size


	



	if ($curespfs>($respfs+$settings['fsd'])) {	// compare new and old values if they match. if not, file changed (35B for minimum file difference)
		$respfs = $curespfs;		// update old file size value
		parse_response($settings['srtrs']);		// parse response if change detected
	}
}

function parse_response($srtrs){
	global $fp, $fp2;
	echo 'Parsing start '.time().PHP_EOL;
	echo '('.time().')...seek'.PHP_EOL;
	fseek($fp, ($srtrs*(-1)), SEEK_END); // It needs to be negative (*(-1)) to seek from the file end
	echo '('.time().')...read'.PHP_EOL;
	$data = fread($fp, $srtrs);
	echo '('.time().')...close'.PHP_EOL;
	
	echo '('.time().')...explode'.PHP_EOL;
	$packets_arr = explode('ZX', $data);
	echo '('.time().')...count'.PHP_EOL;
	$last_packet_id = count($packets_arr)-1;
	echo '('.time().')...getting last packet'.PHP_EOL;
	$last_packet = $packets_arr[$last_packet_id];

	if (strlen($last_packet[0])>0) {
		$packet_type = $last_packet[0];
		$packet_size = ord($last_packet[1]);
	}
	echo '('.time().')...packet type'.PHP_EOL;
	echo '('.time().')...packet size='.$packet_size.PHP_EOL;
	
	for ($i=0; $i<strlen($last_packet);$i++) {
		$packet_hex .= sprintf("\\x%02x",ord($last_packet[$i]));
	}
	echo PHP_EOL.'*** Packet HEX is:'.PHP_EOL;

	if ($packet_type==1 && strlen($last_packet)==37) {		// parsing Cadi settings
		$crc = ord($last_packet[($packet_size-3)]);
		$counted_crc = crc_block(2, $last_packet, (ord($last_packet[1])-3));
		if ($counted_crc==$crc) {	// $crc!=0 - dangerous thing, some block could be lost because crc in fact could be 0
			$block_addr = ord($last_packet[2])+ord($last_packet[3])*256;		// block address
			$block_size = 32;
			$block_data = substr($last_packet,4,32);
			dump_settings_block($block_addr, $block_data);
			$repeat_last_cmd = 0;
			usleep($settings['csd']*10);
		}
	}


	echo '('.time().')...status extractor start'.PHP_EOL;
	if ($packet_type==3 && strlen($last_packet)>37) {		// parsing Cadi status
		// STATUS data provider
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
			
			echo '('.time().')...crc block count'.PHP_EOL;
			$counted_crc = crc_block(2, $last_packet, ($packet_size-3));	// 90 xor 88 = 2
			if ($counted_crc==$crc) {		// CRC ok
				$outstr = '';
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
				echo '('.time().')...imploding CSV to write out'.PHP_EOL;
				$csv_string = implode(",", $tofile);
				
				//echo '('.time().')...flushig cadi_status.csv'.PHP_EOL;
				//file_put_contents('cadi_status.csv', '');
				echo '('.time().')...writing cadi_status.csv. Fopen'.PHP_EOL;

				
				echo '('.time().')...FLOCK'.PHP_EOL;
				if (flock($fp2, LOCK_EX)) {
					echo '('.time().')...FSEEK'.PHP_EOL;
					fseek($fp2, 0);
					echo '('.time().')...FWRITE'.PHP_EOL;				
					fwrite($fp2, $csv_string);
					echo '('.time().')...FLUSH'.PHP_EOL;
					fflush($fp2);
					// echo '('.time().')...FCLOSE'.PHP_EOL;
					echo '('.time().')...F_UNLOCK'.PHP_EOL;
					flock($fp2, LOCK_UN);
				}
				// exec('CBTD_STATUS_STRING='.$csv_string);
			}
			else {
				echo 'CRC broken';
				echo 'Counted CRC='.$counted_crc.'<br>';
				echo 'PacketCRCByte='.$crc.'<br>';
			}
		}
	   
	}
	
	echo '('.time().')...Parsing end'.PHP_EOL.PHP_EOL.PHP_EOL;
}


fclose($fp2);
fclose($fp);

function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
}


function get_shm_block(){
	

}




?>
