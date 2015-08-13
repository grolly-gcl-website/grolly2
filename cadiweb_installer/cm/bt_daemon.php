<?php
// error_reporting( E_ALL );
// ini_set('display_errors', 1);
date_default_timezone_set('UTC');
// "echo > cadi_settings_dump" flushes file before downloading new settings from Cadi

$fp_cs = fopen('cadi_status.csv', 'w');
$fp_sresp = fopen('serialresp.out', 'rb');

stream_set_blocking($fp_cs, 0);
stream_set_blocking($fp_sresp, 0);

$respfs = 0;

$settings = array();

include_once('cadi_settings.php');
//$btd_os = 0;	// Ubuntu (default)
$btd_os = 1;	// ArchLinux
exec('date +%Y%m%d -s "20141231"');

echo "Bluetooth daemon for Cadi started";
$btd_cmd_file = 'daemon_cmd';
// initial Cadi BTDaemon settings load
file_put_contents($btd_cmd_file,'reload_settings,');
$respfs = 0;
$ping_delay = 0;
$cycle_counter = 0;
$video = 0;
$status_stream_enabled = 0;		// shows if status stream enabled
$tank = '';
$packet_id = 1;
$repeat_last_cmd = 0;	
$NbrOfDataToSend = 0;	// amount of 16bit variables of settings dump to send
$TxCounter = 0;		// settings transmit pointer
$sbsa = 0;			// Settings Block Start Address

$execmd = 'echo';

$NbrOfDataToRead = 0;	// amount of 16bit variables of settings dump to read from Cadi
$RxCounter = 0;		// settings Rx pointer
$sbsa2 = 0;			// starting address in EEPROM memory of cadi
$ee_addr = 0;

$globcounter = 0;



//----- SHARED MEMORY CONFIGURATION -----
$SEMAPHORE_KEY = 674839575;   			//Semaphore unique key
$SHARED_MEMORY_KEY = 910538672;   	//Shared memory unique key
$shared_memory_id = 0;
$semaphore_id = 0;
$rxbs = 1000;	// rx buffer size in shared memory
$csvs = 400;	// rx buffer size in shared memory
$w_pa = 0;	// variables block offest
$csv_os = 100;	// CSV string address offset in shared memory
$cadi_packet_size = 42;	// maximum size of Cadi packet


$tmparr = sm2arr(sm_get_string());	
$r_pntrs['set_dump'] = 0;	// settings_dump read pointer
$r_pntrs['set'] = $smarr['w_set_bp'];
$r_pntrs['stat'] = $smarr['w_stat_bp'];
$r_pntrs['conf'] = $smarr['w_conf_bp'];


// create cadi settings dump file if not exist
// 150424: Cadi Communication Daemon now creates dump file
/*	if (!file_exists('cadi_settings_dump')) {
		define('SIZE',5000); // size of the file to be created.
		$fp = fopen('cadi_settings_dump', 'w'); // open in write mode.
		echo PHP_EOL."Creating dump file".PHP_EOL;
		fseek($fp, SIZE-1,0); //
		fwrite($fp,'0'); // write data
		fclose($fp); // close the file
	}	
*/	


// load SVG display settings

		echo 'Loading SVG tank defaults'.PHP_EOL;
		// setting tank water polygon parameters (defaults) SVG
		$GLOBALS['tank'][3]['top'] = 14;	// Maximum water level (distance in cm to sonar installed on top of the water tank)
		$GLOBALS['tank'][3]['bottom'] = 100;	// minimum water level
		$GLOBALS['tank'][3]['svg']['height'] = 400;	// SVG water polygon height
		$GLOBALS['tank'][3]['svg']['tank_top'] = 20;	// SVG water polygon top offset

		$GLOBALS['tank'][4]['top'] = 7;
		$GLOBALS['tank'][4]['bottom'] = 47;
		$GLOBALS['tank'][4]['svg']['height'] = 430;
		$GLOBALS['tank'][4]['svg']['tank_top'] = 300;
		$settings['photo_divider'] = 80;
		$settings['fsd'] = 37;
		$settings['csd'] = 25000;
		$settings['sppd'] = 80;
		$settings['srtrs'] = 150;
		print_r($tank);

function make_photoshot($time){
	exec('fswebcam -r 800x600 --jpeg 100 --save /srv/http/imgs/sht'.$time.'.jpg');
	sleep(1);
	exec('fswebcam -r 800x600 --jpeg 100 --save /srv/http/imgs/last.jpg');
}

// init variable for keeping time of next shot to be made
$nextshot = 0;

while(1){

	$now = time();
	if ($nextshot<$now) {
		$nextshot = $now+300;	// every  5 minutes
		make_photoshot($now);
	}

	$cycle_counter++;
	if ($repeat_last_cmd>0) {		// repeat last cmd if no confirmation got
		exec($execmd);
		$repeat_last_cmd--;
	}
	// should put elseif to avoid command run before long ones (like settings upload/download) finished?

	else {						// or continue main loop if no tasks waits to execute
		$command = file_get_contents($btd_cmd_file);
		
		
		if ($RxCounter>=$NbrOfDataToRead && $TxCounter>=$NbrOfDataToSend && $ee_addr>0){
			$ee_addr = 0;
			// setting transfer activities are finished
			echo 'setting transfer activities are finished'.PHP_EOL;
			$fp = fopen("btds/btd_state", "w");
			if (flock($fp, LOCK_EX | LOCK_NB)) { // do an exclusive lock
			    fwrite($fp, "1");
			    flock($fp, LOCK_UN); // release the lock
			} else {
			    echo "Couldn't lock the file !";
			}

			fclose($fp);

		}

		if(!empty($command)){
			unset($execmd);
			$cmd_arr = explode(",", $command);
			// print_r($cmd_arr);
			echo '('.date(DATE_RFC2822).') Got CMD ';
			switch ($cmd_arr[0]) {
				case 'con':	// automatic connection
					$mac = $cmd_arr[2];
					$rfcomm = $cmd_arr[1];
					bt_autodisconnect($mac, intval($rfcomm));
					echo PHP_EOL.'*** Running automatic connection script'.PHP_EOL;
					bt_autoconnect($mac, intval($rfcomm));
					//file_put_contents($btd_cmd_file,'stream,'.$rfcomm.',');
					$respfs = 0;
					break;
				case 'discon':	// opposite to 'con'
					echo PHP_EOL.'*** Running automatic dis-connection script'.PHP_EOL;
					bt_autodisconnect();			
					break;
				case 'bind':
					$execmd = 'rfcomm -r bind /dev/'.$cmd_arr[1].' '.$cmd_arr[2].' 1';
					echo PHP_EOL.'Binding RFCOMM: '.$execmd.PHP_EOL;
					exec($execmd);
					echo 'applying port settings - raw data exchange';
					exec('stty -F /dev/'.$cmd_arr[1].' raw');
					exec('stty -F /dev/'.$cmd_arr[1].' -echo -echoe -echok');
					$execmd = 'ln -s /dev/'.$cmd_arr[1].' /dev/cadi';
					break;
				case 'disconnect':
					$execmd = "'kill -9 $(pidof rfcomm)'";
					break;
				case 'stream':
					sleep(4);	// hardcoded delay before streaming. For stable bluetooth connection
					$tmpcmddd = 'cp serialresp.out log/serialresp'.time().'.out';
					exec($tmpcmddd);
					// $execmd = "cat /dev/".$cmd_arr[1]." > serialresp.out &";
					$execmd = "/srv/http/cm/ccd > /dev/null &";	// start CCD
					$respfs = 0;
					// sleep(10);
					$status_stream_enabled = 1;		// shows if status stream enabled
					break;
				case 'release':
					$execmd = 'rfcomm release 0 ; rm -rf /dev/'.$cmd_arr[1].' ; rm -rf /dev/cadi';
					exec($execmd);
					if ($btd_os==0) {
						$execmd = 'service bluetooth restart';	// Ubuntu 12.04 LTS
					}
					if ($btd_os==1) {
						$execmd = "'./bt_restart_arch.sh'";	// ArchLinux for Raspberry Pi
					}
					exec($execmd);
					break;
					// rx_ee, cadi, <start_addr>, <number_of_data>
				case 'rx_ee':	// sends block of settings dump file to Cadi   UPLOAD
					sync_conf2dump();
					$ping_delay = 10;	// delay status stream requests
					usleep($settings['csd']*$settings['sppd']);	// delay to finish previous transfers
					$sbsa_tx = $cmd_arr[2];	// start address
					
					// 21.11.2014 fix to upload 8bit values
					if (strlen($sbsa_tx)==5) {
						$sbsa_tx = substr($sbsa_tx, 0, 4);
					}
					// EOF 21.11.2014 fix to upload 8bit values

					$NbrOfDataToSend = $cmd_arr[3];	// amount of 16bit variables of settings dump to send
					$TxCounter = 0;		// reset counter
					$execmd="";
					break;
				case 'server2ee':
					usleep($settings['csd']*$settings['sppd']);	// delay to finish previous transfers
					if ($settings['cs_start_addr']==0) {
						$sbsa = 1400;		// default start point
					}
					else {
						$sbsa = $settings['cs_start_addr'];			// settings start EEPROM address
					}
					$TxCounter=0;
					$NbrOfDataToSend = 400;	// amount of settings to read (16bit values)  HARDCODE
					$execmd='echo';

					break;
/*				case 'ee2server':	//	get Cadi EEPROM dump to server's local binary file	DOWNLOAD
					$ping_delay = 10;
					usleep($settings['csd']*$settings['sppd']);	// delay to finish previous transfers
		
					if ($settings['cs_start_addr']==0) {
						$sbsa = 1400;		// default start point
					}
					else {
						$sbsa = $settings['cs_start_addr'];			// settings start EEPROM address
					}
					$RxCounter=0;
					$NbrOfDataToRead = 400;	// amount of settings to read (16bit values)  HARDCODE
					$execmd='echo';

					// create settings dump file if not exist 
					if (!file_exists('cadi_settings_dump')) {
						define('SIZE',5000); // size of the file to be created.
						$fp = fopen('cadi_settings_dump', 'w'); // open in write mode.
						echo PHP_EOL."Creating dump file".PHP_EOL;
						fseek($fp, SIZE-1,0); //
						fwrite($fp,'0'); // write data
						fclose($fp); // close the file
					}
					
					break; */
				case 'ee2server':	//	get Cadi EEPROM dump to server's local binary file	DOWNLOAD
					$ping_delay = 10;
						
					if ($settings['cs_start_addr']==0) {
						$sbsa = 1400;		// default start point
					}
					else {
						$sbsa = $settings['cs_start_addr'];			// settings start EEPROM address
					}

						

					// create settings dump file if not exist 
					/* if (!file_exists('cadi_settings_dump')) {
						define('SIZE',5000); // size of the file to be created.
						$fp = fopen('cadi_settings_dump', 'w'); // open in write mode.
						echo PHP_EOL."Creating dump file".PHP_EOL;
						fseek($fp, SIZE-1,0); //
						fwrite($fp,'0'); // write data
						fclose($fp); // close the file
					} */
					
					break; 
				case 'kill':
					$execmd = "'kill -9 $(pidof rfcomm)'";
					break;
				case 'restart':
					$execmd = "'./bt_restart.sh'";
					break;
				case 'change_video':
					$execmd = "echo";
					$video=$cmd_arr[1];
					break;
				case 'tx':		// send packet
					/*
					if ($status_stream_enabled==1) {
						$ping_delay = 10;
						usleep($settings['csd']*$settings['sppd']);
					}

					// $packet = $cmd_arr[2];
					
					// $part2 = sprintf("\\x%02x",$packet_id);
					// $part3 = sprintf("\\x%02x",0);
					// $packet .= $part2.$part3;
					// $execmd = "/bin/echo -e '".$packet."' >> /dev/".$cmd_arr[1];
					$execmd = 'echo';
					for ($i=0; $i<strlen($packet);$i++) {
						$packet_hex .= sprintf("\\x%02x",ord($packet));
					}
					echo PHP_EOL.'### Pckt: '.$packet_hex.' ###'.PHP_EOL;	// display packet contents into BTD log
					tx2sm($packet);
					// $repeat_last_cmd = 200;	// enable repeater. Disabled within response parser
					// $ping_delay = 10; */
				break;
				case 'reboot':
					$execmd = "reboot";
					break;
				case 'stream_status':
					echo PHP_EOL.'*** Going to stream status locally'.PHP_EOL;
					$status_stream_enabled = $cmd_arr[1];
					

					// change C damon stream_status flag
					$shared_memory_string = sm_get_string();
					//CONVERT TO AN ARRAY OF BYTE VALUES
					$shared_memory_array = array_slice(unpack('C*',"\0".$shared_memory_string), 1);		
					unset($smstr);
					$smarr = sm2arr($shared_memory_string);
					echo 'Current stream_status changed from: '.$smarr['stream_status'].' to '.$status_stream_enabled.PHP_EOL;
					/*echo 'Current w_pointer: '.$smarr['w_pointer'].PHP_EOL;
					echo 'Current r_pointer: '.$smarr['r_pointer'].PHP_EOL;
					echo 'Current strema_status: '.$smarr['w_stat_bp'].PHP_EOL;
					echo 'Current strema_status: '.$smarr['w_set_bp'].PHP_EOL;
					 */

					$smarr['stream_status'] = $cmd_arr[1];
					unset($smstr);
					$smstr = arr2sm($smarr);

				
					sm_open();
					shmop_write($shared_memory_id, $smstr, 0);
					sm_close();

					break;
				case 'cadiweb_update':
					echo 'Going to upgrade to latest Cadiweb version';
					$status_stream_enabled = 0;
					$execmd = 'rfcomm release 0 ; rm -rf /dev/'.$cmd_arr[1].' ; rm -rf /dev/cadi';
					exec($execmd);
					sleep(1);
					$execmd = 'service bluetooth restart';	// Ubuntu 12.04 LTS
					exec($execmd);
					sleep(1);
					$execmd = 'wget https://github.com/plantalog/cadi-gcl/raw/master/cadiweb/install-ubuntu1204.sh -O /tmp/install.sh';
					exec($execmd);
					$execmd = 'chmod 777 /tmp/install.sh';
					exec($execmd);
					$execmd = 'echo starting Cadiweb install >> /var/www/cadiweb_update_log';
					exec($execmd);
					$install_in_progress=1;
					while ($install_in_progress==1) {
						$out = array();
						$execmd = "grep 'IT IS RECOMMENDED TO RESTART COMPUTER' /var/www/cadiweb_update_log";
						exec($execmd, $out);
						if (sizeof($out)>0) {
							if( strpos($out[0],"IT IS RECOMMENDED TO RESTART COMPUTER") !== false) {
								$install_in_progress=0;
		    					}
						}
					}
					$execmd = 'date';
					exec($execmd);
					echo 'CADIWEB UPDATE DONE'.PHP_EOL;

					break;
				case 'reload_settings':	// read settings file fetching the daemon settings
					echo PHP_EOL.' reloading Cadi BTDaemon settings'.PHP_EOL;

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
						}
						echo '*** CBD settings reloaded! ***'.PHP_EOL;
						$settings['csd'] = intval($settings['csd']);
				   		print_r($settings);
						if (!($settings['srtrs']<40 && $settings['srtrs']>1000)) {
							$settings['srtrs'] = 100;	// force default size if not in range [40..1000] bytes
						}
					    fclose($handle);
					}
					else {
						echo PHP_EOL.'WARNING: btds/btdaemon.conf not found!'.PHP_EOL;
						// using default values
						$settings['photo_divider'] = 80;
						$settings['csd'] = 25000;
						$settings['sppd'] = 80;
						$srtrs = 10;
					}
					$execmd = "echo";
					break;
			}

			if (isset($execmd) && strlen($execmd)>1) {
				echo PHP_EOL.'CurCMD='.$execmd.PHP_EOL;
				exec($execmd);
			}
			// flush daemon command file
			file_put_contents($btd_cmd_file,'');
		}
	}

	usleep($settings['csd']);
	if ($cycle_counter%$settings['photo_divider']==0) {	// photo_divider needed for making photo NOT every while() iteration
// uncomment next line if fswebcam used to make photo shots from usb camera (default for Ubuntu)
//		exec('fswebcam -d /dev/video'.$video.' -r 640x480 --jpeg 85 ../img/curimage.jpeg >> /dev/null &');
		echo PHP_EOL;
	}
	if ($cycle_counter>9999999){
		$cycle_counter=0;
	}
//	echo '*** Curesp '.$curespfs.PHP_EOL;


	// shared memory block parser
	if (($cycle_counter%$settings['csv_parse_divider']) == 0 && $status_stream_enabled == 1) {
		update_status_csv();
	}

	
	usleep(1000);


	usleep(1000);

	// sends block of settings dump file into Cadi EEPROM. This block triggered by 'rx_ee' case
	if ($TxCounter<$NbrOfDataToSend && $repeat_last_cmd == 0) {

		file_put_contents('btds/btd_state', '3');	// BTD status - 3:Settings upload from server to Cadi
		$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
		$sfp = ($sbsa_tx+$TxCounter)*2;	// settings file pointer
		echo PHP_EOL.' ^^^^^^^^^^ FIle pointer offset counted like this: '.$sfp.' = ('.$sbsa2.' - '.$settings['cs_start_addr'].' + '.$TxCounter.') * 2';
		fseek($fp,$sfp,0); //point to file start
		$settings_dump = array();
		$settings_dump = fread($fp,2); // read settings dump data
		fclose($fp); // close the file

		unset($arguments);
		for ($i=0; $i<strlen($settings_dump);$i++) {
			$arguments .= sprintf("\\x%02x",ord($settings_dump[$i]));
		}
		
		$ee_addr = $sbsa_tx+$TxCounter;

		unset($packet);
		$packet = '';
		$packet = chr(90).chr(88).chr(50);	// ZX2	
//		$packet .= chr(7);	// packet payload size	
		$packet .= chr(11);	// Protbuzzz v3
		$packet .= chr(15);	// command (rx_ee for 16bit variable (type=2))
		$packet .= chr($ee_addr%256);	// address
		$packet .= chr(floor($ee_addr/256));
		$packet .= $settings_dump[1];
		$packet .= $settings_dump[0];

echo PHP_EOL.'======= Please look at the dump piece '.$arguments.' from '.$sfp.' writing to EEPROM at '.$ee_addr.PHP_EOL;

		unset($crc);
// $counted_crc = crc_block(2, $last_packet, ($packet_size-3));
		$crc =  chr(crc_block(0, $packet, 11));
		$packet .= $crc;
		unset($arguments);
		for ($i=0; $i<10;$i++) {
			$arguments .= sprintf("\\x%02x",ord($packet[$i]));
		}
		unset($packet);
		$cadi_packet = $arguments;

		$TxCounter++;		// increment for next block
		echo PHP_EOL.'CADI SETTINGS UPLOAD: '.$cadi_packet.PHP_EOL;

		// store it as "tx" command for daemon into daemon_cmd
		$btdcmd = '';
		$btdcmd = 'tx,cadi,'.$cadi_packet.',';
		file_put_contents($btd_cmd_file,$btdcmd);
	}

	
	// echo PHP_EOL.$globcounter++.PHP_EOL;
	if ($globcounter>4000000000) {
		$globcounter = 0;
	}
//	exec('streamer -f jpeg -o ../img/curimage.jpeg');
}

function stream_status($status){
	$smarr = get_sm_arr();
	$smarr['stream_status'] = $status;
	put_sm_arr($smarr);
}

function tx2sm($packet){
	stream_status(0);
	usleep(100000);
	$smarr = get_sm_arr();
	for ($i=0;$i<strlen($packet);$i++) {
		$smarr['fifo_tx'][($i+3)] = substr($packet, $i, 1);
	}
	$smarr['txbuff_ne'] = 1;
	$smarr['stream_status'] = 1;
	put_sm_arr($smarr);
}

function update_status_csv(){
	$temppparr = get_sm_arr();
	$csv = status_bin2csv($temppparr['status_buf']);
	

	if (strlen($csv)>2) {				// put_sm_arr() zatiraet sm1->status_buf ?????
		echo 'CSV = '.$csv.PHP_EOL.PHP_EOL;
		$temppparr['csv_string'] = $csv;
		put_sm_arr($temppparr);
	}

	
	
}

function status_bin2csv($statbin){
	// STATUS data provider
	$last_packet = $statbin;
	$block_id = ord($last_packet[38]);		// HARDCODE


	if ($block_id==1) {

		$comm_state = ord($last_packet[4]);		// TxBuffer[4] in STM32 firmware
		$timerStateFlags = decbin(ord($last_packet[5]));
		$cTimerStateFlags = decbin(ord($last_packet[6]));
		$wpStateFlags = ord($last_packet[9]);
		$dht[0] = ord($last_packet[10]);
		$dht[1] = ord($last_packet[11]);
		$dht[2] = ord($last_packet[12]);
		$dht[3] = ord($last_packet[13]);
		$ph1_adc_val = (ord($last_packet[22])*256) + ord($last_packet[23]);
		// Sonar data
		$sonar_read[0] = ord($last_packet[18])+ord($last_packet[19])*265;		// First sonar lower and higher bytes
		$sonar_read[1] = ord($last_packet[20])+ord($last_packet[21])*265;		// First sonar lower and higher bytes

		$cadi_ta[2] = ord($last_packet[14]);		// TxBuffer[14] = (uint8_t)(RTC->CNTH&(0xFF));
		$cadi_ta[3] = ord($last_packet[15]);		// TxBuffer[15] = (uint8_t)((RTC->CNTH>>8)&(0xFF));
		$cadi_ta[0] = ord($last_packet[16]);		// TxBuffer[16] = (uint8_t)(RTC->CNTL&(0xFF));
		$cadi_ta[1] = ord($last_packet[17]);		// TxBuffer[17] = (uint8_t)(((RTC->CNTL)>>8)&(0xFF));
		$cadi_time = $cadi_ta[3]*16777216+$cadi_ta[2]*65536+$cadi_ta[1]*256+$cadi_ta[0];
		$adc_avg[0] = ord($last_packet[22])+ord($last_packet[23])*256;
		$adc_avg[1] = ord($last_packet[24])+ord($last_packet[25])*256;
		$adc_avg[2] = ord($last_packet[26])+ord($last_packet[27])*256;
		$adc_avg[3] = ord($last_packet[28])+ord($last_packet[29])*256;

			$outstr = '';
			$statarr['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
			$statarr['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
			$statarr['timerStateFlags'] = str_pad(decbin(ord($last_packet[5])), 4, "0", STR_PAD_LEFT);
			$statarr['cTimerStateFlags'] = str_pad(decbin(ord($last_packet[6])), 4, "0", STR_PAD_LEFT);
			$statarr['valves'] = str_pad(decbin(((ord($last_packet[31])*256)+ord($last_packet[32]))), 10, "0", STR_PAD_LEFT);
			$statarr['plugs'] = str_pad(decbin(ord($last_packet[8])), 4, "0", STR_PAD_LEFT);
			$statarr['wpStateFlags'] = decbin(ord($last_packet[9]));
			$statarr['dosingPumpsFlags'] = str_pad(decbin(ord($last_packet[34])), 4, "0", STR_PAD_LEFT);
			$statarr['sonar_read'][0] = $sonar_read[0];	// sonar1 distance
			$statarr['sonar_read'][1] = $sonar_read[1];	// sonar2 distance
			$statarr['time'] = $cadi_time;
			$statarr['adc_avg'][0] = $adc_avg[0];	// ADC average value
			$statarr['adc_avg'][1] = $adc_avg[1];	// ADC average value
			$statarr['adc_avg'][2] = $adc_avg[2];	// ADC average value
			$statarr['adc_avg'][3] = $adc_avg[3];	// ADC average value
			$statarr['psi'] = round((($adc_avg[2]-600)/470), 2); 
			$statarr['comm_state'] = ord($last_packet[4]);
			$statarr['auto_flags'] = ord($last_packet[35]);
			$statarr['wpProgress'] = ord($last_packet[36]);
			$statarr['auto_failures'] = ord($last_packet[37]);
			$statarr['runners'] = ord($last_packet[30]);
			$statarr['psi_state'] = ord($last_packet[33]);
			$statarr['ph1_adc_val'] = $ph1_adc_val;

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
			$tofile[22] = $statarr['ph1_adc_val'];	
			$tofile[23] = $statarr['ph1_adc_val'];
			$tofile[24] = $statarr['ph1_adc_val'];
			$tofile[25] = $statarr['ph1_adc_val'];
			
			$csv_string = implode(",", $tofile);

			// echo 'CSVVVVV='.$csv_string;
			return $csv_string;

	}

	return 0;

}


function dump_settings_block($addr, $block_data){
	global $settings;
	$curfsize = filesize('cadi_settings_dump');	// current response file size
	clearstatcache();				// refresh file size
	$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
	fseek($fp, 0,0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,$curfsize); // read settings dump data
	fclose($fp); // close the file
	$settings_hex = '';
	for ($i=0; $i<strlen($block_data);$i++) {
		$settings_hex .= sprintf("\\x%02x",ord($block_data[$i]));
	}
	echo PHP_EOL." !!!!!! Writing settings data from EE addr=".$addr." into cadi_settings_dump dump file: ".$settings_hex.PHP_EOL;
	$offset = ($addr - $settings['cs_start_addr'])*2;	// multiplied by 2 because each value should have 16 bit to store.
	echo PHP_EOL."at offset: ".$offset.PHP_EOL;
	for ($i=0; $i<strlen($block_data);$i++) {
		$settings_dump[($offset+$i)] = $block_data[$i];
	}

	$fp = fopen('cadi_settings_dump', 'w');
	fwrite($fp, $settings_dump);
	fclose($fp); // close the file
}

function sm_get_string(){
	global $shared_memory_id;
	sm_open();
	//----- READ FROM THE SHARED MEMORY -----
	$shared_memory_string = shmop_read($shared_memory_id, 0, shmop_size($shared_memory_id));   	//Shared memory ID, Start Index, Number of bytes to read
	sm_close();
	if($shared_memory_string == FALSE) {
		echo "Failed to read shared memory";
	}
	else {
		return $shared_memory_string;
	}
}



// ===== open Shared Memory =====
function sm_open(){
	global $SEMAPHORE_KEY, $SHARED_MEMORY_KEY;
	global $rxbs, $csvs, $w_pa, $csv_os, $cadi_packet_size;
	global $shared_memory_id, $semaphore_id;

	//Setup access to the shared memory
	$shared_memory_id = shmop_open($SHARED_MEMORY_KEY, "w", 0, 0);	//Shared memory key, flags, permissions, size (permissions & size are 0 to open an existing memory segment)
	$sm_size = shmop_size($shared_memory_id);

	//Create the semaphore
	$semaphore_id = sem_get($SEMAPHORE_KEY);		//Creates, or gets if already present, a semaphore

	//Acquire the semaphore
	if (!sem_acquire($semaphore_id)) {  //If not available this will stall until the semaphore is released by the other process
	    echo "Failed to acquire semaphore $semaphore_id<br />";
	    sem_remove($semaphore_id);	//Use even if we didn't create the semaphore as something has gone wrong and its usually debugging so lets no lock up this semaphore key
	    exit;
	}


//We have exclusive access to the shared memory (the other process is unable to aquire the semaphore until we release it)
	if (empty($shared_memory_id)){
		echo "Failed to open shared memory.";	//
		return 0;
	}
	else {
		return $shared_memory_id;// open ok
	}
}


// ===== close Shared Memory =====
function sm_close(){
	global $SEMAPHORE_KEY, $SHARED_MEMORY_KEY;
	global $rxbs, $csvs, $w_pa, $csv_os, $cadi_packet_size;
	global $shared_memory_id, $semaphore_id;
	//Release the semaphore
	if (!sem_release($semaphore_id)) {  //Must be called after sem_acquire() so that another process can acquire the semaphore
	    echo "Failed to release $semaphore_id semaphore<br />";
	}

	//Detach from the shared memory
	shmop_close($shared_memory_id);
}


// OLD oNE
function sm_get_block() {	// extract CSV from shared memory block
	global $SEMAPHORE_KEY, $SHARED_MEMORY_KEY;
	global $rxbs, $csvs, $w_pa, $csv_os, $cadi_packet_size;

	//Setup access to the shared memory
	$shared_memory_id = shmop_open($SHARED_MEMORY_KEY, "w", 0, 0);	//Shared memory key, flags, permissions, size (permissions & size are 0 to open an existing memory segment)
	$sm_size = shmop_size($shared_memory_id);

	//Create the semaphore
	$semaphore_id = sem_get($SEMAPHORE_KEY);		//Creates, or gets if already present, a semaphore

	//Acquire the semaphore
	if (!sem_acquire($semaphore_id)) {  //If not available this will stall until the semaphore is released by the other process
	    echo "Failed to acquire semaphore $semaphore_id<br />";
	    sem_remove($semaphore_id);	//Use even if we didn't create the semaphore as something has gone wrong and its usually debugging so lets no lock up this semaphore key
	    exit;
	}


	//We have exclusive access to the shared memory (the other process is unable to aquire the semaphore until we release it)
	if (empty($shared_memory_id)){
		echo "Failed to open shared memory.<br />";	//<<<< THIS WILL HAPPEN IF THE C APPLICATION HASN'T CREATED THE SHARED MEMORY OR IF IT HAS BEEN SHUTDOWN AND DELETED THE SHARED MEMORY
	}
	else {
		//----- READ FROM THE SHARED MEMORY -----
		$shared_memory_string = shmop_read($shared_memory_id, 0, shmop_size($shared_memory_id));   	//Shared memory ID, Start Index, Number of bytes to read
		if($shared_memory_string == FALSE) {
			echo "Failed to read shared memory";
			sem_release($semaphore_id);
			exit;
		}
		//CONVERT TO AN ARRAY OF BYTE VALUES
		$shared_memory_array = array_slice(unpack('C*', "\0".$shared_memory_string), 1);		
	}

	//Release the semaphore
	if (!sem_release($semaphore_id)) {  //Must be called after sem_acquire() so that another process can acquire the semaphore
	    echo "Failed to release $semaphore_id semaphore<br />";
	}

	//Detach from the shared memory
	shmop_close($shared_memory_id);
	if (isset($shared_memory_array)) {
		return $shared_memory_array;
	}
	else {
		return 0;
	}
}


function get_sm_arr(){
	global $shared_memory_id;
	$shared_memory_string = sm_get_string();
	//CONVERT TO AN ARRAY OF BYTE VALUES
	$shared_memory_array = array_slice(unpack('C*',"\0".$shared_memory_string), 1);		
	unset($smstr);
	$smarr = sm2arr($shared_memory_string);
	return $smarr;
}

function put_sm_arr($smarr){
	global $shared_memory_id;
	$smstr = arr2sm($smarr);
	sm_open();
	shmop_write($shared_memory_id, $smstr, 0);
	sm_close();
}

// copy shared memoty array of bytes into PHP structured array
function sm2arr($sm_arr) {
	global $rxbs, $w_pa, $csvs;
	$outarr['w_pointer'] = ord($sm_arr[$w_pa]) + ord($sm_arr[($w_pa+1)])*256;	
	$outarr['r_pointer'] = ord($sm_arr[($w_pa+4)]) + ord($sm_arr[($w_pa+5)])*256;	
	$outarr['w_stat_bp'] = ord($sm_arr[($w_pa+8)]) + ord($sm_arr[($w_pa+9)])*256;	
	$outarr['w_set_bp'] = ord($sm_arr[($w_pa+12)]) + ord($sm_arr[($w_pa+13)])*256;	
	$outarr['w_setd_bp'] = ord($sm_arr[($w_pa+16)]) + ord($sm_arr[($w_pa+17)])*256;	
	$outarr['w_conf_bp'] = ord($sm_arr[($w_pa+20)]) + ord($sm_arr[($w_pa+21)])*256;

	// TX pointers
	$outarr['w_tx_pointer'] = ord($sm_arr[($w_pa+24)]) + ord($sm_arr[($w_pa+25)])*256;
	$outarr['r_tx_pointer'] = ord($sm_arr[($w_pa+28)]) + ord($sm_arr[($w_pa+29)])*256;
	$outarr['stream_status'] = ord($sm_arr[($w_pa+32)]);
	$outarr['txbuff_ne'] = ord($sm_arr[($w_pa+33)]);
	$outarr['tx_priority'] = ord($sm_arr[($w_pa+34)]);
	$outarr['dstate'] = ord($sm_arr[($w_pa+35)]);
	$outarr['ping_delay'] = ord($sm_arr[($w_pa+36)]);

	$outarr['empty_arr'] = substr($sm_arr, 37, 63);
	$outarr['csv_string'] = substr($sm_arr, 100, 400);
	$outarr['status_buf'] = substr($sm_arr, 500, 1000);
	$outarr['settings_buf'] = substr($sm_arr, 1500, 1000);
	$outarr['settings_dump'] = substr($sm_arr, 2500, 1000);
	$outarr['confirm_buf'] = substr($sm_arr, 3500, 1000);
	$outarr['fifo_rx'] = substr($sm_arr, 4500, 1000);
	$outarr['fifo_tx'] = substr($sm_arr, 5500, 1000);

	return $outarr;
}


// opposite to sm2arr(): converts PHP array into shared memory array string and stores it
function arr2sm($outarr){

	//print_r($outarr);

	// test display of array to be stored into shared memory
/*	foreach ($outarr as $key=>$packet) {
		unset($field_hex);
		for ($i=0; $i<strlen($packet);$i++) {
			if (($i%16)==0) {
				$field_hex .= PHP_EOL.(str_pad($i, 4, "0", STR_PAD_LEFT)).': ';
			}
			$field_hex .= sprintf("\\x%02x",ord($packet[$i]));

		}
		echo 'THIS IS '.$key.' field (strlen='.(strlen($packet)).')'.$field_hex.PHP_EOL.PHP_EOL.PHP_EOL.PHP_EOL;
	} */


	global $rxbs, $w_pa, $csvs;
	
	$sm_str = null;

	$sm_str .= chr($outarr['w_pointer']%256);		// 0: C unsigned int LSB
	$sm_str .= chr(($outarr['w_pointer']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 3: HSB

	$sm_str .= chr($outarr['r_pointer']%256);		// 4:
	$sm_str .= chr(($outarr['r_pointer']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 7:

	$sm_str .= chr($outarr['w_stat_bp']%256);		// 8:
	$sm_str .= chr(($outarr['w_stat_bp']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 11:

	$sm_str .= chr($outarr['w_set_bp']%256);		// 12:
	$sm_str .= chr(($outarr['w_set_bp']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 15:

	$sm_str .= chr($outarr['w_setd_bp']%256);		// 16:
	$sm_str .= chr(($outarr['w_setd_bp']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 19:

	$sm_str .= chr($outarr['w_conf_bp']%256);		// 20:
	$sm_str .= chr(($outarr['w_conf_bp']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 23:

	$sm_str .= chr($outarr['w_tx_pointer']%256);		// 24:
	$sm_str .= chr(($outarr['w_tx_pointer']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 27:

	$sm_str .= chr($outarr['r_tx_pointer']%256);		// 28:
	$sm_str .= chr(($outarr['r_tx_pointer']>>8)&0x00FF);
	$sm_str .= chr(0);
	$sm_str .= chr(0);					// 31:

	$sm_str .= chr($outarr['stream_status']);		// 32:
	$sm_str .= chr($outarr['txbuff_ne']);			// 33:
	$sm_str .= chr($outarr['tx_priority']);			// 34:
	$sm_str .= chr($outarr['dstate']);			// 35:
	$sm_str .= chr($outarr['ping_delay']);			// 36:

	for ($i=0;$i<63;$i++) {					// 36 - 99
		$sm_str .= substr($outarr['empty_arr'],$i,1);
	}

	for ($i=0;$i<400;$i++) {				// 100 - 499
		$sm_str .= substr($outarr['csv_string'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 500 - 1499
		$sm_str .= substr($outarr['status_buf'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 1500 - 2499
		$sm_str .= substr($outarr['settings_buf'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 2500 - 3499
		$sm_str .= substr($outarr['settings_dump'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 3500 - 4499
		$sm_str .= substr($outarr['confirm_buf'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 4500 - 5499
		$sm_str .= substr($outarr['fifo_rx'],$i,1);
	}

	for ($i=0;$i<1000;$i++) {				// 5500 - 6499
		$sm_str .= substr($outarr['fifo_tx'],$i,1);
	}
	return $sm_str;
}





function bt_autoconnect($macc, $rfcomm_n){
	global $status_stream_enabled;
	$mac = substr($macc,0,17);
	$connected = 0;
	$timeout = time() + 200;	// HARDCODE
	$now = time();
	$attempts = 0;

	$connected = 0;
	$timeout = time() + 200;	// HARDCODE
	$now = time();
	echo 'BT Autoconnect'.PHP_EOL;
	$conn_attempts = 20;		// nuber of attempts to connect
	$repair_every = 5;		// re-pair every N attempts to get 'connected'
	while ($conn_attempts>0 && $connected==0) {
		// *** re-pair bluetooth connection
		if ($conn_attempts%$repair_every==0) {
			echo '*** Starting re-Pair process...'.PHP_EOL;
			$repairing_output = array();
			exec(('expect ../install/autopair.sh '.$mac), $repairing_output);
			echo '*** re-Paired'.PHP_EOL;
			print_r($repairing_output);
		}		
		else {
			echo '*** re-Pair skipped'.PHP_EOL;
		}
		// *** start log streamer
		// $autoconcmd = './bt_autocon_arch.sh '.$mac.' '.intval($rfcomm_n);	// version <1
		$autoconcmd = './bt_autocon_arch.sh '.$mac.' '.intval($rfcomm_n).' 1';	// v1+
		echo $autoconcmd.PHP_EOL.PHP_EOL;
		echo $autoconcmd.PHP_EOL.PHP_EOL;
		echo 'RFCOMM:'.$rfcomm_n.PHP_EOL;
		exec($autoconcmd, $autocon_resp);
		sleep(1);

		// *** extract streamer process id
		$line_id = 0;
		foreach ($autocon_resp as $key=>$bashrespline) {
			$pos = explode('eamer pid=',$bashrespline);
			$position = sizeof($pos);
			echo 'LINE'.$line_id.' = '.$bashrespline.PHP_EOL.$position.PHP_EOL;
			if ($position>1) {	// found LOGGER (serialresp) proc id
				$logger_proc_id = intval($pos[1]);	// extract process id
			}
			else {		// no PID found
				
			}
			$line_id++;
		}
		echo '*** PID of streamer = '.$logger_proc_id.PHP_EOL;

		// *** wait until 'connected' state reached for 'rfcomm'
		$now = time();
		$concheck_timeout = time() + 10;		// HARDCODE 10 seconds timeout for 'connected check'
		while ($now<$concheck_timeout) {
			$now = time();
			exec('rfcomm',$exeresp);
			print_r($exeresp);		// test
			if (strpos($exeresp[0], 'connected')>0) {	// 'connected' found
				$connected = 1;
			}
			else {
				$connected = 0;
			}
			unset($exeresp);
			sleep(2);
			echo '*** Connected?  '.$connected.PHP_EOL;
		}
		if ($connected == 0) {
			exec(('kill -9 '.$logger_proc_id));	// terminate unsuccesfull streamer
		}
		echo PHP_EOL.'*** CONNECTION ATTEMPTS LEFT: '.$conn_attempts.PHP_EOL;
		$conn_attempts--;
	}
	$status_stream_enabled = 1;
}





function bt_autodisconnect($mac, $rfcomm_n){
	global $btd_os;
	$execmd = "'kill -9 $(pidof rfcomm)'";
				
	$execmd = 'rfcomm release 0 ; rm -rf /dev/'.$rfcomm_n.' ; rm -rf /dev/cadi';
	exec($execmd);
	if ($btd_os==0) {
		$execmd = 'service bluetooth restart';	// Ubuntu 12.04 LTS
	}
	if ($btd_os==1) {
		$execmd = "'./bt_restart_arch.sh'";	// ArchLinux for Raspberry Pi
	}
	exec($execmd);

}


function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
} 

function crc_block_ord($input, $ord_arr, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input = 111;
	}
	return $input;
}

fclose($fp_cs);
fclose($fp_sresp);

?>
