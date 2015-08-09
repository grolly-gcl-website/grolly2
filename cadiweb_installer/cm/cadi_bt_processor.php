<?php
if (session_status() == PHP_SESSION_NONE) {
    session_start();
}
//include_once('cadi_response_parser.php');
include_once('packet_processor.php');
include_once('gen_fcm.php');
$btd_state_text = '';



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







if (isset($_GET['action'])) {
	$action = $_GET['action'];
}
else {
	$action = $_POST['action'];
}


switch ($action) {
	case 'bt_connect':
		$toput = 'con,'.$_POST['rfcomm'].','.$_POST['mac'].', ';
		file_put_contents('daemon_cmd', $toput);
		break;
	case 'bt_disconnect':
		$toput = "release,".$_POST['rfcomm'].", ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'btd_stream_start':
		$toput = "stream_status,".$_POST['status'].", ";
		file_put_contents('daemon_cmd', $toput);
		break;
	case 'reboot':
		$toput = "reboot, , ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'cadi_reset':
		bt_tx('cadi', $cadi_packet);
		break;
	case 'bt_restart':
		$toput = "restart,";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'rx_ee':
		$toput = "rx_ee,cadi,".$_POST['addr'].", ".$_POST['number'].", ";
		include_once('cadi_settings.php');
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'tx':
		bt_tx('cadi', $cadi_packet);
		print_r($cadi_packet);
		$smarr = get_sm_arr();
		$smarr['fifo_tx'] = $packet4sm;
		$smarr['txbuff_ne'] = 1;
		put_sm_arr($smarr);

		// sm_comparator();
	
		break;
	case 'tx_packet':
		//$cadi_packet = "Test Cadi Packet";
		//stream_status(0);	// stop pinging cadi
		//usleep(250000);		// delay 150ms to ensure, ping stopped

		stream_status(0);

		usleep(500000);
		$smarr = get_sm_arr();

		$smarr['fifo_tx'] = $packet4sm;
		$smarr['txbuff_ne'] = 1;
		put_sm_arr($smarr);
		echo $packet4sm;
		usleep(500000);
		stream_status(1);

		break;
	case 'rfcomm_scan':
		rfcomm_scan();
		break;
	case 'rfcomm_list_binded':
		rfcomm_list_binded();
		break;
	case 'stop_serial_read':
		stop_serial_read($_POST['process']);
		break;
	case 'tail_serial_log':
		tail_serial_log($_POST['amount']);
		break;
	case 'command_send':
		command_send($_POST['command'], $_POST['mac']);
		break;
	case 'get_status':
//		include_once('cadi_status.php');
		include_once('status_view_1.php');
		echo '---socalledseparator---';
		include_once('status_view_2.php');
		break;
	case 'get_status_csv':
		// $csv = file_get_contents('cadi_status.csv');

		$csv = get_sm_csv();
		$smarr = get_sm_arr();
		$dstate = $smarr['dstate'];
		if ($dstate == 0) {
			$btd_state_text='0 <font color="green">CCD: Idle</font>';
		}
		else if ($dstate == 1) {
			$btd_state_text='1 <font color="green">CCD: CSX_DL</font>';
		}
		else if ($dstate == 2) {
			$btd_state_text='2 <font color="green">CCD: TX</font>';
		}
		else if ($dstate == 3) {
			$btd_state_text='3 <font color="green">CCD: Ping!</font>';
		}		

		echo $csv.','.$btd_state_text;
		
		break;
	case 'get_ip':
		array($output);
		exec('/sbin/ifconfig -a', $output);
		print_r($output);
		echo $out;
		echo 'Should be IFCONFIG output';
		break;
	case 'change_video':
		change_video($_POST['new_video']);
		break;
	case 'btd_apply_settings':		// save settings to file
		btd_apply_settings($_POST['settings']);
		break;
	case 'svg_apply_settings':		// save settings to file
		svg_apply_settings($_POST['settings']);
		break;
	case 'cadiweb_update':		// save settings to file
		$toput = "cadiweb_update, , ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'redraw_update_log':		// save settings to file
		redraw_log();
		break;
	case 'tx_str':
		$tx_str = 'tx,cadi,'.$_POST['str'].',';
		file_put_contents('daemon_cmd', $tx_str);
		break;
	case 'rx_ee_dir':	// semi-direct value upload
		$addr = $_POST['addr'];
		$value = $_POST['value'];
		put_settings_val($addr, $value);
		
		$toput = "rx_ee,cadi,".$_POST['addr'].",2, ";	// 2 - HARDCODEd value type ('2' means 16 bit)
		include_once('cadi_settings.php');
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		echo 'rx_ee_dir completed sending '.$value.' to 0d0'.$addr;
		break;	
/*	case 'upload_csx':
		// parse $params into array of values
		$params = array();
		parse_str($_POST['csx_data'], $params);
		upload_csx($params);
		sync_conf2dump();	// synchronize: config CSV-to-dump
		break; */
/*	case 'download_csx':	// PHP daemon settings DL with Protobuzz v2
		file_put_contents('daemon_cmd', 'ee2server,1500,200,');
		break; */
	case 'download_csx':	// Protobuzzz v3 automatic setting DL with C daemon
		// file_put_contents('daemon_cmd', 'ee2server,1500,200,');
		break;
	case 'get_btd_state':

		break;
	case 'get_settings_val':
		get_settings_val($_POST['addr']);	// read CSV file's value from 'addr' line
		break;
	case 'put_settings_val':
		put_settings_val($_POST['addr'], $_POST['value']);
		break;



}



function stream_status($status){
	global $shared_memory_id;
	sm_open();
	shmop_write($shared_memory_id, chr($status), 32);		// HARDCODE sm1->stream_status = 0;
	sm_close();
}




// uploads Cadi Settings passed from Web Form in Cadiweb into Cadi EEPROM
function upload_csx($csx){
	// print_r($csx);
	$curaddr = 0;
	$ndx = 0;
	unset($outarr);
	$outarr = array();

	$sca = csx2sca($csx);

	// prepare cadi settings config file contents
	unset($outfile);
	foreach ($sca as $key=>$value) {
		unset($line);
		$line = implode(',', $value);
		$line .= PHP_EOL;
		$outfile .= $line;
	}
	
	echo $outfile;

	// put file contents into config file
	file_put_contents('cadi_settings_conf.csv','');
	file_put_contents('cadi_settings_conf.csv',$outfile);
	
	sleep(1);
}

// uploads part of Cadi Settings passed from Web Form in Cadiweb into Cadi EEPROM
function upload_csx_short($csx){
	$sca_full = get_settings_arr();		// get full array of settings, based on CSV
	$sca_short = csx2sca($csx);		// get array of settings passed from Cadiweb UI

	$curaddr = 0;
	$ndx = 0;
	unset($outarr);
	$outarr = array();
	foreach ($sca_short as $key=>$value) {
		unset($tmparr);
		$rowaddr = $value[0];	// address to be set
		$sca_addr = 0;		// reset sca address search to 0
		$sca_ndx = 0;		// reset sca CSV line index to 0 before start seraching proper line
		while ($rowaddr!=$sca_addr && $sca_ndx<sizeof($sca_full)) {	// until end of sca array or address found
			$sca_addr = $sca_full[$sca_ndx++][0];			// reassign new sca address until found
		}
		if ($sca_ndx<sizeof($sca_full)) {		// address found
			$sca_full[--$sca_ndx][2] = $value[2];	// assign new value
		}
	}
	// prepare cadi settings config file contents
	unset($outfile);
	foreach ($sca_full as $key=>$value) {
		unset($line);
		$line = implode(',', $value);
		$outfile .= $line;
	}
	// put file contents into config file
	file_put_contents('cadi_settings_conf.csv',$outfile);

	sleep(1);
	
}

function redraw_log(){
	$logtail = array();
	exec('tail 500 /var/www/cadiweb_update_log', $logtail);
	for ($i=0; $i<sizeof($logtail);$i++){
		echo $logtail[$i].PHP_EOL;
	}
	
}

function btd_apply_settings($settings){
	file_put_contents('btds/btd.conf',$settings);
	file_put_contents('daemon_cmd','reload_settings,');	// force Cadi BTDaemon to reload settings file
	echo $settings;
}

function svg_apply_settings($settings){
	file_put_contents('svg.conf',$settings);
	echo $settings;
}

function command_send($command, $mac){
	$out = array();
	$cmd = 'sudo echo '.time().': '.$command.' >> cadi_input';
	exec($cmd, $out);
	echo $cmd;
}

function change_video($video){
		$toput = "change_video,".$video.", , ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
//		sleep(1);
}

function bt_tx($rfcomm, $data){
		$toput = "tx,".$rfcomm.",".$data.", ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
}

function tail_serial_log($amount){
	$filename = 'serialresp.out';
	if (1) {
		# Processing
		
		$i = 0;
		$arrr = read_file('serialresp.out', 10);
		echo "---separator---";
		$cadi_str_arr = explode(',', $arrr[8]);
		$cadi_time = date('l jS \of F Y h:i:s A', $cadi_str_arr[0]);
		
		
		for ($i=0; $i<3; $i++) {
			$curflag = substr($cadi_str_arr[3], $i, 1);
			if ($curflag == '1'){
				$plugout .= '<b style="background:green; color:white;">&nbsp;'.$i.'&nbsp;</b>';
			}
			else {
                       		$plugout .= '<b style="background:red; color:white;">&nbsp;'.$i.'&nbsp;</b>';
			}
		}

		for ($i=0; $i<3; $i++) {
			$curflag = substr($cadi_str_arr[7], $i, 1);
			if ($curflag == '1'){
				$ctimerout .= '<b class="flgenb">'.$i.'</b>';
			}
			else {
                       		$ctimerout .= '<b class="flgdsb">'.$i.'</b>';
			}
		}
		
		
		$timer_States='';
		echo $arrr[8];
	}
		

}

function stop_serial_read($psid){
	$out = array();
	exec('kill -9 '.$psid, $out);
	echo 'kill -9 '.$psid;
}

function rfcomm_scan(){
	$out = array();
	$out2 = array();
	exec('hciconfig hci0 up');
	exec('hcitool scan', $out);

	// parse 'hcitool scan' output and prepare options to put into <select> html tag
	for ($i=1; $i<sizeof($out); $i++) {
		$out2[$i][0] = substr($out[$i], 1, 17);
	    	$out2[$i][1] = substr($out[$i], 18, strlen($out[$i])-18);
		echo '<option value="'.$out2[$i][0].'">';
		echo $out2[$i][1].' ('.$out2[$i][0].')';
		echo '</option>';
	}

	echo '<option>NOTHING FOUND</option>';
}


function rfcomm_list_binded(){
	$out = array();
	exec('rfcomm -a', $out);
	echo '<ul>';
	for ($i=0; $i<sizeof($out); $i++) {
		$rfend = strpos($out[$i], ':');
		$rfcomm_name = substr($out[$i], 0, $rfend);
		echo '<li>
			'.$out[$i].'&nbsp;&nbsp;&nbsp;
				<div 
					style="display:inline; border:1px solid red;"
					onClick=bt_disconnect("'.$rfcomm_name.'");>
					Disconnect
				</div>
			</li>';
	}
	echo '</ul>';
	unset($out);
	echo '<ul>';

/*	for ($i=0; $i<sizeof($out); $i++) {
		$position = strpos($out[$i], 'rfcomm');
		if (strpos($out[$i], 'oot')>0 && $position==0) {
			$startpos = digit_offset($out[$i]);
			$psid = substr($out[$i], $startpos,5);
			echo '<li>
				'.$out[$i].'
					<div 
						style="border:1px solid red; display:inline;" 
						onClick=stopSerialRead('.$psid.')>
						Kill '.$psid.'</div></li>';
		}
	} */
	echo '</ul>';
}

function digit_offset($text){
    preg_match('/^\D*(?=\d)/', $text, $m);
    return strlen($m[0]);
}


function rfcomm_bind($name, $mac, $channel){
	$out = array();
	$out = exec('rfcomm -r bind /dev/'.$name.' '.$mac.' '.$channel);
}

function rfcomm_release($name){
	$out = array();
	$out = exec('rfcomm release /dev/'.$name);	
}


function rfcomm_connect($rfcomm_name, $mac){
	$out = array();
	$cmd = './bt_expect.exp '.$rfcomm_name.' '.$mac.' > /dev/null &';
	$cmd = 'id';
	$out = exec($cmd);
	echo $cmd.'<br>';
	print_r($out);
}

// kill the process with name "rfcomm"
function rfcomm_kill(){
	$out = array();
	exec('kill -9 $(pidof rfcomm)');
	print_r($out);
}


function read_file($file, $lines) {
       $handle = fopen($file, "r");
       $linecounter = $lines;
       $pos = -2;
       $beginning = false;
       $text = array();
       while ($linecounter > 0) {
         $t = " ";
         while ($t != "\n") {
           if(fseek($handle, $pos, SEEK_END) == -1) {
		$beginning = true;
		break;
	   }
           $t = fgetc($handle);
           $pos --;
         }
         $linecounter --;
         if($beginning) {
		rewind($handle);
	 }
         $text[$lines-$linecounter-1] = fgets($handle);
         if($beginning) {
		break;
	 }
       }
       fclose ($handle);
       return array_reverse($text); // array_reverse is optional: you can also just return the $text array which consists of the file's lines.
}

function display_timers(){
	$sca = get_settings_arr(null);

	// print_r($sca);
	foreach ($sca as $key=>$row) {
		$sca2[$row[0]]['value'] = $row[2];
	}
	$sca = $sca2;
	unset($sca2);
	$offset_timer1 = 1498;
	$size_timer = 5;
	$offset_timer_on = 0;
	$offset_timer_off = 2;
	$offset_timer_flags = 4;
	$timers_amount = 4;


	for ($i=0; $i<$timers_amount;$i++) {
		$offset = $offset_timer1+$i*$size_timer;
		$addr_tmrs_on = $offset;
		$addr_tmrs_off = $offset+2;
		
		echo '
		<tr>
			<td>Timer '.($i+1).'</td>
			<td>&nbsp;ON
				<input 
					onChange="rx_ee_this(this)" 
					name="csx_'.$addr_tmrs_on.'_value" 
					type="text" 
					value="'.$sca[$addr_tmrs_on]['value'].'" 
				/>
				
			</td>
			<td>&nbsp;OFF
				<input 
					onChange="rx_ee_this(this)" 
					name="csx_'.$addr_tmrs_off.'_value" 
					type="text" 
					value="'.$sca[$addr_tmrs_off]['value'].'" 
				/>
			</td>
			<td>
				
			</td>
		</tr>';
	}

}

function get_sm_csv(){
	$SEMAPHORE_KEY = 674839575;   			//Semaphore unique key
	$SHARED_MEMORY_KEY = 910538672;   	//Shared memory unique key

	$rxbs = 1000;	// rx buffer size in shared memory
	$w_pa = 1400;	// read pointer offset in shared memory
	$csv_os = 100;	// CSV string address offset in shared memory
	$cadi_packet_size = 42;	// maximum size of Cadi packet

	//Setup access to the shared memory
	$shared_memory_id = shmop_open($SHARED_MEMORY_KEY, "w", 0, 0);	//Shared memory key, flags, permissions, size (permissions & size are 0 to open an existing memory segment)
	$sm_size = shmop_size($shared_memory_id);




	//We have exclusive access to the shared memory (the other process is unable to aquire the semaphore until we release it)
	if (empty($shared_memory_id)){
		echo "Failed to open shared memory.<br />";	//<<<< THIS WILL HAPPEN IF THE C APPLICATION HASN'T CREATED THE SHARED MEMORY OR IF IT HAS BEEN SHUTDOWN AND DELETED THE SHARED MEMORY
	}
	else {
		//--------------------------------------------
		//----- READ AND WRITE THE SHARED MEMORY -----
		//--------------------------------------------
		// echo "Shared memory size: ".shmop_size($shared_memory_id)." bytes<br />";


		//----- READ FROM THE SHARED MEMORY -----
		$csv = shmop_read($shared_memory_id, $csv_os, 100);   	//Shared memory ID, Start Index, Number of bytes to read
		if($csv == FALSE) {
			echo "Failed to read shared memory";
			exit;
		}

	}

//Detach from the shared memory
shmop_close($shared_memory_id);

return $csv;

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
	
	$sm_str;
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








?>
