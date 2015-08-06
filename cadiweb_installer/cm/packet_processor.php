<?php

include_once('cadi_settings.php');


function crc_block_ord($input, $ord_arr, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input = 111;
	}
	return $input;
}

function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
} 

// extracts 16byte block of settings dump file to send to Cadi
function dump2block($addr){
	global $settings_startaddr;
	$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
	fseek($fp, ($addr*2),0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,32); // read settings dump data
	fclose($fp); // close the file
	$settings_hex = '';
	for ($i=0; $i<(strlen($block_data)/2);$i++) {
		// swap
		$temp = $settings_dump[$i*2];
		$settings_dump[$i]=$settings_dump[($i*2+1)];
		$settings_dump[($i*2+1)] = $temp;
	}
	return $settings_dump;
}
	
$packet;		// declare variable
$packet = null;		// init variable

if (isset($_POST['cmd'])) {
	$packet_pref = "ZX2";
	$packet.=$packet_pref;
	$cmd = $_POST['cmd'];
	switch($cmd) {

		case 101:

			break;
		case 102:
			$packet .= chr(5);	// packet payload size
			$packet .= chr(102);
			$packet .= chr(floor($_POST['duration']/256));		// watering duration HSB
			$packet .= chr($_POST['duration']%256);		// LSB
			$packet .= chr($_POST['line_id']%256);	// id of output line
			break;
		case 103:
			$packet .= chr(4);	// packet payload size
			$packet .= chr(103);
			$packet .= chr(floor($_POST['duration']/256));		// duration of mixing HSB
			$packet .= chr($_POST['duration']%256);		// LSB
			break;
		case 104:
			$packet .= chr(8);	// packet payload size
			$packet .= chr(104);
			$packet .= chr(floor($_POST['duration']/256));		// duration of mixing HSB
			$packet .= chr($_POST['duration']%256);		// LSB
			$packet .= chr(floor($_POST['volume']/256));		// volume of fertilizer to mix-in (HSB)
			$packet .= chr($_POST['volume']%256);		// LSB
			$packet .= chr($_POST['doserid']%256);	// doserId
			$packet .= chr($_POST['speed']%256);	// doser speed (percents: 40% to 100%)
			echo PHP_EOL.$_POST['volume'].PHP_EOL.$_POST['doserid'].PHP_EOL.$_POST['speed'].PHP_EOL.$_POST['duration'];
			break;
		


		case 115:	// prepare Settings Upload Packet
			// first, determine which type the data is
			// there are 3 data types used: 8-bit, 16-bit and 32-bit
			$valarr = get_csv_value($_POST['addr']);
			$type = $valarr[1];
			$value = $valarr[2];
			$description = $valarr[3];
			switch ($type) {
				case 1:		// 8-bit value (ZX3,size,cmdid,addrH,addrL,parity,val,crc,paketid)
					$parity = substr($_POST['addr'],4,1);	// get parity (0:Higher or 1:Lower)

					$packet .= chr(6);	// payload size
					$packet .= chr(31);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address higher byte
					$packet .= chr($_POST['addr']%256);		// address lower byte
					$packet .= chr($parity);			// 0:Higher or 1:Lower
					$packet .= chr($value);				// value
					
					break;
				case 2:		// 16-bit value
					$packet .= chr(6);	// payload size
					$packet .= chr(32);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address Higher byte
					$packet .= chr($_POST['addr']%256);		// address Lower byte
					$packet .= chr(floor($value/256));		// VAL: Higher byte
					$packet .= chr($value%256);			// VAL: Lower byte
					break;
				case 3:		// 32-bit value
					$packet .= chr(8);	// payload size
					$packet .= chr(33);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address Higher byte
					$packet .= chr($_POST['addr']%256);		// address Lower byte
					$packet .= chr(($value>>24)&0xFF);		// VAL: Higher byte
					$packet .= chr(($value>>16)&0xFF);			// VAL: Lower byte
					$packet .= chr(($value>>8)&0xFF);		// VAL: Higher byte
					$packet .= chr($value&0xFF);			// VAL: Lower byte
					break;

			}

			break;

		case 0:		// get_water(8,8,16)
			$packet .= chr(6);	// packet payload size
			$packet .= chr(0);
			$packet .= chr($_POST['valve']);		// add valve_id
			$packet .= chr($_POST['counter']);		// counter_id
			$packet .= chr(intval($_POST['gw_amount']/256));	// first byte of uint16_t amount (of water)
			$packet .= chr($_POST['gw_amount']%256);		// second byte

			break;
		case 1:		// plugStateSet(8,8)
			$packet .= chr(4);	// packet payload size (Payload starts here)
			$packet .= chr(1);	//  Command ID
			$packet .= chr($_POST['plug']);		 // Plug ID  (command arguments)
			$packet .= chr($_POST['state']);	// new Plug State (Payload ends here)
			break;
		case 2:		// Direct drive enable
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(2);	// command
			break;
		case 3:		// Direct drive disable
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(3);	// command
			break;
		case 4:		// open_valve()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(4);	// command
			$packet .= chr($_POST['valve']);	// valve_id
			break;
		case 5:		// close_valve()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(5);	// command
			$packet .= chr($_POST['valve']);	// valve_id
			break;
		case 6:		// get_settings_block()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(6);	// command
			$packet .= chr($_POST['block_id']);	// SETTINGS block id
			break;

		case 8:		// set auto_flags
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(8);	// command
			$packet .= chr($_POST['flags']);	// new auto_flags byte
			break;
/*		case 9:		// dosing pump enable/disable
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(9);	// command
			$packet .= chr($_POST['pump_id']);	// pumpId for doser
			$packet .= chr($_POST['state']);	// new state for doser
			break;	*/
		case 9:		// dosing pump fertilizer intake in seconds
			$packet .= chr(5);	// packet payload size (including this size byte)
			$packet .= chr(9);	// command
			$packet .= chr($_POST['pump_id']);	// pumpId for doser
			$packet .= chr($_POST['amount']);	// in seconds
			$packet .= chr($_POST['speed']);	// in %, 1 - enabled full
			break;
		case 10:		// set new valve_failed 8bit value
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(10);	// command
			break;
		case 11:		// stop all processes and force manual control
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(11);	// command
			break;
		case 12:		// set time
			$curtime = time();
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(12);	// command
			$packet .= chr(floor($curtime/16777216));	// command
			$packet .= chr((($curtime%16777216)/65536));	// command
			$packet .= chr((($curtime%65536)/256));	// command
			$packet .= chr($curtime%256);	// command
			echo $curtime;
			break;
		case 13:		// get_status_block();
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(13);	// command
			break;
		case 15:		// rx_ee((RxBuffer[2]&(RxBuffer[3]<<8)), 1); writes 16bit variable into STM32's Emulated EEPROM
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(15);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$packet .= chr($_POST['value']%256);	// value lower byte
			$packet .= chr(floor($_POST['value']/256));	// higher byte
			break;
		case 16:		// rx_ee((RxBuffer[2]&(RxBuffer[3]<<8)), 16); writes 32 byte (16 vars) block into STM32's Emulated EEPROM
			$packet .= chr(20);	// packet payload size (including this size byte)
			$packet .= chr(16);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$settings_block_data = dump2block($_POST['addr']);
			$packet .= $settings_block_data;
			break;
		case 17:		// 8 bit value send to Cadi Higher byte
			$packet .= chr(5);	// packet payload size
			$packet .= chr(17);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$packet .= chr($_POST['value']);	// value lower byte
			break;
		case 18:		// 8 bit value send to Cadi Lower Byte
			$packet .= chr(5);	// packet payload size
			$packet .= chr(18);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$packet .= chr($_POST['value']);	// value lower byte
			break;
		case 19:		// loadSettings() STM32 function call
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(19);	// command
			break;

		case 26:	//
			
			break;

		case 34:			// RUN_DEMO	
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(34);	// command
			break;

		case 35:			
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(35);	// command - RUN WATERING PUMP
			$packet .= chr($_POST['secs']%256);	// address lower byte
			break;

		case 37:			
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(37);	// command - CLOSE_VALVES
			break;

		case 38:
			$packet .= chr(6);	// 
			$packet .= chr(38);	// mix solution in MixTank
			$packet .= chr(floor($_POST['secs']/16777216));	// number of seconds, 32 bit
			$packet .= chr((($_POST['secs']%16777216)/65536));	// 
			$packet .= chr((($_POST['secs']%65536)/256));	// command
			$packet .= chr($_POST['secs']%256);	// command

// Above 50th there are commands that send successfull execution confirmation "ZX7"
		case 51:		// get_status_block();
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(51);	// command
			$packet .= chr($_POST['block_id']);	// STATUS block_id
			break;
		case 58:		// EE_ReadVariable(uint32_t addr)
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(58);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			break;
		case 59:		// EE_ReadVariable(uint32_t addr)
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(59);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			break;
		case 63:		// get_settings_dump_(uint16_t startaddr, uint16_t amount)
			if ($_POST['amount']==0) {
				$_POST['amount'] = 512;		// DEFAULT HARDCODE
			}
			if ($_POST['addr']==0) {		
				$_POST['addr'] = 1400;		// DEFAULT HARDCODE
			}
			
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(63);	// command
			$packet .= chr($_POST['amount']%256);	// address lower byte
			$packet .= chr(floor($_POST['amount']/256));	// higher byte
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			break;
		case 64:	// send EE addr value
			include_once('gen_fcm.php');
			put_settings_val($_POST['addr'], $_POST['value']);
			sync_conf2dump();
			echo 'Packing '.$_POST['value'].PHP_EOL;
			$value_arr = get_csv_value($_POST['addr']);
			$value_arr[2] = $_POST['value'];	// update array with new value
			$type = $value_arr[1];
			$value = $value_arr[2];
			echo 'De-Packing '.PHP_EOL;
			print_r($value_arr);
			$description = $value_arr[3];
			switch ($type) {
				case 1:		// 8-bit value (ZX3,size,cmdid,addrH,addrL,parity,val,crc,paketid)
					$parity = substr($_POST['addr'],4,1);	// get parity (0:Higher or 1:Lower)

					$packet .= chr(6);	// payload size
					$packet .= chr(31);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address higher byte
					$packet .= chr($_POST['addr']%256);		// address lower byte
					$packet .= chr($parity);			// 0:Higher or 1:Lower
					$packet .= chr($value);				// value
					
					break;
				case 2:		// 16-bit value
					$packet .= chr(6);	// payload size
					$packet .= chr(32);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address Higher byte
					$packet .= chr($_POST['addr']%256);		// address Lower byte
					$packet .= chr(floor($value/256));		// VAL: Higher byte
					$packet .= chr($value%256);			// VAL: Lower byte
					break;
				case 3:		// 32-bit value
					$packet .= chr(8);	// payload size
					$packet .= chr(33);	// cmdid
					$packet .= chr(floor($_POST['addr']/256));	// address Higher byte
					$packet .= chr($_POST['addr']%256);		// address Lower byte
					$packet .= chr(($value>>24)&0xFF);		// VAL: Higher byte
					$packet .= chr(($value>>16)&0xFF);			// VAL: Lower byte
					$packet .= chr(($value>>8)&0xFF);		// VAL: Higher byte
					$packet .= chr($value&0xFF);			// VAL: Lower byte
					break;

			}
			break;

	}

	// SOME HARDCODE
	$packet[3] = chr(ord($packet[3])+1);	// increase packet length byte for CRC use (TEMPORARY solution, use fixed values when stable)
	$packet[3] = chr(ord($packet[3])+4);	// Protobuzzz v3: Packet length = full packet size

//	$packet[3] = chr(ord($packet[3])+4);	// Protobuzzz v3: Packet length = full packet size

	
//	$packet[3] = chr(ord($packet[3])+1);	// increase packet length byte for packet ID use (same story as before)
								// Packet ID is being generated and added to the end of packet within Cadi BTDaemon

	// CRC
	$curcrc = chr(crc_block(0, $packet, (strlen($packet))));	// calculate XOR CRC checksum for the packet
	$packet .= $curcrc;	// add CRC to packet
	

	// packet for Shared Memory
	
	$packet_crc = chr(crc_block(0,$packet,(strlen($packet))));
	$out = $packet.$packet_crc;
	$packet4sm = $out;


	// packet id will be added within daemon runtime

	// convert packet into "echo -e" Linux command usable format

/*	not used since Protobuzzz v3
	unset($arguments);
	for ($i=0; $i<strlen($packet);$i++) {
		$arguments .= sprintf("\\x%02x",ord($packet[$i]));
	}
	unset($packet); 
	$packet = $arguments; */





	$cadi_packet = $arguments;



	unset($packet_hex);
	for ($i=0; $i<strlen($packet4sm);$i++) {
		$packet_hex .= sprintf("\\x%02x",ord($packet4sm[$i]));
	}
	
	echo $packet_hex;

}

else {
	$cadi_packet = "empty packet";
}


function crcs($s){	// generate xor crc checksum
	$curxor = 0;
	for ($i=0;$i<strlen($s);$i++){
		$curxor ^ $s[$i];
	}
	return $curxor;
}


function get_dump_val($addr){


}




?>
