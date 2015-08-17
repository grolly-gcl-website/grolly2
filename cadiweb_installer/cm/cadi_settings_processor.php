<?php 
if (session_status() == PHP_SESSION_NONE) {
    session_start();
}

if (isset($_GET['action'])) {
	$action = $_GET['action'];
}
else {
	$action = $_POST['action'];
}


switch ($action) {
	case 'tank_settings':
		$strarr[0] = 'watertank_top_bottom';
		$strarr[1] = $_POST['t3top'];	// fill the new top/bottom values into settings array
		$strarr[2] = $_POST['t3btm'];
		$strarr[3] = $_POST['t4top'];
		$strarr[4] = $_POST['t4btm'];
		$string = implode(',',$strarr);
		cs_replace_string($string);
		break;
	case 'psi_settings':
		$strarr[0] = 'psi_sensor';
		$strarr[1] = $_POST['psi0psi'];	// fill the new top/bottom values into settings array
		$strarr[2] = $_POST['psi32psi'];
		$string = implode(',',$strarr);
		cs_replace_string($string);
		break;
	case 'valve_settings':
		
		break;

}

// replaces full CSV string in Cadiweb settings file. String to replace found by first field match
function cs_replace_string($string){
		$string_arr = explode(',',$string);
		$string_name = $string_arr[0];
		$row = 0;
		if (($handle = fopen("cadi_settings", "r")) !== FALSE) {
		    // read a copy of file into array
		    while (($settings_arr[$row] = fgetcsv($handle, 1000, ",")) !== FALSE) {
				$row++;
		    }
		    fclose($handle);
		}
		foreach ($settings_arr as $key=>$rowarr) {
			if ($rowarr[0] == $string_name) {	// find the config line for tank top/bottom settings
				// found the $key needed
				$keyfound = $key;
			}
		}
		$settings_arr[$keyfound] = $string_arr;
		
		file_put_contents('cadi_settings',' ');	// flush settings file

		if (($handle = fopen("cadi_settings", "w")) !== FALSE) {
			foreach ($settings_arr as $rows) {	// fill the new config file up
				fputcsv($handle, $rows);
			}
			fclose($handle);
		}
}


?>
