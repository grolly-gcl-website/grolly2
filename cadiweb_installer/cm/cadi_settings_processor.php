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
		$row = 0;
		if (($handle = fopen("cadi_settings", "r")) !== FALSE) {
		    // read a copy of file into array
		    while (($settings_arr[$row] = fgetcsv($handle, 1000, ",")) !== FALSE) {
				$row++;
		    }
		    fclose($handle);
		}
		foreach ($settings_arr as $key=>$rowarr) {
			if ($rowarr[0] == 'watertank_top_bottom') {	// find the config line for tank top/bottom settings
				// found the $key needed
				$keyfound = $key;
			}
		}
		$settings_arr[$keyfound][1] = $_POST['t3top'];	// fill the new top/bottom values into settings array
		$settings_arr[$keyfound][2] = $_POST['t3btm'];
		$settings_arr[$keyfound][3] = $_POST['t4top'];
		$settings_arr[$keyfound][4] = $_POST['t4btm'];
		
		file_put_contents("cadi_settings",' ');	// flush settings file

		if (($handle = fopen("cadi_settings", "w")) !== FALSE) {
			foreach ($settings_arr as $rows) {	// fill the new config file up
				fputcsv($handle, $rows);
			}
			fclose($handle);
		}


		break;
	case 'valve_settings':
		
		break;

}



?>
