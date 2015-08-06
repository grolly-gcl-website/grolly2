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
		$temp = file_get_contents('svg.conf');
		$data = explode(",", $temp);
		$data[0] = $_POST['t3top'];
		$data[1] = $_POST['t3btm'];
		$data[4] = $_POST['t4top'];
		$data[5] = $_POST['t4btm'];
		$out = implode($data,",");
		file_put_contents('svg.conf', $out);
		echo $out;
		break;
	case 'valve_settings':
		
		break;
}

?>
