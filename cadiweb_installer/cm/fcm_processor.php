<?php
if (session_status() == PHP_SESSION_NONE) {
    session_start();
}
include_once('gen_fcm.php');

if (isset($_GET['action'])) {
	$action = $_GET['action'];
}
else {
	$action = $_POST['action'];
}


switch ($action) {
	case '1':	// generate flags block
		$block_id = $_POST['blk_id'];
		gen_flags_block($block_id);
		break;
	case '':

		break;

}

?>
