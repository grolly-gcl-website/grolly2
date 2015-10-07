<?php
/**
 * This file is meant to be included in /index.php.
 * It prints some of the JavaScript variables
 */

$ROOT_PATH = dirname(dirname(__FILE__));
$CADI_SETTINGS_FILE = $ROOT_PATH . '/cm/cadi_settings';

// load tank levels settings
$row = 1;
if (($handle = fopen($CADI_SETTINGS_FILE, "r")) !== FALSE) {
    while (($rowarr = fgetcsv($handle, 1000, ",")) !== FALSE) {
        $num = count($_SESSION['settings_data'][$row]);
        if (substr($rowarr[0],0,1)!='#') {
            $_SESSION['settings_data'][($rowarr[0])] = $rowarr;
            $row++;
        }
    }
    fclose($handle);
}

$svg_t3top = $_SESSION['settings_data']['watertank_top_bottom'][1];
$svg_t3btm = $_SESSION['settings_data']['watertank_top_bottom'][2];
$svg_t4top = $_SESSION['settings_data']['watertank_top_bottom'][3];
$svg_t4btm = $_SESSION['settings_data']['watertank_top_bottom'][4];    
$psi0psi_ = $_SESSION['settings_data']['psi_sensor'][1];
$psi32psi_ = $_SESSION['settings_data']['psi_sensor'][2];

$js_svg_valves = 'var svg_valves = [';

for ($i=1;$i<10;$i++) {
    $js_svg_valves .= $_SESSION['settings_data']['svg_valves'][$i].',';
}

$js_svg_valves .= $_SESSION['settings_data']['svg_valves'][10].'];';


function print_vars() {
    global $psi0psi_, $psi32psi_, $js_svg_valves;
    echo 'var psi0psi_ = '.$psi0psi_.';'.PHP_EOL;
    echo 'var psi32psi_ = '.$psi32psi_.';'.PHP_EOL;
    echo $js_svg_valves;
}

?>
