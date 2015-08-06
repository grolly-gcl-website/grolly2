<?php
/*
=== Cadiweb Statistics Module: WP Run Parser ===
	This program extracts the Watering Program runs into csv log cuts,
SVG drawings and report files
*/

//if (!extension_loaded('php_gd2')) {
 //   if (strtoupper(substr(PHP_OS, 0, 3)) === 'WIN') {
        
 //   } else {
 //       dl('php_gd2.dll');
 //   }
//}


// 'process' triggers responsible for running parser blocks
$proc_svg = 1;
$proc_csv = 1;
$proc_csv_grande = 1;
$proc_reports = 1;
$proc_osd = 1;

ini_set ("display_errors", "1");
error_reporting(E_ALL);

date_default_timezone_set("UTC");

dl('php_gd2.dll');

$colorarr = array(1 => "FF0000", "00FF00", "0000FF", "FFFF00", "FF00FF", "00FFFF", "000000", 
        "800000", "008000", "000080", "808000", "800080", "008080", "808080", 
        "C00000", "00C000", "0000C0", "C0C000", "C000C0", "00C0C0", "C0C0C0", 
        "400000", "004000", "000040", "404000", "400040", "004040", "404040", 
        "200000", "002000", "000020", "202000", "200020", "002020", "202020", 
        "600000", "006000", "000060", "606000", "600060", "006060", "606060", 
        "A00000", "00A000", "0000A0", "A0A000", "A000A0", "00A0A0", "A0A0A0", 
        "E00000", "00E000", "0000E0", "E0E000", "E000E0", "00E0E0", "E0E0E0");

$infilename = 'csvout.csv';
$outfilename = 'wp_prog_out.csv';

$block_size = 500000;
$infilesize = filesize($infilename);
$blocks_to_read = ($infilesize/$block_size) + 1;
$wplength = 100;		// number of seconds to extract after wp state became 0
$last_wp_on = 0;		// timestamp of the last second WP seen enabled
$row = 1;
$i = 0;
$outhandle = fopen($outfilename, "r");
$header = 'time,tim1,tim2,tim3,tim4,v0,v1,v2,v3,v4,wpState,snar1,sonar2,adc1,dadc2,adc3,adc4,psi,comm_state,doser1,doser2,doser3,doser4,auto_flags,wpProgress,auto_failures,runners,psi_state'.PHP_EOL;


if (($handle = fopen($infilename, "r")) !== FALSE) {
	// iterate over each CSV liine
    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
        $row++;
	$now = $data[0];
	$curstate = $data[7];		// field to monitor
	if ($curstate>$prevstate) {	// detected rising edge
		fclose($outhandle);	// close previous
		$file_no++;		// increase output file number
		$file_timestamp = 0;	// set file timestamp
	
		// filenames for CSV cutout
		$exflnm = $flnm;
		$flnm = $now.'_'.$file_no.'_'.$outfilename;
		// filenames for report
		$exflnm_wp_report_tbl = $flnm_wp_report_tbl;
		$flnm_wp_report_tbl = $now.'_report';
		$header = 'time,tim1,tim2,tim3,tim4,v0,v1,v2,v3,v4,wpState,snar1,sonar2,adc1,dadc2,adc3,adc4,psi,comm_state,doser1,doser2,doser3,doser4,auto_flags,wpProgress,auto_failures,runners,psi_state'.PHP_EOL; //possibly not needed anymore
		// start new file
		file_put_contents($flnm, $header);	// start new WP csv cut file
		$outhandle = fopen($flnm, "w");
		fwrite($outhandle,$header);
		// generate SVG image
		$n = 0;
		$i = 0;
		unset($outsvg);
		$outsvg = '<!DOCTYPE html>
				<html>
				<body>';

		$outsvg .= '<svg width="400px" height="400px" viewBox="0 0 '.sizeof($svg_arr).' 1100" preserveAspectRatio="none">
		';
		$outsvg_grande .= '<svg width="400px" height="400px" viewBox="0 0 '.sizeof($svg_arr).' 1100" preserveAspectRatio="none">
		';
		$outsvg .= '<text x="0" y="15" fill="red">'.$datetime.'</text>';
		$outsvg_grande .= '<text x="0" y="15" fill="red">'.$datetime.'</text>';
		foreach ($svg_arr as $key=>$row){
			while ($n<25) {
				$line[$n++] .= $key.','.(500-$row[$n]).' ';
			}
			if ($key%5==0) {		// every 5 seconds set vertical separator peak
				$line['axis'] .= $key.',510 '.$key.',0 '.$key.',510 ';
			}
			else {
				$line['axis'] .= $key.',510 ';
			}
			$n = 0;
		}
		
		while ($n<25) {
			$outsvg .= PHP_EOL.'
			
				<polyline points="'.$line[$n].'"
 					fill-opacity="0"
					fill="rgb(249,249,249)"
					stroke-width="1"
					stroke="#'.$colorarr[$n].'"/>
			';
	// <<<<<< GRAND TOTAL SVG BLOCK
			$outsvg_grande .= PHP_EOL.'
			
				<polyline points="'.$line[$n].'"
 					fill-opacity="0"
					fill="rgb(249,249,249)"
					stroke-width="1"
					stroke="#'.$colorarr[$n].'"/>
			';
	// >>>>>>>
			unset($line[$n]);
			$n++;
		}
	// <<<<<< GRAND TOTAL SVG BLOCK
		$outsvg_grande .=
			PHP_EOL.'
			<polyline points="'.$line['axis'].'"
 				fill-opacity="0"
				fill="rgb(249,249,249)"
				stroke-width="1"
				stroke-opacity="0.2"
				stroke="#000000"/></svg>
		';
	// >>>>>>>

		$outsvg .= PHP_EOL.'
			<polyline points="'.$line['axis'].'"
 				fill-opacity="0"
				fill="rgb(249,249,249)"
				stroke-width="1"
				stroke-opacity="0.2"
				stroke="#000000"/>
		';
		unset($line['axis']);
		$outsvg .= '</svg></body>
				</html>
		';
		
		file_put_contents(($exflnm.'.php'), $outsvg);

	/////// REPORT file write
	$report_contents = 'd1,d2,d3,d4,v0,v1,v2,v3,v4,psiover0,wateramount'.PHP_EOL;	// start report with csv fields header
	$report_contents .= 'Valves: '.implode(',', $counters['valves']).PHP_EOL;
	$report_contents .= 'Dosers: '.implode(',', $counters['dosers']).PHP_EOL;
	$report_contents .= 'watering: '.$counters['psi_over0'].' seconds'.PHP_EOL;
	$report_contents .= 'Total duration: '.$counters['total'].' seconds'.PHP_EOL;
	$report_contents .= 'Water amount intaken: ';
	$water_intaken = $start_wateramount-$data[9];
	if ($water_intaken>3) {
		$report_contents .= $water_intaken;
	}
	else {
		$report_contents .= '0';
	}
	echo PHP_EOL;
	file_put_contents($exflnm_wp_report_tbl,$report_contents);
		for ($j = 0; $j<5;$j++) {
			$counters['valves'][$j] = 0;
			$counters['dosers'][$j] = 0;
		}
	$start_wateramount = $data[9];
	$counters['psi_over0'] = 0;
	$counters['total'] = 0;
		
	/////// EOF REPORT file write



		unset($svg_arr);
		unset($outsvg);
		echo 'Now = '.$now.PHP_EOL;
		echo 'outsvg strlen = '.strlen($outscg).PHP_EOL;
		$datetime = date("Y-m-d H:i:s", $now);	
	}
	if ($curstate>0) {		// if monitored field active
		$last_wp_on = $now;	// increase tail pointer
	}
	if (($last_wp_on+$wplength)>$now) {

		$rarr[0] = substr($data[3],0,1);	// timer 1
		$rarr[1] = substr($data[3],1,1);
		$rarr[2] = substr($data[3],2,1);
		$rarr[3] = substr($data[3],3,1);
		$rarr[4] = substr($data[5],0,1);	// valve 0
		$rarr[5] = substr($data[5],1,1);
		$rarr[6] = substr($data[5],2,1);
		$rarr[7] = substr($data[5],3,1);
		$rarr[8] = substr($data[5],4,1);	// valve 4
		$rarr[9] = substr($data[16],0,1);	// doser 0
		$rarr[10] = substr($data[16],1,1);	// doser 1
		$rarr[11] = substr($data[16],2,1);	// doser 2
		$rarr[12] = substr($data[16],3,1);	// doser 3
		
		for ($j = 0; $j<5;$j++) {
			$counters['valves'][$j]+=$rarr[(4+$j)];
		}
		for ($j = 0; $j<4;$j++) {
			$counters['dosers'][$j]+=$rarr[(9+$j)];
		}
		if ($data[14]>1) {
			$counters['psi_over0']++;
		}

		$counters['total']++;
		


		// extract this record
		$bin_x = 7;
		$tofile[0] = $data[0];				// timestamp
		$tofile[1] = $rarr[0]*$bin_x+110;		// timer 1
		$tofile[2] = $rarr[1]*$bin_x+120;
		$tofile[3] = $rarr[2]*$bin_x+130;
		$tofile[4] = $rarr[3]*$bin_x+140;		// timer 4
		$tofile[5] = $rarr[4]*$bin_x+150;		// valve 0
		$tofile[6] = $rarr[5]*$bin_x+160;
		$tofile[7] = $rarr[6]*$bin_x+170;
		$tofile[8] = $rarr[7]*$bin_x+180;
		$tofile[9] = $rarr[8]*$bin_x+190;		// valve 4
		$tofile[10] = $rarr[9]*$bin_x+200;		// doser 0
		$tofile[11] = $rarr[10]*$bin_x+210;		// doser 1
		$tofile[12] = $rarr[11]*$bin_x+220;		// doser 2
		$tofile[13] = $rarr[12]*$bin_x+230;		// doser 3
		$tofile[14] = 0;	
		// $tofile[14] = $data[7];				// wpState		
		$tofile[15] = $data[8];				// sonar_read[0]
		$tofile[16] = $data[9]+100;			// sonar_read[1]
		$tofile[17] = $data[10]/85+250;			// adc1
		$tofile[18] = $data[11]/85+300;			// adc2
		$tofile[19] = $data[12]/85+350;			// adc3
		$tofile[20] = $data[13]/85+400;			// adc4
		$tofile[21] = $data[14];			// psi
		$tofile[22] = $data[15];			// comm_state
		$tofile[23] = $data[17];			// auto_flags
		$tofile[24] = $data[18];			// wpProgress
		$tofile[25] = $data[19];			// auto_failures
		$tofile[26] = $data[20];			// runners
		$tofile[27] = $data[21];			// psi_state
		// $data2 = implode(",", $tofile);

		$svg_arr[$i++]=$tofile;
		fputcsv($outhandle, $tofile);
	}
	else {
		// skip record
		
	}
	$prevstate = $data[7];
        
    }
    fclose($outhandle);
    fclose($handle);
    $outsvg_grande2 =  '<!DOCTYPE html>
				<html>
				<body>'.$outsvg_grande.'</body></html>';


	file_put_contents('full.svg',$outsvg_grande2);
}

?>
