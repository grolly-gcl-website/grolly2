<?php

echo exec('dir');

$start = 1424810359;
$end =   1425152071;


$dir = "lapse3";



$reportarr['watering_duration'] = 0;
$reportarr['water_intaken'] = 0;
$reportarr['time'] = 0;
$reportarr['fresh_solution'] = 0;
for ($i=0;$i<5;$i++) {
	$reportarr['dosers'][$i] = 0;
	$reportarr['valves'][$i] = 0;	
}




for ($i=$start; $i<$end;$i++) {
	$flnm_wp_report_tbl = $i.'_report';
	$filename = 'sht'.$i.'.jpg';
	if (file_exists($flnm_wp_report_tbl)){		// if report found
		echo '===== New report for :'.$flnm_wp_report_tbl.PHP_EOL;
		new_osd_layer($i);
	}
	if (file_exists($dir.'/'.$filename)){
	//	echo $filename.PHP_EOL;
		apply_osd_to_jpg(($dir.'/'.$filename), $i);
			
		
	}
	else {
		// echo 'Probing failed '.$filename.PHP_EOL;

	}
}

function apply_osd_to_jpg($filename, $timestamp){
	global $reportarr, $dir;
	echo 'OSDing image '.$timestamp.PHP_EOL	;
	// Создание изображения 300x100
	// $im = imagecreatetruecolor(300, 100);
	$im = imagecreatefromjpeg($filename);
	//$red = imagecolorallocate($im, 0xFF, 0xFF, 0xCC);
	$bg_color = imagecolorallocatealpha($im, 222, 222, 222, 97);
	$black = imagecolorallocatealpha($im, 0, 0, 0, 25);
	$white = imagecolorallocatealpha($im, 255, 255, 255, 25);

//	echo '<<<<<<<<<<< REPORT ARR'.PHP_EOL;
//	print_r($reportarr);

	// Сделаем краный фон
	imagefilledrectangle($im, 0, 0, 199, 199, $bg_color);

	// Путь к ttf файлу шрифта
	$font_file = './cousine-regular.ttf';
	$font_file_bold = './cousine.bold.ttf';


	$font_size = 10;	// text font size
	$line_space = 12;	// text lines spacing
	$left_margin = 2;	// text margin from left border


	$wp_run_time_ago = intval(($timestamp-$reportarr['time'])/60);	// when last WP happened (minuetes ago)
	$solution_age = intval(($timestamp-$reportarr['solution_birth'])/60);
	date_default_timezone_set("UTC");
	$photo_time = date("Y-m-d H:i:s", $timestamp);


	if ($reportarr['fresh_solution']>0) {
		$freshness_text = 'FRESH soluton';	
	}
	else {

	}



	$i = 2;


	// Рисуем текст 'PHP Manual' шрифтом 13го размера BOLD
	imagefttext($im, $font_size, 0, $left_margin, 11, $black, $font_file_bold, 'T: '.$photo_time);

	// Рисуем текст 'PHP Manual' шрифтом 13го размера BOLD
	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Last wtrng: '.$wp_run_time_ago.' mins ago');

	$i++;

	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Duration: '.$reportarr['watering_duration'].' secs');

	$i++;

	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Solution age:'.$solution_age.' mins');


	$i++;

	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Water amount:'.$reportarr['water_intaken'].' cms');


	$i++;

	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Grow:'.$reportarr['dosers'][1].' secs');

	$i++;

	imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Bloom:'.$reportarr['dosers'][0].' secs');





	imagejpeg($im, ($dir.'/out/out'.$timestamp.'.jpg'));
	imagedestroy($im);

//	print_r($reportarr);
}

function new_osd_layer($timestamp){
	$filename = $timestamp.'_report';
	global $reportarr;
	$reportarr['time'] = $timestamp;
	// $flcontents = file_get_contents($filename);
	echo '>>>>>>>> opening file '.$filename;
	$i=0;
	if (($handle = fopen($filename, "r")) !== FALSE) {
		while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
			$tmparr[$i++] = $data;
		}
	}
	
	$tmparr[1][0] = substr($tmparr[1][0], 8, (strlen($tmparr[1][0])-8));
	$tmparr[2][0] = substr($tmparr[2][0], 8, (strlen($tmparr[2][0])-8));

	for($i=0; $i<5; $i++){
		$reportarr['valves'][$i] = intval($tmparr[1][$i]);
		$reportarr['dosers'][$i] = intval($tmparr[2][$i]);
	}
	

//	print_r($tmparr);

//	print_r($reportarr);

	$wtrng_time = explode(':',$tmparr[3][0]);
	$reportarr['watering_duration'] = intval($wtrng_time[1]);

	$total_duraton = explode(':',$tmparr[4][0]);
	$reportarr['total_duration'] = intval($total_duration[1]);


	$wtrng_time = explode(':',$tmparr[5][0]);
	$reportarr['water_intaken'] = intval($wtrng_time[1]);

	if ($reportarr['water_intaken']>0) {
		$reportarr['fresh_solution'] = 1;
		$reportarr['solution_birth'] = $timestamp;
	}
	else {
		$reportarr['fresh_solution'] = 0;
	}

//	print_r($reportarr);

}




?>
