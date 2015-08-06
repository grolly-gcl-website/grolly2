<?php

error_reporting(-1);
ini_set('display_errors', 'On');
// getimagesize($file);

// phpinfo();


// echo getimagesize('welco_wpx1_pump.jpg');


// Создание изображения 300x100
// $im = imagecreatetruecolor(300, 100);
$im = imagecreatefromjpeg('osd_test1.jpg');
//$red = imagecolorallocate($im, 0xFF, 0xFF, 0xCC);
$bg_color = imagecolorallocatealpha($im, 222, 222, 222, 97);
$black = imagecolorallocatealpha($im, 0, 0, 0, 25);
$white = imagecolorallocatealpha($im, 255, 255, 255, 25);

// Сделаем краный фон
imagefilledrectangle($im, 0, 0, 199, 199, $bg_color);

// Путь к ttf файлу шрифта
$font_file = './cousine-regular.ttf';
$font_file_bold = './cousine.bold.ttf';


$font_size = 10;	// text font size
$line_space = 12;	// text lines spacing
$left_margin = 2;	// text margin from left border

$wp_run_time_ago = 500;	// when last WP happened (minuetes ago)
$wp_run_secs = 90;	// amount of seconds, the last wp was running
$fw_amount = 10;
$doser_run[1] = 5;
$doser_run[2] = 10;


$i = 2;
date_default_timezone_set("UTC");
$photo_time = date("Y-m-d H:i:s", time());

// Рисуем текст 'PHP Manual' шрифтом 13го размера BOLD
imagefttext($im, $font_size, 0, $left_margin, 11, $black, $font_file_bold, 'T: '.$photo_time);

// Рисуем текст 'PHP Manual' шрифтом 13го размера BOLD
imagefttext($im, $font_size, 0, $left_margin, $i*$line_space, $black, $font_file_bold, 'Last wtrng: '.$wp_run_time_ago.' mins ago');

$i++;

imagefttext($im, $font_size, 0, $left_margin	, $i*$line_space, $black, $font_file_bold, 'Duration: '.$wp_run_secs.' secs');

$i++;

imagefttext($im, $font_size, 0, $left_margin	, $i*$line_space, $black, $font_file_bold, 'Fresh/Existing solution');


$i++;

imagefttext($im, $font_size, 0, $left_margin	, $i*$line_space, $black, $font_file_bold, 'Water amount:'.$fw_amount.' cms');


$i++;

imagefttext($im, $font_size, 0, $left_margin	, $i*$line_space, $black, $font_file_bold, 'Grow:'.$doser_run[1].' secs');

$i++;

imagefttext($im, $font_size, 0, $left_margin	, $i*$line_space, $black, $font_file_bold, 'Bloom:'.$doser_run[2].' secs');






// and the same with normal
// imagefttext($im, 13, 0, 5, 55, $white, $font_file, 'PHP Manual');

// Вывод изображения
header('Content-Type: image/png');

imagepng($im);
imagedestroy($im);
?>

