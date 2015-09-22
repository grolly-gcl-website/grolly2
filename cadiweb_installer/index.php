<?php
    /*
    if ( $http_user_agent ~* (nmap|nikto|wikto|sf|sqlmap|bsqlbf|w3af|acunetix|havij|appscan) ) {
        return 403;
    }
    */

    session_start();
    $_SESSION['cadiweb_version'] = '1.0';
?>

<!doctype html>
<html lang="en">
    <head>
        <meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
        <title>Cadi web UI</title>

        <link rel="stylesheet" type="text/css" href="css/resetcss.css">
        <link rel="stylesheet" type="text/css" href="js/jquery-ui-1.11.2.custom/jquery-ui.css">
        <link rel="stylesheet" type="text/css" href="css/jquery.svg.css"> 
        <link rel="stylesheet" type="text/css" href="css/style.css">
        
        <script src="js/jquery-1.11.2.min.js"></script>
        <script src="js/jquery-ui-1.11.2.custom/jquery-ui.min.js"></script>
        <script src="js/jquery-ui-timepicker-addon.js"></script>

        <script type="text/javascript" src="js/svg/jquery.svg.js"></script>
        <script type="text/javascript" src="js/svg/jquery.svganim.js"></script>
        <script src="js/raphael-min.js"></script>
        <script src="js/justgage.js"></script>
        <script src="js/cadi.js"></script>
        <script>
        <?php
            include(dirname(__FILE__) . '/includes/js.php');
            print_vars(); ?>

            function redraw_svg_layer() {
                $.post('cm/cadi_bt_processor.php', {action: 'get_status_csv'}, function(data) {
                    // XXX: what is 42?
                    // exit early
                    if (data.length <= 42) {
                        return;
                    }

                    var statusArray = data.split(',');
                    // display BTD State
                    $('#btd_state').html(statusArray[22]);

                    if (statusArray[0] > 1000000000) {
                        // workaround for first lost char of CSV (shared memory)
                        var date = new Date(statusArray[0]*1000);
                        var datestring = date.toString().substr(0,24);
                        $('#cadi_time2').html(datestring);
                    }

                    var temp = parseFloat(statusArray[1]).toFixed(1);
                    var rh = parseFloat(statusArray[2]).toFixed(1);
                    $('#cadi_temp').html('&nbsp;T: '+temp+'C');
                    $('#cadi_rh').html('rH: '+rh+'%');

                    var ph1_adc_val = statusArray[10];
                    $('#ph1_adc_val').html('pH: '+ph1_adc_val);
                    $('#sens_ph1_adc_val').html(ph1_adc_val);

                    var ec1_adc_val = statusArray[11];
                    $('#ec1_adc_val').html('EC: '+ec1_adc_val+' uS');

                    // $('#sens_ph1_adc_val').html(ec1_adc_val);
                    // $('#csv_string').html(data);
                    /* var wpProgress = statusArray[18];
                    $('#tf1').html(wpProgress);
                    var auto_failures = statusArray[19];
                    $('#tf2').html(auto_failures);
                    var runners = statusArray[20];
                    $('#tf3').html(runners);

                    var tsf = statusArray[3];        // timer state flags
                    $('#tsf').html(tsf);
                    var ctsf = statusArray[4];        // ctimer state flags
                    $('#ctsf').html(ctsf);    */

                    // tank 3 water level redraw
            
                    <?php echo 'var t3top = '.$svg_t3top.';'.PHP_EOL;
                        echo 'var t3btm = '.$svg_t3btm.';'.PHP_EOL; 
                    ?>

                    t3curlvl = statusArray[8];     // provide current level globally
                    var t3h = 0, t3y = 0;
                    if (statusArray[8]>t3top && statusArray[8]<t3btm) {
                        var lvl = Math.floor((275 * (t3btm-statusArray[8])/(t3btm-t3top)));
                        t3h = 275 - lvl;
                        t3y = 167 + lvl;
                    } else {
                        t3h = 275;
                        t3y = 167;
                    }

                    $('#tank3_water').attr('height',t3h);
                    $('#tank3_water').attr('y',t3y);

                    // tank 4 water lvl redraw
                    <?php echo 't4top = '.$svg_t4top.';'.PHP_EOL;
                        echo 't4btm = '.$svg_t4btm.';'.PHP_EOL; ?>

                    t4curlvl = statusArray[9];    // provide current level globally
                    var t4h = 0, t4y = 0;
                    if (statusArray[9]>t4top && statusArray[9]<t4btm) {
                        var lvl2 = Math.floor((250 * (t4btm-statusArray[9])/(t4btm-t4top)));
                        t4h = lvl2;
                        t4y = 200 + 243 - lvl2;
                    } else {
                        t4h = 243;
                        t4y = 200;
                    }

                    $('#tank4_water').attr('height',t4h);
                    $('#tank4_water').attr('y',t4y);

                    $('#psi_adc_current').html(statusArray[12]);
                    // draw labels for tanks, displaying current level
                    $('#t3l_txt').html(statusArray[8]+'mm');
                    $('#t4l_txt').html(statusArray[9]+'mm');

                    var psi_gauge_val = ((statusArray[12]-psi0psi_)/((psi32psi_ - psi0psi_)/32));
                    $('#psi_gauge_val').val(psi_gauge_val);

                    // get valve states colors
                    // 'svg_valves' line from cadi_settings config file
                    var valves = statusArray[5];
                    for (var i=0;i<10;i++) {
                        if (valves.charAt(15-svg_valves[i])=="1") {
                            $('#valve_'+i).attr('fill', 'green');
                        } else {
                            $('#valve_'+i).attr('fill', 'red');
                        }
                    }

                    // Grolly PSI pump status
                    var psi_pump_state = parseInt(statusArray[21]);
                    if (psi_pump_state==100) {
                        $('#psi_pump').attr('fill', 'red');
                    } else if (psi_pump_state>1 && psi_pump_state<100) {
                        $('#psi_pump').attr('fill', 'green');
                    } else {
                        $('#psi_pump').attr('fill', 'red');
                    }

                    // Grolly mix pump status
                    if (statusArray[16].charAt(3)=="1") {
                        $('#mix_pump').attr('fill', 'green');
                    } else {
                        $('#mix_pump').attr('fill', 'red');
                    }

                    // dosing pumps status squares colors
                    for (var i=1;i<5;i++) {
                        if (statusArray[16].charAt(4-i)=="1") {
                            $('#cdp'+i).attr('fill', 'green');
                        } else {
                            $('#cdp'+i).attr('fill', 'red');
                        }
                    }

                    /*
                    if (statusArray[15]==51){    
                        $('#cdds').html('CDD Enabled');
                        $('#cdds').attr('fill', 'green');
                    }
                    else {
                        $('#cdds').html('CDD Disabled');
                        $('#cdds').attr('fill', 'red');
                    } */

                    var af_bin = statusArray[17].toString(2);
                    $('#auto_flags').html(statusArray[17]+' ('+af_bin+')');

                    var vs_flag = $('#vs_flag').is(':checked');
                    if (vs_flag==1) {
                        d = new Date();
                        $("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
                    }
                });
            }
        </script>
    </head>
    <body>
        <input type="hidden" id="status_stream_interval" value="" />

        <div id="cadi_tabs">
            <ul>
                <li><a href="#tabs-1">Status</a></li>
                <li><a href="cm/cadi_timers.php">Timers</a></li>
                <li><a href="cm/cadi_watering.php">Watering</a></li>
                <li><a href="cm/cadi_fertilization.php">Fertilization</a></li>
                <li><a href="cm/cadi_sensors.php">Sensors</a></li>
                <li><a href="cm/cadi_advanced.php">Advanced</a></li>
                <!--<li><a href="cm/cadi_dd.php">Direct drive</a></li> -->
            </ul>

            <div id="csv_string"></div>

            <div class="ral">
                <b id="btd_state">BTD state</b>
                <button onClick="download_csx(1)">Reload Grolly Settings</button>
                <div class="lcso">Downloading Grolly settings. Please wait ...</div>
                <input type="hidden" value="0" id="csxdl_interval" />
                <!--
                <button class="btn_ fr" onClick="check_plug()">Apply settings</button>
                <button class="btn_ fr">Reload settings</button>
                -->
            </div>
            <div id="tabs-1">
                <input type="checkbox" checked id="flag_status_stream" onClick="cadi_status_stream()" />Stream STATUS (from server cache)
                <input type="checkbox" id="vs_flag" /> also Video
                <button class="btn_" id="cadi_view_cam" onClick="cadi_view(1)">Cam</button>
                <button class="btn_" id="cadi_view_map" onClick="cadi_view(2)">Map</button>
                <br>

                <table>
                    <tr>
                        <td style="vertical-align:top;">
                            <div id="system_view_1">
                                <div id="status_block" style="float:left;">
                                    <?php // XXX: this file is missing // include_once(dirname(__FILE__) . '/cm/status_view_1.php'); ?>
                                </div>
                                <img id="cadi_img" style="float:right;" src="img/curimage.jpeg?" />
                            </div>
                            <div id="system_view_2" style="border: 0px solid blue;">
                                <div style="float:left;">
                                    <div style="float:left;">
                                        <?php echo file_get_contents(dirname(__FILE__) . '/res/interface.svg'); ?>
                                        <div id="csv_string_box"></div>
                                        <div id="psi_gauge" style="width:300px; height:240px; float:left; position: absolute; display: block; top:585px; left:270px;"></div>
                                        <input type="hidden" id="psi_gauge_val" value="0" />
                                    </div style="width:100%; border: 1px solid red;">
                                    <div id="svg_container" style="display:block; min-width:60%; min-height:700px; float:left; border:1px solid yrllow; position:absolute;">
                                    </div>
                                </div>
                        </td>
                        <td>
                            <div style="float:right; border: 0px solid blue;">
                                <?php include_once(dirname(__FILE__) . '/cm/cadi_dd.php'); ?>
                            </div>
                        </td>
                    </tr>
                </table>
    =================================================    
                <br><br>

                <div onClick="cadi_bt_scan();" style="display:inline; border: 1px solid red;">Scan</div>
                <select id="bind_mac" name="bind_mac">
                    <option>Scan to get the list</option>
                </select>

                <button onClick="bt_connect();" style="display:inline; border: 1px solid red;">Connect</button>
                RFCOMM NUMBER: <input type="text" style="width: 40px;" value="0" id="rfcomm_nr" /><br>
                Binded RFCOMM list (<button onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">refresh</button>):
                <div id="binded_rfcomms"> </div>
                <button class="btn_" onClick="btd_stream_status(1);">BTD status stream ON</button>
                <button class="btn_" onClick="btd_stream_status(0);">BTD status stream OFF</button>
<!--    <div title="use this field to send the data to Cadi while connected" id="tx_form">
        <input type="text" name="tx_data" id="tx_data" />
        <div onClick="bt_tx_packet()">Send</div>    
    </div> 


      <div title="send ZX2[1],[1],[1]" id="tx_form">
        <input type="text" name="tx_data_packet" id="tx_data_packet" />
        <div onClick="bt_tx()">Send packet</div>    
    </div> -->

                <br>
                =================================================    
                <br>
                <br>

<!--  <div onClick="bt_setdd()">Set DD</div>    

<div onClick="plugStateSet('1','1')">Enable P1</div>
<div onClick="plugStateSet('1','0')">Disable P1</div>  -->


                <br>
                ==========================================
                <br>


                <br>
                <button onClick="run_demo()">Run demo</button>
                <br>

                <button onClick="cw_reboot()">Reboot server</button><br>
                <button onClick="cadi_reset()">Cadi reset</button><br>
                ====================================
                <br>
                <button onClick="cadiweb_update()">Cadiweb software update</button><br>
                <input type="hidden" id="log_interval_id" value="0" />
                <div>
                    <textarea size="10" id="cadi_update_log"></textarea>
                </div>
                <button onClick=stop_log_stream();>Stop log stream</button>
                ====================================
                <br>
                Status stream delay<input type="text" id="status_stream_delay" value="800"/>
                <br>


                <input type="text" value="0" id="video_stream" title="this value is N in '/dev/videoN'" />
                <button onClick="change_video()">Change video</button>
            </div>
        </div>

    </body>
</html>
