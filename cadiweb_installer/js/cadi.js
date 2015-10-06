function rx_ee_dir(addr, value) {
    // direct value upload
    $.post(
        'cm/cadi_bt_processor.php',
        {action: 'tx_packet', cmd:64, addr:addr, value:value},
        function(data) { alert(addr+'/'+value); }
    );
}

/**
 Helper function that checks if the object has property
 */
function hasProperty(object, property) {
    return object ? hasOwnProperty.call(object, key) : false;
}

/**
 Helper function that returns address from provided input
*/
function getAddrFromInput(input) {
    if (! hasProperty(input, 'name'))
        throw "Input does not have 'name' property";

    var inputData = input.name.split('_');

    if (inputData.length < 1)
        throw "Invalid input format";

    return inputData[0];
}

/* accepts the <input>. The 'name' of <input> should be in format:
 aaa_bbb_ccc, where
    - aaa: fields group name (should not intersect with other groups if setting appeared twice)
    - bbb: value address in eeprom
    - ccc: something else
the value is extracted from the 'value' property of <input>
*/
function rx_ee_(input) {
    var addr = getAddrFromInput(input);
    var value = input.value;
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:64, addr:addr, value:value});
}

// XXX: is 'g1' needed as a global variable? It's not used anywhere
// TODO: refactor this bit of code
var g1;
window.onload = function() {
    var g1 = new JustGage({
        id: "psi_gauge",
        value: getRandomInt(0, 60),
        min: 0,
        max: 60,
        title: "",
        label: "PSI"
    });

    setInterval(function() {
        var gaugeVal = $('#psi_gauge_val').val();
        g1.refresh(gaugeVal);
    }, 500);
}; 

$(function() {
    $( "#cadi_tabs" ).tabs({
        beforeLoad: function( event, ui ) {
            ui.jqXHR.error(function() {
            ui.panel.html(
            "Couldn't load this tab. We'll try to fix this as soon as possible. " +
            "If this wouldn't be a demo." );
            });
        }
    });
    $('#svg_container').svg({onLoad: drawMapLayer});
    $( ".btn_" ).button();
    $("#system_view_1").hide();
    cadi_list_rfcomms();
});

$(document).ready(function() {
    cadi_status_stream();
});

var t3curlvl = 0; // XXX: also used in another file
var t4curlvl = 0; // XXX: also used in another file

function drawMapLayer(svg) {
    // draw water levels
    // tank max level in pixels
    // draw cadi time 
    svg.text(140, 500, 'Cadi time',{fill: 'red', strokeWidth: 0, fontSize: '29', id:'cadi_time2'});
    svg.text(140, 575, 'Temp',{fill: 'green', strokeWidth: 0, id:'cadi_temp'});
    svg.text(140, 595, 'rH',{fill: 'blue', strokeWidth: 0, id:'cadi_rh'});
    svg.text(140, 620, 'pH',{fill: 'red', strokeWidth: 0, id:'ph1_adc_val'});
    svg.text(140, 645, 'EC',{fill: 'orange', strokeWidth: 0, id:'ec1_adc_val'});

    svg.text(355, 690, 'Pressure',{fill: 'red', strokeWidth: 0, id:'pressure_label'});

    // display CDD status text
    svg.text(140, 530, 'CDD status',{fill: 'red', strokeWidth: 0, id:'cdds'});

    // display auto_flags
    svg.text(140, 550, 'auto_flags:',{fill: 'red', strokeWidth: 0, id:'auto_flags'});

    // fertilizers
    svg.text(635, 120, 'B',{fill: 'green', strokeWidth: 0, id:'blooml'});
    svg.text(635, 200, 'G',{fill: 'green', strokeWidth: 0, id:'growl'});

    // draw tank levels in text
    svg.text(227, 380, 'XXXmm',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t3l_txt'});
    svg.text(450, 380, 'XXXmm',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t4l_txt'});

    // display pressure
    svg.text(610, 445, '',{fill: 'red', strokeWidth: 0, fontSize: '30', id:'psi_val'});
}

function get_ip() {
    $.post('cm/cadi_bt_processor.php', {action: 'get_ip'}, function(data) {
        alert(data);
    });
}

function change_video() {
    var new_video =$("#video_stream").val();
    $.post('cm/cadi_bt_processor.php', {action: 'change_video', new_video:new_video}, function(data){
        alert('new video='+new_video+' ----- '+data);
    });
}

function toggleValve(valveId) {
    alert(valveId);
}

function bt_connect() {
    //alert('start');
    var mac = $('#bind_mac').val();
    var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
    $.post('cm/cadi_bt_processor.php', {action: 'bt_connect', mac:mac, rfcomm: rfcomm}, function(data) {
        cadi_list_rfcomms();
        $('#main_output').html(data);
    });
}

function bt_restart() {
    alert('restarting');
    $.post('cm/cadi_bt_processor.php', {action: 'bt_restart'}, function(data){
        alert('restarted');
        cadi_list_rfcomms();
        $('#main_output').html(data);
    });
}

function run_watering_pump() {
    var secs = $("#run_watering_secs").val();
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '35', secs:secs}, function(data){
        cadi_list_rfcomms();
    });
}

function run_demo() {
    alert("Demo!");
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '34'}, function(data){
        cadi_list_rfcomms();
    });
}

function rcmd(cmd) {
    // packet created directly in packet processor
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: cmd}, function(data){
        $('#main_output').html(data);
    });
}

function bt_disconnect() {
    btd_stream_status(0);    // Stop pinging Cadi before disconnect
    var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
    $.post('cm/cadi_bt_processor.php', {action: 'bt_disconnect', rfcomm:rfcomm}, function(data) {
        alert('disconnected');
        cadi_list_rfcomms();
        $('#main_output').html(data);
    });
}

function get_status_block() {
    var blocks = $('#status_block_ids').val();
    var block_ids = blocks.split(',');
    for (var i=0; i < block_ids.length; i++) {
        $.post('cm/cadi_bt_processor.php', {action: 'get_status'}, function(data){
            blocks = data.split('---socalledseparator---');
            $('#status_block').html(blocks[0]);
            $('#system_view_2').html(blocks[1]);
            var d = new Date();
            var vs_flag = $('#vs_flag').is(':checked');
            if (vs_flag==1) {
                alert('vs checked');
                $("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
            }
        });
    }
}

function check_plug() {
    alert("checking!");
    $(function(){
        $('#radio_plug1').attr('checked','checked');
        $('#plug1_radio').attr('checked','checked');
        $('#plug1_radio1').attr('checked','true');
        $("radio_").buttonset("refresh");
        $("#radio_plug1").buttonset("refresh");
    });
}

function cadi_bt_scan() {
    $.post('cm/cadi_bt_processor.php', {action: 'rfcomm_scan'}, function(data) {
        $('#bind_mac').html(data);
    });  
}

function cw_reboot() {
    // reboots the machine, running CadiWeb server
    if (confirm('Reboot Cadi server?')) {
        $.post('cm/cadi_bt_processor.php', {action: 'reboot'}, function(data) {
        });
    }
}

function cadi_reset(){
    if (confirm('Reset Cadi?')) {
        rcmd(13);
    }
}

function cadi_list_rfcomms(){
    $.post('cm/cadi_bt_processor.php', {action: 'rfcomm_list_binded'}, function(data) {
        $('#binded_rfcomms').html(data);
    });
}

function stopSerialRead(psid){
    $.post('cm/cadi_bt_processor.php', {action: 'stop_serial_read', process:psid}, function(data) {
        cadi_list_rfcomms();
    });  
}

function bt_tx_packet() {
    var data = $('#tx_data_packet').val();
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', data: data}, function(data) {
        cadi_list_rfcomms();
        alert(data);
    });
}

function bt_tx() {
    var data = $('#tx_data').val();
    $.post('cm/cadi_bt_processor.php', {action: 'tx', cmd: '1', plug_id:'1', state:data}, function(data) {
        cadi_list_rfcomms();
    });
}

function plugStateSet(plug, state){
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data) {
        cadi_list_rfcomms();
    });  
}

function bt_setdd() {
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
        cadi_list_rfcomms();
    });  
}

function cadi_status_stream() {
    var state = $('#flag_status_stream').is(':checked');
    switch (state) {
        case 0:
            var interval = $('#status_stream_interval').val();
            clearInterval(interval);
            break;
        case 1:
            var delay=$("#status_stream_delay").val();
            var interval = setInterval(function(){redraw_svg_layer()},delay);    // enables SVG draw with JS
            $('#status_stream_interval').val(interval);
            break;
    }
}

function cadi_view(viewId) {
    switch (viewId) {
        case 1:
            $("#system_view_1").show();
            $("#system_view_2").hide();
            break;
        case 2:
            $("#system_view_2").show();
            $("#system_view_1").hide();
            break;
    } 
}

function btd_stream_status(newStatus){
    $.post('cm/cadi_bt_processor.php', {action: 'btd_stream_start', status: newStatus}, function(data) {
        // cadi_list_rfcomms();
        // alert('Streaming to server cache');
    });
}

function redraw_update_log(){
    $.post('cm/cadi_bt_processor.php', {action: 'redraw_update_log'}, function(data) {
        $('cadi_update_log').html(data);
    });
}

function cadiweb_update(){
    btd_stream_status(0);
    $.post('cm/cadi_bt_processor.php', {action: 'cadiweb_update'}, function(data){});
    alert("Do not use Cadiweb control panel until server finishes software upgrade with reboot!");
    var log_upd_int_id = setInterval(function(){redraw_update_log},1000);    // enables SVG draw with JS
    $('#log_interval_id').val(log_upd_int_id);
}

function stop_log_stream(){
    var  interval = $('#log_interval_id').val();
    clearInterval(interval);
}

function eeRead() {
    var addr = $('#ee_addr').val();
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:18, addr:addr}, function(data){
        alert('EEPROM value read');
        var out = data.split('seppy');
        $('ee16bit_value').val(out[0]);
        $('ee32bit_value').val(out[1]);
    });  
}

function eeWrite(dataType) {
    var addr = $('#ee_addr').val();
    var cmd=0;
    switch ($dataType) {
        case 1:
            var value = $('#ee16bit_value').val();
            cmd = 15;
            break;
        case 2:
            var value = $('#ee32bit_value').val();
            cmd = 16;
            break;
    }
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', data_type:dataType, value:value, cmd:cmd}, function(data){
        alert('EEPROM value sent');
    }); 
 
}

function enable_dlsettings_overlay(state){
    if (state==1) {
        $('.lcso').css('display', 'block');
    } else {
        $('.lcso').css('display', 'none');
    }
}

function download_csx(type){
    enable_dlsettings_overlay(1);
    var interval_csxdl = $('#csxdl_interval').val();
    clearInterval(interval_csxdl);
    setTimeout(function(){ }, 1000);
    $.post('cm/cadi_bt_processor.php', {action: 'dl_settings'}, function(data){
        // alert(data);
        var interval_csxdl = setInterval(function(){csx_dl_proc(type)},1000);    
        $('#csxdl_interval').val(interval_csxdl);    
    });
}

// running during Cadi settings download
function csx_dl_proc(type){
    $.post('cm/cadi_bt_processor.php', {action: 'check_dl_set_status'}, function(data){
        var tmparr = data.split('v0t0n0');
        var btdState = tmparr[1];
        if (btdState == "1") {                // if BTDaemon is idle
            var interval_csxdl = $('#csxdl_interval').val();
            clearInterval(interval_csxdl);         //, clear interval for checking status
            if (type==0) {
                $.post('cm/csx_get_table.php', {}, function(data){
                    $('#csxform').html(data);    //  and reload settings table
                });
            }
            enable_dlsettings_overlay(0);
        }
    });
}

function mix_solution(){
    var secs = $('#run_watering_secs').val();
    $.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:38, secs:secs}, function(data){
        
    });  
}
