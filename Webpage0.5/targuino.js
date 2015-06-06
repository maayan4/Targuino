$(document).ready(function () {
    
    $('#RTL_vel_range').noUiSlider({
        start: 0.5,
        step: 0.1,
        range: {
            'min': 0,
            'max': 3.5
        },   
        connect: 'lower',
    });
    
    $('#LTR_vel_range').noUiSlider({
    start: 0.5,
    step: 0.1,
    range: {
        'min': 0,
        'max': 3.5
    },   
    connect: 'lower',
    });
            
  $('#RTL_vel_range').Link('lower').to('-inline-<div class="tooltip"></div>', function ( value ) { 
      $(this).html(value);  
  });
    
  $('#LTR_vel_range').Link('lower').to('-inline-<div class="tooltip"></div>', function ( value ) { 
    $(this).html(value);  
  });

  $('#RTL_vel_range').on('set',SaveRTLVel);
  $('#LTR_vel_range').on('set',SaveLTRVel);

$('.spinner-button').click(function(){
  var target = $(this).siblings('.spinner-input');
  if ($(this).hasClass('spinner-up')){
    $(target).val((parseInt($(target).val()) + 1));
  }
    else{
    $(target).val((parseInt($(target).val()) - 1));
  }
   // $('.spinner-input').trigger('change');
    return false;
});

$('.spinner-input').change(function(){
    if($(this).val() > 60 || $(this).val() < 0){
        $('.spinner').css({"border" : "2px solid red"});
    }
    else if(Math.floor($(this).val()) != $(this).val() || $(this).val().isNumeric){ 
        $('.spinner').css({"border" : "2px solid red"});
    }
    else{
        $('.spinner').css({"border" : "1px solid #dcdcdc"});
    }})


});

var nocache = "";

setInterval(UpdateStatus,2000); //calls UpdateStatus again after 1000 [ms]. This actually makes the function repeats periodically.

function UpdateStatus(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var USreq = new XMLHttpRequest();
USreq.onreadystatechange = function(){
    if (this.readyState == 4 && this.status == 200){
        if(this.responseXML !== null){
            document.getElementById("disp_RTL_vel").innerHTML = this.responseXML.getElementsByTagName("RTL_Velocity")[0].childNodes[0].nodeValue;
            document.getElementById("disp_RTL_delay").innerHTML = this.responseXML.getElementsByTagName("RTL_Delay")[0].childNodes[0].nodeValue;
            document.getElementById("disp_LTR_vel").innerHTML = this.responseXML.getElementsByTagName("LTR_Velocity")[0].childNodes[0].nodeValue;
            document.getElementById("disp_LTR_delay").innerHTML = this.responseXML.getElementsByTagName("LTR_Delay")[0].childNodes[0].nodeValue;
            document.getElementById("disp_xbee_status").innerHTML = this.responseXML.getElementsByTagName("XBEE_STATUS")[0].childNodes[0].nodeValue;
            document.getElementById("disp_sysmode").innerHTML = this.responseXML.getElementsByTagName("SysMode")[0].childNodes[0].nodeValue;
            document.getElementById("disp_track_length").innerHTML = this.responseXML.getElementsByTagName("TrackLength")[0].childNodes[0].nodeValue;
            document.getElementById("StatTextArea").innerHTML = this.responseXML.getElementsByTagName("Messages")[0].childNodes[0].nodeValue + "&#13;&#10;";
        }  
    }
}
USreq.open("GET", "status" + nocache,       true);
USreq.send(null);
}

function StartCal(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var request = new XMLHttpRequest();
request.open("GET", "BeginCalibration" + nocache,       true);
request.send(null);
}
function StartRun(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var request = new XMLHttpRequest();
request.open("GET", "BeginRunning" + nocache,       true);
request.send(null);
}

function Stop(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var request = new XMLHttpRequest();
request.open("GET", "Stop" + nocache,       true);
request.send(null);
}

function SaveRTLVel(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var velocity = 10000 * $('#RTL_vel_range').val();
var request = new XMLHttpRequest();
request.open("GET", "RTL_vel=" + velocity + nocache,       true); //multiply by 100 to avoid memcpy issues on arduino
request.send(null);
}

function SaveLTRVel(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var velocity = 10000 * $('#LTR_vel_range').val();
var request = new XMLHttpRequest();
request.open("GET", "LTR_vel=" + velocity + nocache,       true);
request.send(null);
}

function SaveRTLDelay(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var delay = 1000 * document.getElementById("RTL_delay").value;
var request = new XMLHttpRequest();
request.open("GET", "RTL_delay=" + delay + nocache,       true); //multiply by 1000 for [ms]
request.send(null);
}

function SaveLTRDelay(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var delay = 1000 * document.getElementById("LTR_delay").value;
var request = new XMLHttpRequest();
request.open("GET", "LTR_delay=" + delay + nocache,       true);
request.send(null);
}

function ClearStatus(){
document.getElementById("StatTextArea").innerHTML = "";
}
