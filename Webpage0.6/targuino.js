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

function ClearStatus(){
document.getElementById("StatTextArea").innerHTML = "";
}

function updateSpinnerInput(element){
    
    var target = element.parentElement.getElementsByClassName("spinner-input")[0];
    
    if(element.className == "spinner-button spinner-down"){ 
        target.value = parseInt(target.value) - 1;
    }
    else if(element.className == "spinner-button spinner-up"){ 
        target.value = parseInt(target.value) + 1;
    }
    
    checkDelayValue(target);
    
}

function checkDelayValue(element){
    var par = element.parentElement;
    if(element.value > 60 || element.value < 0){
        par.style.border = "2px solid red";
    }
    else{ 
        par.style.border = "none";
        updateDelay(element);
    }
}

function updateVelocity(element){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var velocity = 10000 * element.value;
    var request = new XMLHttpRequest();
    if(element.id == "RTLvelInput"){ request.open("POST", "RTL_vel=" + velocity + nocache,       true);}
    else if(element.id == "LTRvelInput"){ request.open("POST", "LTR_vel=" + velocity + nocache,       true);}
    request.send(null);
}

function updateDelay(element){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var delay = 1000 * element.value;
    var request = new XMLHttpRequest();
    if(element.id == "RTLD_in"){ request.open("POST", "RTL_vel=" + delay + nocache,       true);}
    else if(element.id == "LTRD_in"){ request.open("POST", "LTR_vel=" + delay + nocache,       true);}
    request.send(null);
}
