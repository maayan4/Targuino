var nocache = "";

setInterval("UpdateStatus()",2000); //calls UpdateStatus again after 1000 [ms]. This actually makes the function repeats periodically.
function UpdateStatus(){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var USreq = new XMLHttpRequest();
    USreq.onreadystatechange = function(){
        if (this.readyState == 4 && this.status == 200){
            if(this.responseText !== null){
                ChangeStatus(this.responseText);
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
    document.getElementsByName("velocity").value = document.getElementById("RTL_vel_range").nodeValue;
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var velocity = document.getElementById("RTL_vel_range").value;
    var request = new XMLHttpRequest();
    request.open("GET", "RTL_vel=" + 100*velocity + nocache,       true);
    request.send(null);
}

function SaveLTRVel(){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var velocity = document.getElementById("LTR_vel_range").value;
    var request = new XMLHttpRequest();
    request.open("GET", "LTR_vel=" + 100*velocity + nocache,       true);
    request.send(null);
}

function SaveRTLDelay(){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var delay = document.getElementById("RTL_delay").value;
    var request = new XMLHttpRequest();
    request.open("GET", "RTL_delay=" + 1000*delay + nocache,       true);
    request.send(null);
}

function SaveLTRDelay(){
    nocache = "&nocache=" + Math.floor(Math.random()*10000);
    var delay = document.getElementById("LTR_delay").value;
    var request = new XMLHttpRequest();
    request.open("GET", "LTR_delay=" + 1000*delay + nocache,       true);
    request.send(null);
}

function ClearStatus(){
    document.getElementById("StatTextArea").innerHTML = "";
}

function ChangeStatus(newStatus){
    document.getElementById("StatTextArea").innerHTML =            document.getElementById("StatTextArea").innerHTML + "&#13;&#10;" + newStatus;
}
    