var Scenario = "KR";
var nocache = "";

setInterval("UpdateStatus()",1000); //calls UpdateStatus again after 1000 [ms]. This actually makes the function repeats periodically.
function UpdateStatus(){
    nocache = "&nocache=" + Math.random();
    var USreq = new XMLHttpRequest();
    USreq.onreadystatechange = function(){
        if (this.readyState == 4 && this.status == 200){
            if(this.responseText != null){
                ChangeStatus(this.responseText);
            }  
        }
    }
    USreq.open("GET", "status" + nocache,       true);
    USreq.send(null);
}

function StartCal(){
    nocache = "&nocache=" + Math.random();
    var request = new XMLHttpRequest();
    request.open("GET", "StartCal" + nocache,       true);
    request.send(null);
}
function StartRun(){
    nocache = "&nocache=" + Math.random();
    var request = new XMLHttpRequest();
    request.open("GET", "StartRun" + nocache,       true);
    request.send(null);
}

function Stop(){
    nocache = "&nocache=" + Math.random();
    var request = new XMLHttpRequest();
    request.open("GET", "Stop" + nocache,       true);
    request.send(null);
}

function SaveVel(){
    nocache = "&nocache=" + Math.random();
    var velocity = document.getElementsByID("vel_input_textbox").value;
    var request = new XMLHttpRequest();
    request.open("GET", "SaveVel=" + velocity + " " + nocache,       true);
    request.send(null);
}

function ClearStatus(){
    document.getElementById("StatTextArea").innerHTML = "";
}

function ChangeStatus(newStatus){
    document.getElementById("StatTextArea").innerHTML = document.getElementById("StatTextArea").innerHTML + "&#13;&#10;" + newStatus;
}
    

function ScenarioChange(Scen){
    switch(Scen){
        case "LTR":
            Scenario = "LTR";
            break;
        case "RTL":
            Scenario = "LTR";
            break;
        case "KR":
            Scenario = "KR";
            break;
        default:
            Scenario = "KR";
    }
    var SCreq = new XMLHttpRequest(); 
    SCreq.onreadystatechange = function(){
        if (this.readyState == 4 && this.status == 200){
            if(this.responseText != null){
                //ChangeStatus(this.responseText);
            }  
        }
    }
    SCreq.open("GET", "Scenario=" + Scenario + " " + nocache,       true);
    SCreq.send(null);
}