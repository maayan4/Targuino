
$(document).ready(function () {
    
    $('#rrr').noUiSlider({
        start: 0.5,
        step: 0.1,
        range: {
            'min': 0,
            'max': 3.5
        },   
        connect: 'lower',
    });
            
  $('#rrr').Link('lower').to('-inline-<div class="tooltip"></div>', function ( value ) { 
      $(this).html('<strong>Value: </strong>' + '<span>' + value + '</span>');  
  });

  $('#rrr').on('set',SaveLTRVel);

});

 
function StartRun(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var request = new XMLHttpRequest();
request.open("GET", "BeginRunning" + nocache,       true);
request.send(null);
}

function SaveLTRVel(){
nocache = "&nocache=" + Math.floor(Math.random()*10000);
var velocity = 10000 * $("#rrr").val();
var request = new XMLHttpRequest();
request.open("GET", "LTR_vel=" + velocity + nocache,       true);
request.send(null);
}