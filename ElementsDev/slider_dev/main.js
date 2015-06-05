
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
		
$(document).on('click', '.spinner-button', function(){
  var target = $(this).siblings(':first');
  if ($(this).hasClass('spinner-up')){
    $(target).val((parseInt($(target).val()) + 1));
  }else{
    $(target).val((parseInt($(target).val()) - 1));
  }
  $(':input.spinner-input').trigger('change');
  return false;
});
    
$('.spinner-input').change(function(){
    if($(this).val() > 30 || $(this).val() < 0){
        $('.spinner').css({"border" : "1px solid red"});
    }
    else if(Math.floor($(this).val()) != $(this).val() || $(this).val().isNumeric){ 
        $('.spinner').css({"border" : "1px solid red"});
    }
    else{
        $('.spinner').css({"border" : "1px solid #dcdcdc"});
    }})

    
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