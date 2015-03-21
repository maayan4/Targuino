
// Put each of your customised sliders here.
// window.document.sliders = [ { ... }, { ... }, ... ] = an array of objects.

var sliders = [

	    // First slider (and the only one in this template file)
	    {
	    interactive : true,		// User modifiable on 'true'
	    continuous : true,		// Any position allowed if 'true'
	    span_id : "slider1",

	    left : 100,			// all in 'px' pixels
	    top : 150,
	    pane_image: "pane.gif",

	    scale_width : 200,
	    scale_height : 20,		
	    scale_image : "scale.gif",

	    stylus_width : 38,
	    stylus_height : 52,
	    stylus_up   : "hand.gif",
	    stylus_down : "hand2.gif",

	    tick_height : 10,
	    tick_width : 2,
	    tick_image : "tick.gif",

	    ticks : 2,
	    start_tick : 1,
	    tick_tabs : null,		// auto-calc'ed if set to null

	    label_size : 20,		// in 'px' not in 'pt'
	    label_font : "\"Courier\"",
	    labels : ["Conservative","Liberal"],
	    values : ["1","2"],

	    form_field_id : "slider1",
	    form_id : "form1"			// in  the HTML page.
	}

	// next slider goes here. copy { ... } from the first slider
	// and add a , before the new slider.
    ];

