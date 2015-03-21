
// ---------------------------------------------------------------------
// Beware testing this on Netscape 4.x.
// 1. get the latest 4.x  sub-version (currently 4.79)
// 2. if pages seems to render unreliably, load a different page then
//    reload the page under test.
// 3. why Netscape 4.x requires double-nested SPANs we don't know.
//
// functions:
//	slider_render_all()
//	slider_render()
//	slider_render_scale()
//	slider_render_pane()
//	slider_render_ticks()
//	slider_render_labels()
//	slider_render_stylus_up()
//	slider_render_stylus_down()
//
//	slider_normalise()
//	slider_align()
//
//	slider_stylus_mousedown()
//	slider_stylus_mousemove()
//	slider_stylus_mouseup()
//
//	slider_set_value()
//
// data:
//
// 	a copy of sliders.js must be loaded before this library.
//
//	sliding = a global variable indicating state.
//
// ---------------------------------------------------------------------

var sliding = false;			// state for user drag'n'drop

// ---------------------------------------------------------------------
// create all the sliders in one go
function slider_render_all()
{
    var i;
    for (i=0; i<sliders.length; i++)
    {
	slider_render(sliders[i]);
    }
}

// ---------------------------------------------------------------------
// create the slider specified by some slider[x] object.
function slider_render(obj)
{
var i;
var span;

    with (window.document)
    {
	if (old)
	{
	    write("<BR>No browser support for sliders<BR>");
	    return;
	}

	// now write out all the bits

	slider_render_scale(obj);
	slider_render_pane(obj);

	if ( obj.tick_tabs == null )	// calculate tabs between ticks
	{
	    obj.tick_tabs = (obj.scale_width-obj.tick_width) / (obj.ticks-1.0);
	}

	for (i=0; i<obj.ticks; i++)	// draw ticks and labels
	{
	    slider_render_tick(i, obj);
	    slider_render_label(i, obj);
	}

	slider_render_stylus_up(obj);
	slider_render_stylus_down(obj);

	// Install event handlers

	if (obj.interactive == true)
	{
	    if (moz || dom2)
	    {
		span = getElementById(obj.span_id);
		span.addEventListener("mousedown",slider_stylus_mousedown, 0);
		span.addEventListener("mouseup",slider_stylus_mouseup, 0);
		span.addEventListener("mousemove",slider_stylus_mousemove, 0);
	    }
	    if ( ie4 || ie5 || ie6 )
	    {
		span = all(obj.span_id);
		span.onmousedown = slider_stylus_mousedown;
		span.onmouseup = slider_stylus_mouseup;
		span.onmousemove = slider_stylus_mousemove;
	    }
	    if ( nn4 )
	    {
		span = layers[obj.span_id];
		span.captureEvents(Event.MOUSEDOWN|Event.MOUSEUP|Event.MOUSEMOVE);
		span.onMouseDown = slider_stylus_mousedown;
		span.onMouseUp = slider_stylus_mouseup;
		span.onMouseMove = slider_stylus_mousemove;
		window.document.onMouseMove = null;
	    }
	}
    }
}

// ---------------------------------------------------------------------
// render the scale "slot" or line
function slider_render_scale(obj)
{
    with (window.document) {
    write("<SPAN><SPAN " +
		"STYLE=\"" +
		    "z-index:1; margins:0; padding:0; " +
		    "position: absolute; " +
		    "top:" + (obj.top + (obj.stylus_height/2) - (obj.scale_height/2)) + "px; " +
		    "left:" + obj.left + "px; " +
		"\" >");

    write("<IMG " +
		"SRC=\""+obj.scale_image+"\" " +
		"HEIGHT=\"" + obj.scale_height + "\" " +
		"WIDTH=\"" + obj.scale_width + "\" " +
		">");
    write("</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// Background clickable span used for all mouse input
// There is a hack here for NS 4.x - neither ID nor NAME
// are accessable from event handlers, so we tack the ID onto
// the end of the SRC attribute. Still a valid URL and doesn't break NN 4.
// If we don't do this, we can't tell which slider it is.
function slider_render_pane(obj)
{
    with (window.document) {
	write("<SPAN><SPAN " +
		    "ID=\"" + obj.span_id + "\" " +
		    "STYLE=\"" +
    			"z-index:10; margins:0; padding:0; " +
		        "position: absolute; " +
			"top: " + obj.top + "px; " +
			"left: " + (obj.left - obj.stylus_width/2) + "px; " +
	    "\" >" );

	write("<IMG " +
		    "ID=\"" + obj.span_id + "_pane\" " +
		    ( nn4 ? "SRC=\""+obj.pane_image+"#"+obj.span_id+"\" "
		          : "SRC=\""+obj.pane_image+"\" "
		    ) +
		    "HEIGHT=\"" + (obj.stylus_height + obj.tick_height + obj.label_size) + "\" " +
		    "WIDTH=\"" + (obj.scale_width+obj.stylus_width) + "\" " +
		    ">");
	write("</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// a mark along the scale
function slider_render_tick(i,obj)
{
    with (window.document) {
	write("<SPAN><SPAN " +
		    "STYLE=\"" +
			"z-index:1;" +
			"position: absolute; " +
			"top:" + (obj.top + obj.stylus_height) + "px; " +
			"left:" + (obj.left + i*obj.tick_tabs) + "px; " +
			"font-size:" + obj.tick_width + "px; " +
		    "\">" );

	write("<IMG " +
		    "SRC=\"" + obj.tick_image + "\" " +
		    "HEIGHT=\"" + obj.tick_height + "\" " +
		    "WIDTH=\"" + obj.tick_width + "\" " +
		    ">");
	write("</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// a label for one of the tick marks
function slider_render_label(i,obj)
{
   var label_style = "z-index:1; padding:0; margins:0;";

   with (window.document)
   {
	write("<SPAN><SPAN " +
		    "STYLE=\"" +
			"position: absolute; "+
			"top:" + (obj.top + obj.stylus_height + obj.tick_height) + "px; " +
			"left:" + (obj.left + i*obj.tick_tabs) + "px; " +
			"font-size:" + obj.label_size + "px; " +
			((!nn4) ? "font-family:"+obj.label_font+"; " : "") +
			label_style + "\" " +
	       ">" );
	write(obj.labels[i] + "</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// the stylus on the scale line (the "claw" or "pointer")
function slider_render_stylus_up(obj)
{
  with (window.document) {
	write("<SPAN><SPAN " +
		    "ID=\"" + obj.span_id + "up" + "\" " +
		    "STYLE=\"" +
			"z-index:2; " +
			"position: absolute; margins:0; padding:0; "+
			"top:" + obj.top + "px; " +
			"left:" + (obj.left + obj.start_tick*obj.tick_tabs - obj.stylus_width/2) + "px; " +
		"\" >");
	write("<IMG " +
		( nn4 ? "SRC=\""+obj.stylus_up+"#"+obj.span_id+"up\" "
		      : "SRC=\""+obj.stylus_up+"\" "
		) +
		"HEIGHT=\"" + obj.stylus_height + "\" " +
		"WIDTH=\"" + obj.stylus_width + "\" " +
		">");
    write("</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// the highlighted stylus on the scale line
function slider_render_stylus_down(obj)
{
  with (window.document) {
	write("<SPAN><SPAN " +
		    "ID=\"" + obj.span_id + "down" + "\" " +
		    "STYLE=\"" +
			"z-index:3; " +
			"position: absolute; margins:0; padding:0;"+
			( nn4 ? "visibility: hide; " : "visibility: hidden; ")+
			"top:" + obj.top + "px; " +
			"left:" + (obj.left + obj.start_tick*obj.tick_tabs - obj.stylus_width/2) + "px; " +
		"\" >");
	write("<IMG " +
		( nn4 ? "SRC=\""+obj.stylus_down+"#"+obj.span_id+"down\" "
		      : "SRC=\""+obj.stylus_down+"\" "
		) +
		"HEIGHT=\"" + obj.stylus_height + "\" " +
		"WIDTH=\"" + obj.stylus_width + "\" " +
		">");
    write("</SPAN></SPAN>");
    }
}

// ---------------------------------------------------------------------
// event handler for 3 classes of browser
function slider_stylus_mousedown(obj)
{
    var i;
    var slider_name;

    if ( old )
    	return;

    if (moz || dom2)
    {
        slider_name = obj.currentTarget.id;
	document.getElementById(slider_name+"up").style.visibility = "hidden";
	document.getElementById(slider_name+"down").style.visibility = "visible";
	obj.stopPropagation();
	obj.preventDefault();
    }

    if ( ie4 || ie5 || ie6 )
    {
	// picks up the <IMG> tag as src.
	// picks up the z-index:0 tag if the <IMG> edge is reached
	with (window.event.srcElement)
	{
	    if ( id == parentElement.id + "_pane" )
	    {
	    	slider_name = parentElement.id;
	    }
	    else
	    {
	    	slider_name = window.document.activeElement.id;
	    }
	}

	document.all(slider_name+"up").style.visibility = "hidden";
	document.all(slider_name+"down").style.visibility = "visible";
	window.event.cancelBubble = true;
	window.event.returnValue = false;
    }

    if ( nn4 )
    {
	slider_name = obj.target.src;
	i = slider_name.indexOf("#",0);	// the NN 4.x hack
	slider_name = slider_name.substring(i+1,slider_name.length);

	window.document.layers[slider_name+"up"].visibility = "hide";
	window.document.layers[slider_name+"down"].visibility = "show";
    }

    sliding = true;

    return false;	// for nn4
}

// ---------------------------------------------------------------------
// event handler for 3 classes of browser
function slider_stylus_mousemove(obj)
{
    var slider_name = "";
    var config = null;
    var x = 0;
    var i = 0;

    if ( old || !sliding )
    	return;

    // find current location and slider data

    if (moz || dom2)
    {
        slider_name = obj.currentTarget.id;
	x = obj.clientX;
	obj.stopPropagation();
	obj.preventDefault();
    }
    if ( ie4 || ie5 || ie6 )
    {
	// picks up the <IMG> tag as src.
	// picks up the z-index:0 tag if the <IMG> edge is reached
	// picks up <html> if user move is very fast

	with (window.event.srcElement)
	{
	    if ( tagName == "HTML" )
	    {
	    	sliding = false;
	        return;		// very fast mouse swipe by user - do nothing
	    }
	    else if ( id == parentElement.id + "_pane" )
	    {
	    	slider_name = parentElement.id;
	    }
	    else
	    {
	    	slider_name = window.document.activeElement.id;
	    }
	}

	x = window.event.clientX;
	window.event.cancelBubble = true;
	window.event.returnValue = false;
    }
    if ( nn4 )
    {
	slider_name = obj.target.src;
	i = slider_name.indexOf("#",0);	// the  NN 4.x hack
	slider_name = slider_name.substring(i+1,slider_name.length);
	x = obj.pageX;
    }

    for (i=0; i<sliders.length; i++)
    {
	if (sliders[i].span_id == slider_name )
	{
	    config = sliders[i];
	}
    }

    x = slider_normalise(x, config);
    x -= config.stylus_width/2;		// left edge of stylus, not center

    // set new location

    if (moz || dom2)
    {
	document.getElementById(slider_name+"up").style.left = x + "px";
	document.getElementById(slider_name+"down").style.left = x + "px";
    }
    if ( ie4 || ie5 || ie6 )
    {
        document.all(slider_name+"up").style.left = x + "px";
        document.all(slider_name+"down").style.left = x + "px";
    }
    if ( nn4 )
    {
	window.document.layers[slider_name+"up"].left = x;
	window.document.layers[slider_name+"down"].left = x;
    }

    return false;	// for nn4
}

// ---------------------------------------------------------------------
// event handler for 3 classes of browser
function slider_stylus_mouseup(obj)
{
    var slider_name;
    var x;

    if ( old || !sliding)
    	return;

    // find current location and slider data
    if (moz || dom2)
    {
        slider_name = obj.currentTarget.id;
	obj.stopPropagation();
	obj.preventDefault();
	x = obj.clientX;
    }

    if ( ie4 || ie5 || ie6 )
    {
	// picks up the <IMG> tag as src.
	// picks up the z-index:0 tag if the <IMG> edge is reached

	with (window.event.srcElement)
	{
	    if ( id == parentElement.id + "_pane" )
	    {
	    	slider_name = parentElement.id;
	    }
	    else
	    {
	    	slider_name = window.document.activeElement.id;
	    }
	}

	x = window.event.clientX;
	window.event.cancelBubble = true;
	window.event.returnValue = false;
    }

    if ( nn4 )
    {
	slider_name = obj.target.src;
	i = slider_name.indexOf("#",0);	// the NN 4.x hack
	slider_name = slider_name.substring(i+1,slider_name.length);
	x = obj.pageX;
    }

    for (i=0; i<sliders.length; i++)
    {
	if (sliders[i].span_id == slider_name )
	{
	    config = sliders[i];
	}
    }

    // calculate final slider position

    x = slider_normalise(x, config);
    x = slider_align(x, config);

    x -= config.stylus_width/2;		// center, not edge of stylus

    // do final update to position
    if ( moz || dom2)
    {
	document.getElementById(slider_name+"up").style.left = x + "px";
	document.getElementById(slider_name+"down").style.left = x + "px";

	document.getElementById(slider_name+"up").style.visibility = "visible";
	document.getElementById(slider_name+"down").style.visibility = "hidden";
    }
    if ( ie4 || ie5 || ie6 )
    {
	document.all(slider_name+"up").style.left = x + "px";
	document.all(slider_name+"down").style.left = x + "px";

	document.all(slider_name+"up").style.visibility = "visible";
	document.all(slider_name+"down").style.visibility = "hidden";
    }

    if ( nn4 )
    {
	window.document.layers[slider_name+"up"].left = x;
	window.document.layers[slider_name+"down"].left = x;

	window.document.layers[slider_name+"up"].visibility = "show";
	window.document.layers[slider_name+"down"].visibility = "hide";
    }

    window.sliding = false;
    return false;
}

// ---------------------------------------------------------------------
// make sure position x is never beyond the allowed bounds of the slider
function slider_normalise(x,obj)
{
    var xmin, xmax;
    // calculate new stylus location

    xmin = obj.left;
    xmax = obj.left + obj.scale_width

    x = ( x <= xmin ) ? xmin : x;
    x = ( x >= xmax ) ? xmax : x;

    return x;
}

// ---------------------------------------------------------------------
// if the slider isn't continuous==true , align x with a tick mark.
function slider_align(x,obj)
{
    var tab = 0;
    if (!obj.continuous)		// must align with a tick
    {
	while ( (x -= obj.tick_tabs) > obj.left - obj.tick_tabs/2 )
	{
	    tab++;
	}
	x = obj.left + tab * obj.tick_tabs;
    }
    return x;
}

// ---------------------------------------------------------------------
// copy the slider setting into a form field. Call this from your own place.
function slider_set_value(obj)
{
    var i = 0, x = 0, data = null;

    if (moz || dom2)
    {
	x = window.document.getElementById(obj.span_id+"up").style.left;
	x = x.substring(0,x.length-2) - 0;
    }

    if ( ie4 || ie5 || ie6 )
    {
        x = document.all(obj.span_id+"up").style.left;
	x = x.substring(0,x.length-2) - 0;
    }

    if ( nn4 )
    {
	x = window.document.layers[obj.span_id+"up"].left;
    }

    x += obj.stylus_width/2;

    if (!obj.continuous)	// must be aligned with a tick
    {
        x -= obj.left;
	while ( (x-=obj.tick_tabs) >= 0 )
	{
	    i++;
	}
	data = obj.values[i];
    }
    else
    {
	i = (x*1.0 - obj.left) / obj.status_width;
    	data = i * (obj.values[object.ticks-1] - obj.values[0]);
    }
    window.document.forms[obj.form_id][obj.form_field_id].value
    	= data + "";
}
