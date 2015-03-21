
// Script and variables to indicate the current browser

var nn4 = false;	// Netscape Navigator or Communicator
var ie4 = false;
var ie5 = false;
var ie6 = false;
var moz = false;	// Mozilla, NS 6+, Konqueror, etc
var dom1 = false;	// fully supports DOM1
var dom2 = false;	// fully supports (important bits of) DOM2
var old = false;	// very old browser

function browser_detect()
{
    // do it the official W3C way

    if ( window.document.implementation != null)
    {
         dom1 = window.document.implementation.hasFeature("HTML","1.0");
         dom2 = window.document.implementation.hasFeature("HTML","2.0") &&
         	window.document.implementation.hasFeature("Events","2.0") &&
         	window.document.implementation.hasFeature("Core","2.0") &&
         	window.document.implementation.hasFeature("CSS2","2.0");
    }

    // Mozilla based browsers contain the Gecko rendering engine

    moz = (window.navigator != null )
		? ( window.navigator.userAgent.indexOf("ecko") != -1 )
		: false;

    nn4 = (window.document.layers != null && !moz);

    // IE has incremental support for the Document Object Model

    ie6 = (window.document.all && dom1);
    ie5 = (window.document.all && window.document.getElementsByTagName && !ie6);
    ie4 = (window.document.all && !ie6 && !ie5);
   
    // something horrible and old - sorry Opera.

    old = (!ie4 && !ie5 && !ie6 && !dom1 && !nn4 && !moz);

}

browser_detect();

