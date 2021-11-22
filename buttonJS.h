/*
 * HTML and Javascript code
 */
const char body[] PROGMEM = R"===(
<!DOCTYPE HTML>
<html>
  <head>
    <title>Joy</title>
    <meta charset="utf-8">
    <meta name="description" content="Example page of use pure Javascript JoyStick">
    <meta name="author" content="Roberto D'Amico">
    <link rel="shortcut icon" type="image/png" href="http://bobboteck.github.io/img/roberto-damico-bobboteck.png">
    <style>
*
{
  box-sizing: border-box;
}
body
{
  margin: 0px;
  padding: 0px;
  font-family: monospace;
}
.row
{
  display: inline-flex;
  clear: both;
}
.columnLateral
{
  float: left;
  width: 15%;
  min-width: 300px;
}
.columnCetral
{
  float: left;
  width: 70%;
  min-width: 300px;
}
#joy2Div
{
  width:200px;
  height:200px;
  margin:50px
}
#joystick
{
  border: 1px solid #FF0000;
}
#joystick2
{
  border: 1px solid #0000FF;
}
    </style>
    <script>
    var JoyStick = (function(container, parameters)
{
  parameters = parameters || {};
  var title = (typeof parameters.title === "undefined" ? "joystick" : parameters.title),
    width = (typeof parameters.width === "undefined" ? 0 : parameters.width),
    height = (typeof parameters.height === "undefined" ? 0 : parameters.height),
    internalFillColor = (typeof parameters.internalFillColor === "undefined" ? "#00AA00" : parameters.internalFillColor),
    internalLineWidth = (typeof parameters.internalLineWidth === "undefined" ? 2 : parameters.internalLineWidth),
    internalStrokeColor = (typeof parameters.internalStrokeColor === "undefined" ? "#003300" : parameters.internalStrokeColor),
    externalLineWidth = (typeof parameters.externalLineWidth === "undefined" ? 2 : parameters.externalLineWidth),
    externalStrokeColor = (typeof parameters.externalStrokeColor ===  "undefined" ? "#008000" : parameters.externalStrokeColor),
    autoReturnToCenter = (typeof parameters.autoReturnToCenter === "undefined" ? true : parameters.autoReturnToCenter);
  
  // Create Canvas element and add it in the Container object
  var objContainer = document.getElementById(container);
  var canvas = document.createElement("canvas");
  canvas.id = title;
  if(width === 0) { width = objContainer.clientWidth; }
  if(height === 0) { height = objContainer.clientHeight; }
  canvas.width = width;
  canvas.height = height;
  objContainer.appendChild(canvas);
  var context=canvas.getContext("2d");
  
  var pressed = 0; // Bool - 1=Yes - 0=No
    var circumference = 2 * Math.PI;
    var internalRadius = (canvas.width-((canvas.width/2)+10))/2;
  var maxMoveStick = internalRadius + 5;
  var externalRadius = internalRadius + 30;
  var centerX = canvas.width / 2;
  var centerY = canvas.height / 2;
  var directionHorizontalLimitPos = canvas.width / 10;
  var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
  var directionVerticalLimitPos = canvas.height / 10;
  var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
  // Used to save current position of stick
  var movedX=centerX;
  var movedY=centerY;
    
  // Check if the device support the touch or not
  if("ontouchstart" in document.documentElement)
  {
    canvas.addEventListener("touchstart", onTouchStart, false);
    document.addEventListener("touchmove", onTouchMove, false);
    document.addEventListener("touchend", onTouchEnd, false);
  }
  else
  {
    canvas.addEventListener("mousedown", onMouseDown, false);
    document.addEventListener("mousemove", onMouseMove, false);
    document.addEventListener("mouseup", onMouseUp, false);
  }
  // Draw the object
  drawExternal();
  drawInternal();

  /******************************************************
   * Private methods
   *****************************************************/

  /**
   * @desc Draw the external circle used as reference position
   */
  function drawExternal()
  {
    context.beginPath();
    context.arc(centerX, centerY, externalRadius, 0, circumference, false);
    context.lineWidth = externalLineWidth;
    context.strokeStyle = externalStrokeColor;
    context.stroke();
  }

  /**
   * @desc Draw the internal stick in the current position the user have moved it
   */
  function drawInternal()
  {
    context.beginPath();
    if(movedX<internalRadius) { movedX=maxMoveStick; }
    if((movedX+internalRadius) > canvas.width) { movedX = canvas.width-(maxMoveStick); }
    if(movedY<internalRadius) { movedY=maxMoveStick; }
    if((movedY+internalRadius) > canvas.height) { movedY = canvas.height-(maxMoveStick); }
    context.arc(movedX, movedY, internalRadius, 0, circumference, false);
    // create radial gradient
    var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
    // Light color
    grd.addColorStop(0, internalFillColor);
    // Dark color
    grd.addColorStop(1, internalStrokeColor);
    context.fillStyle = grd;
    context.fill();
    context.lineWidth = internalLineWidth;
    context.strokeStyle = internalStrokeColor;
    context.stroke();
  }
  
  /**
   * @desc Events for manage touch
   */
  function onTouchStart(event) 
  {
    pressed = 1;
  }

  function onTouchMove(event)
  {
    // Prevent the browser from doing its default thing (scroll, zoom)
    event.preventDefault();
    if(pressed === 1 && event.targetTouches[0].target === canvas)
    {
      movedX = event.targetTouches[0].pageX;
      movedY = event.targetTouches[0].pageY;
      // Manage offset
      if(canvas.offsetParent.tagName.toUpperCase() === "BODY")
      {
        movedX -= canvas.offsetLeft;
        movedY -= canvas.offsetTop;
      }
      else
      {
        movedX -= canvas.offsetParent.offsetLeft;
        movedY -= canvas.offsetParent.offsetTop;
      }
      
      var delx = (movedX - centerX)
      var dely = (movedY - centerY)

      var rlen= Math.sqrt(delx **2 + dely  ** 2)
      if (rlen > maxMoveStick){
        movedX = delx * maxMoveStick/rlen + centerX
        movedY = dely * maxMoveStick/rlen + centerY

      }
      
      // Delete canvas
      context.clearRect(0, 0, canvas.width, canvas.height);
      // Redraw object
      drawExternal();
      drawInternal();
    }
  } 

  function onTouchEnd(event) 
  {
    pressed = 0;
    // If required reset position store variable
    if(autoReturnToCenter)
    {
      movedX = centerX;
      movedY = centerY;
    }
    // Delete canvas
    context.clearRect(0, 0, canvas.width, canvas.height);
    // Redraw object
    drawExternal();
    drawInternal();
    //canvas.unbind('touchmove');
  }

  /**
   * @desc Events for manage mouse
   */
  function onMouseDown(event) 
  {
    pressed = 1;
  }

  function onMouseMove(event) 
  {
    if(pressed === 1)
    {
      movedX = event.pageX;
      movedY = event.pageY;
      // Manage offset
      if(canvas.offsetParent.tagName.toUpperCase() === "BODY")
      {
        movedX -= canvas.offsetLeft;
        movedY -= canvas.offsetTop;
      }
      else
      {
        movedX -= canvas.offsetParent.offsetLeft;
        movedY -= canvas.offsetParent.offsetTop;
      }

      var delx = (movedX - centerX)
      var dely = (movedY - centerY)

      var rlen= Math.sqrt(delx **2 + dely  ** 2)
      if (rlen > maxMoveStick){
        movedX = delx * maxMoveStick/rlen + centerX
        movedY = dely * maxMoveStick/rlen + centerY

      }
      
      // Delete canvas
      context.clearRect(0, 0, canvas.width, canvas.height);
      // Redraw object
      drawExternal();
      drawInternal();
    }
  }

  function onMouseUp(event) 
  {
    pressed = 0;
    // If required reset position store variable
    if(autoReturnToCenter)
    {
      movedX = centerX;
      movedY = centerY;
    }
    // Delete canvas
    context.clearRect(0, 0, canvas.width, canvas.height);
    // Redraw object
    drawExternal();
    drawInternal();
    //canvas.unbind('mousemove');
  }

  /******************************************************
   * Public methods
   *****************************************************/
  
  /**
   * @desc The width of canvas
   * @return Number of pixel width 
   */
  this.GetWidth = function () 
  {
    return canvas.width;
  };
  
  /**
   * @desc The height of canvas
   * @return Number of pixel height
   */
  this.GetHeight = function () 
  {
    return canvas.height;
  };
  
  /**
   * @desc The X position of the cursor relative to the canvas that contains it and to its dimensions
   * @return Number that indicate relative position
   */
  this.GetPosX = function ()
  {
    return movedX;
  };
  
  /**
   * @desc The Y position of the cursor relative to the canvas that contains it and to its dimensions
   * @return Number that indicate relative position
   */
  this.GetPosY = function ()
  {
    return movedY;
  };
  
  /**
   * @desc Normalizzed value of X move of stick
   * @return Integer from -100 to +100
   */
  this.GetX = function ()
  {
    return (100*((movedX - centerX)/maxMoveStick)).toFixed();
  };

  /**
   * @desc Normalizzed value of Y move of stick
   * @return Integer from -100 to +100
   */
  this.GetY = function ()
  {
    return ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed();
  };
  
  /**
   * @desc Get the direction of the cursor as a string that indicates the cardinal points where this is oriented
   * @return String of cardinal point N, NE, E, SE, S, SW, W, NW and C when it is placed in the center
   */
  this.GetDir = function()
  {
    var result = "";
    var orizontal = movedX - centerX;
    var vertical = movedY - centerY;
    
    if(vertical >= directionVerticalLimitNeg && vertical <= directionVerticalLimitPos)
    {
      result = "C";
    }
    if(vertical < directionVerticalLimitNeg)
    {
      result = "N";
    }
    if(vertical > directionVerticalLimitPos)
    {
      result = "S";
    }
    
    if(orizontal < directionHorizontalLimitNeg)
    {
      if(result === "C")
      { 
        result = "W";
      }
      else
      {
        result += "W";
      }
    }
    if(orizontal > directionHorizontalLimitPos)
    {
      if(result === "C")
      { 
        result = "E";
      }
      else
      {
        result += "E";
      }
    }
    
    return result;
  };
});
    
    </script>
  </head>
  <body>
    <div class="columnLateral">
      <div id="joy1Div" style="width:200px;height:200px;margin:50px"></div>
      Posizione X:<input id="joy1PosizioneX" type="text" /><br />
      Posizione Y:<input id="joy1PosizioneY" type="text" /><br />
      Direzione:<input id="joy1Direzione" type="text" /><br />
      X :<input id="joy1X" type="text" /></br>
      Y :<input id="joy1Y" type="text" />
    </div>   
    <script type="text/javascript">
// Create JoyStick object into the DIV 'joy1Div'

function send(x, y)
{
  var xhttp = new XMLHttpRequest();
  
  xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
        }
      };

  var str = "val=";
  var res = str.concat(x, '_', y);
  xhttp.open("GET", res, true);
  xhttp.send()
}

var Joy1 = new JoyStick('joy1Div');

var joy1IinputPosX = document.getElementById("joy1PosizioneX");
var joy1InputPosY = document.getElementById("joy1PosizioneY");
var joy1Direzione = document.getElementById("joy1Direzione");
var joy1X = document.getElementById("joy1X");
var joy1Y = document.getElementById("joy1Y");

setInterval(function(){ joy1IinputPosX.value=Joy1.GetPosX(); }, 50);
setInterval(function(){ joy1InputPosY.value=Joy1.GetPosY(); }, 50);
setInterval(function(){ joy1Direzione.value=Joy1.GetDir(); }, 50);
setInterval(function(){ joy1X.value=Joy1.GetX(); }, 50);
setInterval(function(){ joy1Y.value=Joy1.GetY(); }, 50);
setInterval(function () { send(joy1X.value, joy1Y.value); }, 100);
    </script>
  </body>
</html>
)===";
