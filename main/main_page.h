/*
 * main_page.h
 *
 *  Created on: Dec 16, 2018
 *      Author: brett
 */

#ifndef MAIN_MAIN_PAGE_H_
#define MAIN_MAIN_PAGE_H_

#include <string>

//todo: add react web UI to the main page
//todo: make sure this string is not being loaded into RAM

const std::string main_page_raw = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>slider demo</title>
  <link rel="stylesheet" href="//code.jquery.com/ui/1.12.1/themes/smoothness/jquery-ui.css">
  <style>#slider { margin: 10px; }  </style>
  <script src="//code.jquery.com/jquery-1.12.4.js"></script>
  <script src="//code.jquery.com/ui/1.12.1/jquery-ui.js"></script>
  <style>
    .slidecontainer {
      width: 100%;
    }
    
    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 25px;
      background: #d3d3d3;
      outline: none;
      opacity: 0.7;
      -webkit-transition: .2s;
      transition: opacity .2s;
    }
    
    .slider:hover {
      opacity: 1;
    }
  </style>
  <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js"></script>
</head>
<body>

<h1>Custom Range Slider</h1>
<p>Drag the slider to display the current value.</p>

<div class="slidecontainer">
  <input type="range" min="0" max="180" value="90" class="slider" id="slider1">
</div>
<div>
  <p id="state"><p>
</div>


<script>
  var slider = document.getElementById("slider1");
  var output = document.getElementById("state");
  slider.onclick = function() {
    output.innerHTML = this.value;
    $.post( "axes/first", { value: this.value } )
      .done(function (data) {
        //alert( "Data Loaded: " + data );
      })
      .fail(function(data) {
        alert( "POST to the board failed with message: " + data );
      });
  };
</script>

</body>
</html>
)";


#endif /* MAIN_MAIN_PAGE_H_ */
