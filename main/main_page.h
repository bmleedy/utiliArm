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
</style>
<script src="https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js"></script>
</head>
<body>

<h1>Custom Range Slider</h1>
<p>Drag the slider to display the current value.</p>

<div id="myslider"></div>


<script>
$( "#myslider" ).slider();
</script>

</body>
</html>
)";


#endif /* MAIN_MAIN_PAGE_H_ */
