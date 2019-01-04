<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Cryologger</title>
  <link rel="stylesheet" href="//code.jquery.com/ui/1.12.1/themes/smoothness/jquery-ui.css">

  <style>
  * {
    box-sizing: border-box;
  }
  
  /* Optional: Makes the page fill the window. */
  html, body {
    height: 100%;
    margin: 0;
    padding: 0;
    font-family: 'Roboto','sans-serif';
  }

  a {
    color: black;
    text-decoration: none; /* No underline. */
  }
  footer {
    text-align: center;
  }

  #floating-panel {
    position: absolute;
    top: 160px;
    left: 40px;
    z-index: 5;
    background-color: #fff;
    padding: 5px 20px;
    border: 1px solid #999;
    text-align: center;
    line-height: 10px;
  }

  /* Always set the map height explicitly to define the size of the div element that contains the map. */
  #map {
    height: 90%;
  }
  /*
  #tabs {
    text-align: center;
  }
  #tab-1 {
    display: inline-block;
    margin: 0 auto;
  }
  */
  .charts {
    height: 250px; 
    width: 100%; 
  }

  .pointer {
    cursor: pointer;
  }
</style>

<!--Load the AJAX API-->
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<!--Load the jQuery tabs widget-->
<script src="//code.jquery.com/jquery-1.12.4.js"></script>
<script src="//code.jquery.com/ui/1.12.1/jquery-ui.js"></script>
<script type="text/javascript">
<?php

$dir = 'sqlite:/home/tarinoit/public_html/cryologger.org/db/cryologger.db';
$db  = new PDO($dir) or die("Unable to open $dir");

$table_name = "cryologger_itb";
$imei = 300434063418130;

if (isset($_POST['exportImei'])) {
    $imei = $_POST['exportImei'];
    
    // Filename for download
    $filename = $imei . "_" . date('Ymd') . ".csv";
    
    header("Content-Disposition: attachment; filename=\"$filename\"");
    header("Content-Type: text/csv");
    
    $out = fopen("php://output", 'w');
    
    $flag = false;
    
    $result = $db->query('SELECT * FROM '.$table_name.' WHERE imei = '.$imei.'') or die('Query failed!');
    $result->setFetchMode(PDO::FETCH_ASSOC);
    while(false !== ($row = $result->fetch())) {
        if(!$flag) {
            // Display field/column names as first row
            fputcsv($out, array_keys($row), ',', '"');
            $flag = true;
        }
        fputcsv($out, array_values($row), ',', '"');
    }
    
    fclose($out);
    exit;
}

?>
      // Load the Visualization API and the controls package.
      google.charts.load('current', {'packages':['corechart', 'controls']});

      // Set a callback to run when the Google Visualization API is loaded.
      google.charts.setOnLoadCallback(drawDashboard);

      // Callback that creates and populates a data table,
      // instantiates a dashboard, a range filter, several charts,
      // and passes in the data and draws it.
      function drawDashboard() {

        $(document).ready(function() {
          $('#beacon').change(function() {
            var imei = $(this).val();
            $.ajax({
              type: 'POST',
              url: 'getData.php',
              data: {changeImei: imei},
              async: false,
              success: function(result) {
                //alert(imei);
                //This is where you will tell it to load the new data
                dashboard.draw(result);
              },
              error: function(result) {
                alert('error');
              },
            });
          });
        });
        
        // Perform SQLite query and encode data as JSON.
        var jsonData = $.ajax({
          url: "getData.php",
          dataType: "json",
          async: false
        }).responseText;

        // Create data table out of JSON data loaded from server.
        var data = new google.visualization.DataTable(jsonData);

        // ChartRangeFilter controls relative date range of displayed data.
        changeRange = function(range) {
          control.setState({'range': {start: new Date(dateRange.max.getTime() - (range*oneDay))}});
          control.draw();
        };
        
        // ChartRangeFilter variables.
        var numDays = 7; // Past week
        var oneDay = (1000 * 60 * 60 * 24);
        var rangeDays = numDays * oneDay;
        var dateRange = data.getColumnRange(0);
        
        // Create a dashboard.
        var dashboard = new google.visualization.Dashboard(
          document.getElementById('dashboard_div'));
    
        // Create a range slider, passing some options.
        var control = new google.visualization.ControlWrapper({
          'controlType': 'ChartRangeFilter',
          'containerId': 'control_div',
          'options': {
            // Filter by the date axis.
            'filterColumnIndex': 0,
            'ui': {
              'chartType': 'LineChart',
              'chartOptions': {
                'chartArea': {'width': '80%'},
                'hAxis': {format: "yyyy-MM-dd"},
                'lineWidth': 1,
              },
            },

          },
          //'state': {'range': {'start': new Date(2018, 11, 1)}}
          //'state': {'range': {start: new Date(dateRange.max.getTime() - rangeDays)}}
        });

        // Create a line chart, passing some options.
        var lineChart1 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div1',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'series': {
              0: {'targetAxisIndex': 0, 'lineWidth': 1},
              1: {'targetAxisIndex': 1, 'lineWidth': 1}
            },
            'vAxes': {
                    // Adds titles to each axis.
                    0: {'title': 'Temperature (°C)'},
                    1: {'title': 'Pressure (kPa)'}
                  },
                  'legend': 'bottom'
                },
                'view': {'columns': [0, 1, 2]}
              });

        var lineChart2 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div2',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Pitch (°)'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 3]}
        });

        var lineChart3 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div3',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Roll (°)'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 4]}
        });

        var lineChart4 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div4',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Heading (°)'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 5]}
        });

        var lineChart5 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div5',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Latitude (°)'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 6]}
        });

        var lineChart6 = new google.visualization.ChartWrapper({
          'chartType': 'LineChart',
          'containerId': 'chart_div6',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Longitude (°)'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 7]}
        });

        var columnChart1 = new google.visualization.ChartWrapper({
          'chartType': 'AreaChart',
          'containerId': 'chart_div7',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'series': {
              0: {'targetAxisIndex': 0},
              1: {'targetAxisIndex': 1,}

            },
            'vAxes': {
                    // Adds titles to each axis.
                    0: {'title': 'Satellites'},
                    1: {'title': 'HDOP'}
                  },
                  'legend': 'bottom',
                },
                'view': {'columns': [0, 8, 9]}
              });

        var lineChart7 = new google.visualization.ChartWrapper({
          'chartType': 'AreaChart',
          'containerId': 'chart_div8',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Voltage (V)','viewWindow': {'min': 0, 'max': 8}},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 10]}
        });
        
        var columnChart2 = new google.visualization.ChartWrapper({
          'chartType': 'ColumnChart',
          'containerId': 'chart_div9',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Transmit Time (s)'},
            'legend': 'bottom'
          },
          'view': {'columns': [0, 11]}
        });

        var lineChart9 = new google.visualization.ChartWrapper({
          'chartType': 'AreaChart',
          'containerId': 'chart_div10',
          'options': {
            'hAxis': {'slantedText': false, format: "yyyy-MM-dd" },
            'vAxis': {'title': 'Message Counter'},
            'lineWidth': 1,
            'legend': 'bottom'
          },
          'view': {'columns': [0, 12]}
        });

        // Establish dependencies, declaring that 'control' drives 'lineChart'.
        dashboard.bind(control, [lineChart1, lineChart2, lineChart3, lineChart4, lineChart5, lineChart6, columnChart1, lineChart7, columnChart2, lineChart9]);

        // Draw the dashboard.
        dashboard.draw(data);
        
        // Add responsiveness to the dashboard.
        //$(window).resize(function(){
        //    dashboard.draw(data);
        //});
      }

      //Google Maps
      var customIcon = {
        300434063418130: {icon: 'http://maps.google.com/mapfiles/ms/icons/red.png'},
        300434063415110: {icon: 'http://maps.google.com/mapfiles/ms/icons/purple.png'},
        300434063419120: {icon: 'http://maps.google.com/mapfiles/ms/icons/blue.png'},
        300434063411050: {icon: 'http://maps.google.com/mapfiles/ms/icons/green.png'},
        300434063415160: {icon: 'http://maps.google.com/mapfiles/ms/icons/yellow.png'},
        300434063416060: {icon: 'http://maps.google.com/mapfiles/ms/icons/orange.png'},
      };

      var customColor = {
        300434063418130: {color: 'red'},
        300434063415110: {color: 'purple'},
        300434063419120: {color: 'blue'},
        300434063411050: {color: 'green'},
        300434063415160: {color: 'yellow'},
        300434063416060: {color: 'orange'},
      };

      function initMap() {
        var map = new google.maps.Map(document.getElementById('map'), {
          center: new google.maps.LatLng(75.0, -75.0),
          zoom: 4,
          mapTypeId: 'satellite',
          zoomControl: true,
          mapTypeControl: true,
          scaleControl: true,
          streetViewControl: false,
          fullscreenControl: true
        });

        google.maps.event.addDomListener(document.getElementById('1'), 'click', function () {
          map.setCenter(new google.maps.LatLng(65.954637,-62.243917));
          map.setZoom(9)
        });
        google.maps.event.addDomListener(document.getElementById('2'), 'click', function () {
          map.setCenter(new google.maps.LatLng(74.145697,-75.607582));
          map.setZoom(7)
        });
        google.maps.event.addDomListener(document.getElementById('3'), 'click', function () {
          map.setCenter(new google.maps.LatLng(77.4681,-77.787783));
          map.setZoom(9)
        });
        google.maps.event.addDomListener(document.getElementById('4'), 'click', function () {
          map.setCenter(new google.maps.LatLng(77.96864,-78.445112));
          map.setZoom(9)
        });
        google.maps.event.addDomListener(document.getElementById('5'), 'click', function () {
          map.setCenter(new google.maps.LatLng(67.406613,-63.310997));
          map.setZoom(9)
        });
        google.maps.event.addDomListener(document.getElementById('6'), 'click', function () {
          map.setCenter(new google.maps.LatLng(67.305345,-63.213673));
          map.setZoom(9)
        });
        google.maps.event.addDomListener(document.getElementById('7'), 'click', function () {
          map.setCenter(new google.maps.LatLng(75.0, -75.0));
          map.setZoom(4)
        });

        var infoWindow = new google.maps.InfoWindow;

          // Change this depending on the name of your PHP or XML file
          downloadUrl('http://cryologger.org/xml.php', function(data) {
            var xml = data.responseXML;
            var markers = xml.documentElement.getElementsByTagName('marker');
            var group = {};
            Array.prototype.forEach.call(markers, function(markerElem) {
              var time = markerElem.getAttribute('transmit_time');
              var unixtime = markerElem.getAttribute('unixtime');
              var type = markerElem.getAttribute('type');
              var point = new google.maps.LatLng(
                parseFloat(markerElem.getAttribute('latitude')),
                parseFloat(markerElem.getAttribute('longitude')));
              var latitude = markerElem.getAttribute('latitude');
              var longitude = markerElem.getAttribute('longitude');
              var temperature = markerElem.getAttribute('temperature');
              var pressure = markerElem.getAttribute('pressure');
              var pitch = markerElem.getAttribute('pitch');
              var roll = markerElem.getAttribute('roll');
              var heading = markerElem.getAttribute('heading');
              var satellites = markerElem.getAttribute('satellites');
              var hdop = markerElem.getAttribute('hdop');
              var voltage = markerElem.getAttribute('voltage');
              
              if (type in group) {
                group[type].push(point);
              } else {
                group[type] = [point];
              }

              var infowincontent = document.createElement('div');
              var strong = document.createElement('strong');
              strong.textContent = type
              infowincontent.appendChild(strong);
              infowincontent.appendChild(document.createElement('br'));

              var text = document.createElement('text');
              text.setAttribute('style', 'white-space: pre;');
              text.textContent = "Transmit time:\t" + time + " UTC"
              + "\r\nSample time:\t\t" + unixtime + " UTC"
              + "\r\nLatitude:\t\t\t" + latitude + "°" 
              + "\r\nLongitude:\t\t" + longitude + "°"
              + "\r\nTemperature:\t" + temperature + "°C" 
              + "\r\nPressure:\t\t" + pressure + " hPa"
              + "\r\nPitch:\t\t\t" + pitch + "°"
              + "\r\nRoll:\t\t\t" + roll + "°"
              + "\r\nHeading:\t\t" + heading + "°"
              + "\r\nSatellites:\t\t" + satellites
              + "\r\nHDOP:\t\t\t" + hdop
              + "\r\nVoltage:\t\t\t" + voltage + " V"
              infowincontent.appendChild(text);

              var icon = customIcon[type] || {};
              var marker = new google.maps.Marker({
                map: map,
                position: point,
                icon: icon.icon,
              });

              marker.addListener('click', function() {
                infoWindow.setContent(infowincontent);
                infoWindow.open(map, marker);
              });

            }); // End of Array.protoype.forEach.call()

            for (type in group) {
              var color = customColor[type] || {};
              var polyline = new google.maps.Polyline({
                map: map,
                path: group[type],
                strokeColor: color.color,
                strokeOpacity: 1.0,
                strokeWeight: 2
              });
            }
          }); // End of downloadUrl()
          
          // Change this depending on the name of your PHP or XML file
          downloadUrl('http://cryologger.org/xml2.php', function(data) {
            var xml = data.responseXML;
            var polylines = xml.documentElement.getElementsByTagName('polyline');
            var group = {};
            Array.prototype.forEach.call(polylines, function(markerElem) {
              var type = markerElem.getAttribute('type');
              var point = new google.maps.LatLng(
                parseFloat(markerElem.getAttribute('latitude')),
                parseFloat(markerElem.getAttribute('longitude')));

              if (type in group) {
                group[type].push(point);
              } else {
                group[type] = [point];
              }

            }); // End of Array.protoype.forEach.call()

            for (type in group) {
              var color = customColor[type] || {};
              var polyline = new google.maps.Polyline({
                map: map,
                path: group[type],
                strokeColor: color.color,
                strokeOpacity: 1.0,
                strokeWeight: 2
              });
            }
          }); // End of downloadUrl()
        } // End of function initMap()

        function downloadUrl(url, callback) {
          var request = window.ActiveXObject ?
          new ActiveXObject('Microsoft.XMLHTTP') :
          new XMLHttpRequest;

          request.onreadystatechange = function() {
            if (request.readyState == 4) {
              request.onreadystatechange = doNothing;
              callback(request, request.status);
            }
          };

          request.open('GET', url, true);
          request.send(null);
        }

        function changeCenter() {
          map.setCenter({lat: -34, lng: 151}); 
        }

        function doNothing() {}

      </script>
      <script async defer src="http://maps.googleapis.com/maps/api/js?key=AIzaSyCH-plKZjrnPNv3yV16nteqRK7BJGflZ5A&callback=initMap" type="text/javascript"></script>
    </head>

    <body>
      <div id="tabs">
        <ul>
          <li><a href="#tab-1"><span>Data</span></a></li>
          <li><a href="#tab-2"><span>Map</span></a></li>
          <li><a href="#tab-3"><span>Export</span></a></li>
        </ul>

        <div id="tab-1">
          <!--Table and divs that hold the dashboard and charts-->

          <h3> Cryologger Tracking Beacon Data:</h3>

          <select id="beacon" >
            <option value="300434063418130">Beacon 1</option>
            <option value="300434063415110">Beacon 2</option>
            <option value="300434063419120">Beacon 3</option>
            <option value="300434063411050">Beacon 4</option>
            <option value="300434063415160">Beacon 5</option>
            <option value="300434063416060">Beacon 6</option>
          </select>
          
          <button onclick="changeRange(7);"> 1 week </button>
          <button onclick="changeRange(30);"> 1 month </button>
          <button onclick="changeRange(365);"> 1 year </button>
  
          <!--Div that will hold the dashboard-->
          <div id="dashboard_div">
            <!--Divs that will hold each control and chart-->
            <div id="chart_div1" class="charts"></div>
            <div id="chart_div2" class="charts"></div>
            <div id="chart_div3" class="charts"></div>
            <div id="chart_div4" class="charts"></div>
            <div id="chart_div5" class="charts"></div>
            <div id="chart_div6" class="charts"></div>
            <div id="chart_div7" class="charts"></div>
            <div id="chart_div8" class="charts"></div>
            <div id="chart_div9" class="charts"></div>
            <div id="chart_div10" class="charts"></div>
            <div id="control_div" class="charts" style='height: 100px;'></div>
          </div>
        </div>

        <div id="tab-2">
          <div id="map"></div>
          <div id="floating-panel">
            <p class="pointer" id="1">Beacon 1 <img src='http://maps.google.com/mapfiles/ms/icons/red.png' height="15"></p>
            <p class="pointer" id="2">Beacon 2 <img src='http://maps.google.com/mapfiles/ms/icons/purple.png' height="15"></p>
            <p class="pointer" id="3">Beacon 3 <img src='http://maps.google.com/mapfiles/ms/icons/blue.png' height="15"></p>
            <p class="pointer" id="4">Beacon 4 <img src='http://maps.google.com/mapfiles/ms/icons/green.png' height="15"></p>
            <p class="pointer" id="5">Beacon 5 <img src='http://maps.google.com/mapfiles/ms/icons/yellow.png' height="15"></p>
            <p class="pointer" id="6">Beacon 6 <img src='http://maps.google.com/mapfiles/ms/icons/orange.png' height="15"></p>
            <button id="7">Reset zoom</button>
          </div>

        </div>
        
        <div id="tab-3">
          <h4>Export data to .csv</h4>
          <ul>
            <li>Beacon 1</li>
            <li>Beacon 2</li>
            <li>Beacon 3</li>
            <li>Beacon 4</li>
            <li>Beacon 5</li>
            <li>Beacon 6</li>
          </ul>
          <button onclick="self.location.href = '/export.php';">Download Beacon 2 data</button>
          
          <form action="" method="post" target="_blank">
      <select name="exportImei" onchange="this.form.submit();" >
        <option value="300434063418130" selected>Select beacon</option>
        <option value="300434063418130">Beacon 1 (S/N 13651)</option>
        <option value="300434063415110">Beacon 2 (S/N 13652)</option>
        <option value="300434063419120">Beacon 3 (S/N 13653)</option>
        <option value="300434063411050">Beacon 4 (S/N 13654)</option>
        <option value="300434063415160">Beacon 5 (S/N 13655)</option>
        <option value="300434063416060">Beacon 6 (S/N 13656)</option>
      </select>
      </form>
        </div>
      </div>
      <script>
        // jQuery UI Tabs
        $( "#tabs" ).tabs();
      </script>
    </body>
    </html>