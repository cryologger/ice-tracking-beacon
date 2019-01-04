<?php

$dir = 'sqlite:/home/tarinoit/public_html/cryologger.org/db/cryologger.db';
$db  = new PDO($dir) or die("Unable to open $dir");

$table_name = "cryologger_itb";
$imei = 300434063418130;

if (isset($_POST['changeImei'])) {
  $imei = $_POST['changeImei'];
}

// Write your SQLite query here (parameters from $_GET or $_POST may be used)
$query = $db->query('SELECT * FROM '.$table_name.' WHERE imei = '.$imei.' AND temperature < 100 AND temperature != 0 ORDER BY unixtime DESC');

$table = array();
$rows = array();

$table['cols'] = array(
/* Define DataTable columns here
* Each column gets its own array
* Syntax of the arrays is:
* label => column label
* type => data type of column (string, number, date, datetime, boolean)
*/
array('label' => 'Unixtime', 'type' => 'datetime'),
array('label' => 'Temperature', 'type' => 'number'),
array('label' => 'Pressure', 'type' => 'number'),
array('label' => 'Pitch', 'type' => 'number'),
array('label' => 'Roll', 'type' => 'number'),
array('label' => 'Heading', 'type' => 'number'),
array('label' => 'Latitude', 'type' => 'number'),
array('label' => 'Longitude', 'type' => 'number'),
array('label' => 'Satellites', 'type' => 'number'),
array('label' => 'HDOP', 'type' => 'number'),
array('label' => 'Voltage', 'type' => 'number'),
array('label' => 'Transmit Time', 'type' => 'number'),
array('label' => 'Message Counter', 'type' => 'number')
// Etc...
);

$query->setFetchMode(PDO::FETCH_ASSOC);
while ($row = $query->fetch()) {
// Each column needs to have data inserted via the $temp array
  $temp = array();
  // Convert unixtime to JavaScript compatible milliseconds
  $unixtime = gmdate("Y-m-d H:i:s", $row['unixtime']);
  //$unixtime = "Date(".$unixtime.")"; 
  //$temp[] = array('v' => $unixtime);
  // Assumes dates are patterned 'yyyy-MM-dd hh:mm:ss'
  preg_match('/(\d{4})-(\d{2})-(\d{2})\s(\d{2}):(\d{2}):(\d{2})/', $unixtime, $match);
  $year = (int) $match[1];
  $month = (int) $match[2] - 1; // Convert to zero-index month to match JavaScript date format
  $day = (int) $match[3];
  $hours = (int) $match[4];
  $minutes = (int) $match[5];
  $seconds = (int) $match[6];
  $temp[] = array('v' => "Date($year, $month, $day, $hours, $minutes, $seconds)");
  //$temp[] = array('v' => $row['unixtime']));
  $temp[] = array('v' => $row['temperature']);
  $temp[] = array('v' => $row['pressure']);
  $temp[] = array('v' => $row['pitch']);
  $temp[] = array('v' => $row['roll']);
  $temp[] = array('v' => $row['heading']);
  $temp[] = array('v' => $row['latitude']);
  $temp[] = array('v' => $row['longitude']);
  $temp[] = array('v' => $row['satellites']);
  $temp[] = array('v' => $row['hdop']);
  $temp[] = array('v' => $row['voltage']);
  $temp[] = array('v' => $row['transmitDuration']);
  $temp[] = array('v' => $row['iterationCounter']);
// Etc...

// Insert the temp array into $rows
  $rows[] = array('c' => $temp);
}

// Populate the table with rows of data
$table['rows'] = $rows;

// Encode the table as JSON
$jsonTable = json_encode($table);

// Return the JSON data
echo $jsonTable;

?>
