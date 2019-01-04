<?php

function parseToXML($htmlStr)
{
  $xmlStr=str_replace('<','&lt;',$htmlStr);
  $xmlStr=str_replace('>','&gt;',$xmlStr);
  $xmlStr=str_replace('"','&quot;',$xmlStr);
  $xmlStr=str_replace("'",'&#39;',$xmlStr);
  $xmlStr=str_replace("&",'&amp;',$xmlStr);
  return $xmlStr;
}

$dir = 'sqlite:/home/tarinoit/public_html/cryologger.org/db/cryologger.db';
$db  = new PDO($dir) or die("Unable to open $dir");
$table_name = "cryologger_itb";
$imei = [300434063418130,300434063415110,300434063419120, 300434063411050, 300434063415160, 300434063416060];

header("Content-type: text/xml");

// Start XML file, echo parent node
echo "<?xml version='1.0' ?>";
echo '<polylines>';
$ind = 0;
// Iterate through the rows, printing XML nodes for each
for ($i = 0; $i < sizeof($imei); $i++) {
    $result = $db->query('SELECT * FROM '.$table_name.' WHERE imei = '.$imei[$i].' EXCEPT SELECT * FROM '.$table_name.' WHERE imei = 300434063418130 AND momsn < 358');
    $result->setFetchMode(PDO::FETCH_ASSOC);
    
    while ($row = $result->fetch()) {
      // Add to XML document node
      echo '<polyline ';
      echo 'latitude="' . $row['latitude'] . '" ';
      echo 'longitude="' . $row['longitude'] . '" ';
      echo 'type="' . $row['imei'] . '" ';
      echo '/>';
      $ind = $ind + 1;
    }
}
// End XML file
echo '</polylines>';

$db = null;

?>