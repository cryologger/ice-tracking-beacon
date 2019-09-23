<?php

$dir = 'sqlite:/home/tarinoit/public_html/cryologger.org/db/cryologger.db';
$db  = new PDO($dir) or die("Unable to open $dir");

$table_name = "cryologger_itb";
$imei = 300434063415110;

if (isset($_POST['changeImei'])) {
    $imei = $_POST['changeImei'];
}

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

?>