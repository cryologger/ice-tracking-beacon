<?php

/* 	
*	Rock Seven HTTP POST Request
*	Author: Adam Garbo
*	Date: July 3, 2018
*
*	For more information see: 
*	http://www.makersnake.com/rock7-core-endpoints
*	http://www.rock7mobile.com/downloads/RockBLOCK-Web-Services-User-Guide.pdf
*/

// HTTP POST data parameters
$imei = $_POST["imei"];
$momsn = $_POST["momsn"];
$transmit_time = $_POST["transmit_time"];
$iridium_latitude = $_POST["iridium_latitude"];
$iridium_longitude = $_POST["iridium_longitude"];
$iridium_cep = $_POST["iridium_cep"];
$data = $_POST["data"];

// Log incomming HTTP POST data parameters for debugging purposes
$logFile = "/home/tarinoit/public_html/cryologger.org/logs/raw_post_data.txt";
$logRawData = true; // Set to false to halt logging

if ($logRawData)
{
	writeDebug(
		"imei:" . $imei . ", momsn:" . $momsn . ", transmit_time:" . $transmit_time . ", iridium_latitude:" . $iridium_latitude . ", iridium_longitude:" . $iridium_longitude . ", iridium_cep:" . $iridium_cep . ", data:" . $data , $logFile);
}

// If HTTP POST data are valid, respond with HTTP status 200 (success)
if (!empty($imei) && !empty($data)) {
	echo "OK";	
}
else {
	exit("Bad request");
}

// Open database file
$dir = 'sqlite:/home/tarinoit/public_html/cryologger.org/db/cryologger.db';
$db  = new PDO($dir) or die("Unable to open $dir");

// Structure of hex-encoded message
/*
	For more information see: http://php.net/manual/en/function.pack.php
	v - unsigned short (16 bit, little endian byte order)
	V - unsigned long (32 bit, little endian byte order)
	s - signed short (16 bit, machine byte order)
	l - signed long (32 bit, machine byte order)
*/

$sbd_format = 
	'Vunixtime/' .				// 4
	'stemperature/' .			// 2
	'spressure/' .			    // 2
	'spitch/' .					// 2
	'sroll/' .					// 2
	'vheading/' .				// 2
	'llatitude/' .				// 4
	'llongitude/' .				// 4
	'vsatellites/' .			// 2
	'vhdop/' .					// 2
	'vvoltage/' .				// 2
	'vtransmitDuration/' .      // 2
	'viterationCounter';		// 2
								// 32

// Add quotes to transmit_time and data variables before database insertion (SQLite requirement)
	$transmit_time_txt = addQuotes($transmit_time); 
	$data_txt = addQuotes($data);

$len_data = strlen($data); 							// Calculate length of data parameter 
$message_size = 64; 								// Size of each hex-encoded data string multiplied by 2 (e.g. 30 bytes x 2 = 60)
$num_messages = $len_data / $message_size; 			// Calculate number of individual hex-encoded messages in data parameter
$sbd_raw = (str_split($data, $message_size)); 		// Split data parameter string into individual hex-encoded messages

for ($i = 0; $i < $num_messages; $i++) {
	$sbd_hex = pack('H*', $sbd_raw[$i]);			// Pack data into binary string
	$sbd_dec = unpack($sbd_format, $sbd_hex);		// Unpack from binary string into an array according to $sbd_format

	// Revert variables to original data types
	$unixtime = $sbd_dec['unixtime'];
	$temperature = $sbd_dec['temperature'] / 100.0;
	$pressure = $sbd_dec['pressure'] / 100;
	$pitch = $sbd_dec['pitch'] / 100.0;
	$roll = $sbd_dec['roll'] / 100.0;
	$heading = $sbd_dec['heading'] / 10.0;
	$latitude = $sbd_dec['latitude'] / 1000000;
	$longitude = $sbd_dec['longitude'] / 1000000;
	$satellites = $sbd_dec['satellites'];
	$hdop = $sbd_dec['hdop'] / 100.0;
	$voltage = $sbd_dec['voltage'] / 1000.0;
	$transmitDuration = $sbd_dec['transmitDuration'];
	$iterationCounter = $sbd_dec['iterationCounter'];
	
	// Debug
	echo $unixtime;
	echo "\r\n";
	echo $temperature;
	echo "\r\n";
	echo $pressure;
	echo "\r\n";	
	echo $pitch;
	echo "\r\n";
	echo $roll;
	echo "\r\n";
	echo $heading;
	echo "\r\n";
	echo $latitude;
	echo "\r\n";
	echo $longitude;
	echo "\r\n";
	echo $satellites;
	echo "\r\n";
	echo $hdop;
	echo "\r\n";
	echo $voltage;
	echo "\r\n";
	echo $transmitDuration;
	echo "\r\n";
	echo $iterationCounter;
	echo "\r\n";
	
	// Insert variables into database
	$db->exec("INSERT INTO cryologger_itb (imei, momsn, transmit_time, iridium_latitude, iridium_longitude, iridium_cep, data, unixtime, temperature, pressure, pitch, roll, heading, latitude, longitude, satellites, hdop, voltage, transmitDuration, iterationCounter) 
		VALUES ($imei, $momsn, $transmit_time_txt, $iridium_latitude, $iridium_longitude, $iridium_cep, $data_txt, $unixtime, $temperature, $pressure, $pitch, $roll, $heading, $latitude, $longitude, $satellites, $hdop, $voltage, $transmitDuration, $iterationCounter)");
}
// Close database connection
$db = null;
echo "Success"; // Intended for debugging purposes

function writeDebug($msg,$logFile)
{
	$handle = fopen($logFile,"a") or die("Unable to open file!");
	fwrite($handle, date('Y-m-d H:i:s').": DEBUG:");
	fwrite($handle, $msg."\n");
	fclose($handle);
}

function addQuotes($str){
	return "'$str'";
}

?>
