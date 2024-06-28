<?php
$servername = "localhost";
$dBUsername = "id22229244_admin";
$dBPassword = "23Vfz1999ujl!";
$dBName = "id22229244_esp32";
$conn = mysqli_connect($servername, $dBUsername, $dBPassword, $dBName);
if (!$conn) {
	die("Connection failed: ".mysqli_connect_error());
}
?>