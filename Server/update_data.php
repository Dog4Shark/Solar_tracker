<?php
require('connection.php');

header('Content-Type: application/json');

function logMessage($message) {
    file_put_contents('log.txt', date('Y-m-d H:i:s') . " - " . $message . "\n", FILE_APPEND);
}

$json = file_get_contents('php://input');
logMessage("Raw input data: " . $json);

$data = json_decode($json, true);

if (json_last_error() !== JSON_ERROR_NONE) {
    die("Invalid JSON data");
}


$sql = "UPDATE data SET
    data_time = ?,
    timezone_offset = ?,
    longitude = ?,
    latitude = ?,
    azimuth = IFNULL(?, azimuth),
    elevation = IFNULL(?, elevation),
    target_azimuth = ?,
    target_elevation = ?,
    battery_capacity = ?,
    battery_voltage = ?,
    battery_charge_current = ?,
    pv_voltage = ?,
    pv_power = ?,
    zenith = ?,
    sunrise = ?,
    sunset = ?,
    is_manual_mode = ?,
    min_azimuth = IFNULL(?, min_azimuth),
    max_azimuth = IFNULL(?, max_azimuth),
    min_elevation = IFNULL(?, min_elevation),
    max_elevation = IFNULL(?, max_elevation),
    auto_azimuth = IFNULL(?, auto_azimuth),
    auto_elevation = IFNULL(?, auto_elevation),
    actuator_frequency = ?
    WHERE id = ?";

$stmt = $conn->prepare($sql);


if ($stmt === false) {
    die("Prepare failed: " . $conn->error);
}

$stmt->bind_param("siddddddddddddssiddddddii",
    $data['data_time'],
    $data['timezone_offset'],
    $data['longitude'],
    $data['latitude'],
    $data['azimuth'],
    $data['elevation'],
    $data['target_azimuth'],
    $data['target_elevation'],
    $data['battery_capacity'],
    $data['battery_voltage'],
    $data['battery_charge_current'],
    $data['pv_voltage'],
    $data['pv_power'],
    $data['zenith'],
    $data['sunrise'],
    $data['sunset'],
    $data['is_manual_mode'],
    $data['min_azimuth'],
    $data['max_azimuth'],
    $data['min_elevation'],
    $data['max_elevation'],
    $data['auto_azimuth'],
    $data['auto_elevation'],
    $data['actuator_frequency'],
    $data['id']
);


if ($stmt->execute() === false) {
    die("Execute failed: " . $stmt->error);
} else {
    echo "Record updated successfully";
}


$stmt->close();
$conn->close();
?>