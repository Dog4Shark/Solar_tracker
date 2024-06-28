<?php
header('Content-Type: application/json');
require('connection.php');

$id = isset($_GET['id']) ? intval($_GET['id']) : 1;

$query = "SELECT * FROM data WHERE id = '$id'";
$result = $conn->query($query);

if ($result->num_rows > 0) {
    $row = $result->fetch_assoc();
    echo json_encode($row);
} else {
    echo json_encode(["error" => "No data found for the given id"]);
}

$conn->close();
?>
