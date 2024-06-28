<?php
require('connection.php');
header('Content-Type: application/json');

$id = isset($_GET['id']) ? intval($_GET['id']) : 1;

if ($_SERVER['REQUEST_METHOD'] === 'GET') {
    $sql = $conn->prepare("SELECT updated_by_app FROM data WHERE id = ?");
    $sql->bind_param("i", $id);
    $sql->execute();
    $result = $sql->get_result();

    if ($result->num_rows > 0) {
        $row = $result->fetch_assoc();
        echo json_encode($row);
    } else {
        echo json_encode(array("message" => "No data found"));
    }
}

$conn->close();
?>