<?php
include 'connection.php';

header('Content-Type: application/json');

if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $data = json_decode(file_get_contents('php://input'), true);
    
    if (isset($data['id'])) {
        $id = $data['id'];
        $query = "UPDATE data SET updated_by_app = 0 WHERE id='$id'";
        $stmt = $conn->prepare($query);

        if ($stmt->execute()) {
            echo json_encode(array("message" => "date_updated flag cleared"));
        } else {
            echo json_encode(array("message" => "Failed to clear date_updated flag"));
        }

        $stmt->close();
    } else {
        echo json_encode(array("message" => "id not provided"));
    }
}

$conn->close();
?>
