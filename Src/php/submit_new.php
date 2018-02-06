<?php
printf($_SERVER['REQUEST_METHOD']);
$requestHeaders = apache_request_headers();
print_r($requestHeaders);
printf(file_get_contents('php://input'));
?>
