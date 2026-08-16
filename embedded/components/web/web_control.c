#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "web_control.h"
#include "motor.h"

static const char *TAG = "web_server";

extern const uint8_t index_html_start[] 
    asm("_binary_index_html_start");

extern const uint8_t index_html_end[] 
    asm("_binary_index_html_end");

esp_err_t root_get_handler(httpd_req_t *req) {
    const uint32_t html_size = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start, html_size);
    return ESP_OK;
}

esp_err_t forward_post_handler(httpd_req_t *req) {

    motor_send_cmd(MOTOR_CMD_FORWARD);
    httpd_resp_sendstr(req, "forward");
    return ESP_OK;
}

esp_err_t backward_post_handler(httpd_req_t *req) {

    motor_send_cmd(MOTOR_CMD_BACKWARD);
    httpd_resp_sendstr(req, "backward");
    return ESP_OK;
}

esp_err_t left_post_handler(httpd_req_t *req) {

    motor_send_cmd(MOTOR_CMD_LEFT);
    httpd_resp_sendstr(req, "left");
    return ESP_OK;
}

esp_err_t right_post_handler(httpd_req_t *req) {

    motor_send_cmd(MOTOR_CMD_RIGHT);
    httpd_resp_sendstr(req, "right");
    return ESP_OK;
}

esp_err_t stop_post_handler(httpd_req_t *req) {

    motor_send_cmd(MOTOR_CMD_STOP);
    httpd_resp_sendstr(req, "stop");
    return ESP_OK;
}

const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL
};

const httpd_uri_t uri_forward = {
    .uri = "/forward",
    .method = HTTP_POST,
    .handler = forward_post_handler,
    .user_ctx = NULL
};

const httpd_uri_t uri_backward = {
    .uri = "/backward",
    .method = HTTP_POST,
    .handler = backward_post_handler,
    .user_ctx = NULL
};

const httpd_uri_t uri_left = {
    .uri = "/left",
    .method = HTTP_POST,
    .handler = left_post_handler,
    .user_ctx = NULL
};

const httpd_uri_t uri_right = {
    .uri = "/right",
    .method = HTTP_POST,
    .handler = right_post_handler,
    .user_ctx = NULL
};

const httpd_uri_t uri_stop = {
    .uri = "/stop",
    .method = HTTP_POST,
    .handler = stop_post_handler,
    .user_ctx = NULL
};


httpd_handle_t start_web_server(void) {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_forward));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_backward));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_left));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_right));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_stop));
    }

    return server;
}
