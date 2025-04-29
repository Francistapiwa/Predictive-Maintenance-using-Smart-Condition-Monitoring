#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_http_client.h"

// === CONFIGURATION ===
#define WIFI_SSID "Pulsar"
#define WIFI_PASS "7493Harare"
#define WIFI_CONNECTED_BIT BIT0

#define LIVE_FEED_WRITE_API_KEY "5FT4KSXOKUM47SKK"
#define PREDICTION_READ_API_KEY "O8NZA94VJWGDHYBI"
#define PREDICTION_CHANNEL_ID "2890570"

#define I2C_SDA 32
#define I2C_SCL 33
#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// === CAMERA PINOUT (WROVER-KIT) ===
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static const char *TAG = "FanMonitor";
static EventGroupHandle_t wifi_event_group;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu_dev = NULL;

typedef enum { STOP = 0, NORMAL = 1, IMBALANCED = 2 } FanMode;

// === CAMERA CONFIG ===
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,  // 📸 Changed to wider 320x240 resolution
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

static esp_err_t init_camera() {
    return esp_camera_init(&camera_config);
}

// === CAMERA HTTP HANDLER ===
esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb_discard = esp_camera_fb_get();
    if (fb_discard) esp_camera_fb_return(fb_discard);  // Discard first frame

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return ESP_OK;
}

httpd_handle_t start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = capture_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri);
        ESP_LOGI(TAG, "📷 Snapshot viewer running at http://<ESP_IP>/");
    }

    return server;
}

// === WIFI INIT ===
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

void wifi_init() {
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK }
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "✅ WiFi connected");
}

// === MPU6050 INIT & I2C ===
esp_err_t i2c_mpu_init() {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 100000
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpu_dev);
}

esp_err_t mpu6050_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(mpu_dev, buf, sizeof(buf), 1000);
}

esp_err_t mpu6050_read(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(mpu_dev, &reg, 1, buf, len, 1000);
}

// === THINGSPEAK ===
int fetch_prediction_mode() {
    char url[256];
    sprintf(url, "http://api.thingspeak.com/channels/%s/fields/1/last.txt?api_key=%s",
            PREDICTION_CHANNEL_ID, PREDICTION_READ_API_KEY);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 3000
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    char response[16] = {0};
    if (err == ESP_OK) {
        esp_http_client_read_response(client, response, sizeof(response));
    } else {
        ESP_LOGE(TAG, "Failed to fetch mode: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return atoi(response);
}

void send_data_to_thingspeak(float x, float y, float z, float pitch, float roll) {
    esp_http_client_config_t config = {
        .url = "http://api.thingspeak.com/update",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    char post_data[128];
    snprintf(post_data, sizeof(post_data),
        "api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f&field4=%.2f&field5=%.2f",
        LIVE_FEED_WRITE_API_KEY, x, y, z, pitch, roll);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Data sent to ThingSpeak");
    } else {
        ESP_LOGE(TAG, "ThingSpeak upload failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// === MAIN ===
void app_main(void) {
    nvs_flash_init();
    wifi_init();
    init_camera();
    i2c_mpu_init();
    mpu6050_write(MPU_PWR_MGMT_1, 0x00); // Wake MPU6050
    start_webserver();  // Snapshot viewer via http://<ESP_IP>/

    while (1) {
        uint8_t data[6];
        if (mpu6050_read(MPU_ACCEL_XOUT_H, data, 6) == ESP_OK) {
            int16_t ax = (data[0] << 8) | data[1];
            int16_t ay = (data[2] << 8) | data[3];
            int16_t az = (data[4] << 8) | data[5];

            float pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0 / M_PI;
            float roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0 / M_PI;

            ESP_LOGI(TAG, "Accel X:%d Y:%d Z:%d", ax, ay, az);
            ESP_LOGI(TAG, "Pitch: %.2f°, Roll: %.2f°", pitch, roll);

            send_data_to_thingspeak((float)ax, (float)ay, (float)az, pitch, roll);
            fetch_prediction_mode();
        } else {
            ESP_LOGE(TAG, "MPU6050 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
