#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "math.h"
#include "mqtt_client.h"

#define I2C_MASTER_SCL_IO          22        
#define I2C_MASTER_SDA_IO          21        
#define I2C_MASTER_FREQ_HZ         100000    
#define MPU6050_ADDR               0x68      
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_ACCEL_XOUT_H       0x3B
#define WINDOW_SIZE                5
#define N_SAMPLES                  128      
#define SAMPLING_RATE              100      
#define BUF_SIZE                   (1024)    

#define WIFI_SSID       "iPhone"      
#define WIFI_PASSWORD   "TapsTaps"  
#define WIFI_MAX_RETRY  5

static const char *TAG = "MPU6050";

// Wi-Fi event handling
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static int retry_num = 0;

// MQTT Client handle
esp_mqtt_client_handle_t client;

// Mode variables
typedef enum { STOP, NORMAL, BLOCK, ROTOR_IMBALANCE } OperationMode;
OperationMode current_mode = STOP;

// Ring buffer for moving average filter
float accel_buffer[WINDOW_SIZE] = {0};
int buffer_index = 0;
float sum = 0;
float accel_x[N_SAMPLES];  
int sample_index = 0;

// Function Prototypes
static esp_err_t i2c_master_init();
static esp_err_t mpu6050_write(uint8_t reg, uint8_t data);
static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len);
static float moving_average_filter(float new_value);
static float compute_rms(float *data, int size);
static void read_accelerometer();
static void collect_data_for_mode(OperationMode mode);
static void uart_init();
static OperationMode read_mode_from_input();
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_app_start();

// Wi-Fi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying... (Attempt %d/%d)", retry_num, WIFI_MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(5000)); // 5-second delay between retries
        } else {
            ESP_LOGE(TAG, "Wi-Fi connection failed after %d attempts", WIFI_MAX_RETRY);
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); // Set the connected bit
        ESP_LOGI(TAG, "WIFI_CONNECTED_BIT set successfully");
    }
}

// Wi-Fi initialization
static void wifi_init_sta() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi setup completed.");
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected to broker");
            esp_mqtt_client_subscribe(client, "sensor/command", 0);
            esp_mqtt_client_publish(client, "sensor/rms", "Hello from ESP32", 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGE(TAG, "MQTT Disconnected from broker, retrying...");
            vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for 5 seconds before reconnecting
            esp_mqtt_client_reconnect(client); // Retry to connect
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error: %s", event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT ? "TCP Error" : "Other Error");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Subscribed to topic: %.*s", event->topic_len, event->topic);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT Message published");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data received: %.*s", event->data_len, event->data);
            break;
        default:
            ESP_LOGI(TAG, "MQTT Event: %d", event->event_id);
            break;
    }
}

void mqtt_app_start() {
    // Update the broker URI to use your local broker
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtt://172.20.10.4:1883",  // Your local MQTT broker
        },
    };

    ESP_LOGI(TAG, "Initializing MQTT client with URI: %s", mqtt_cfg.broker.address.uri);
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "MQTT client started successfully");
    }
}

// Initialize I2C communication
static esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    return ESP_OK;
}

// Write data to the MPU6050
static esp_err_t mpu6050_write(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read data from the MPU6050
static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function for moving average filter
static float moving_average_filter(float new_value) {
    sum -= accel_buffer[buffer_index];
    accel_buffer[buffer_index] = new_value;
    sum += new_value;
    buffer_index = (buffer_index + 1) % WINDOW_SIZE;
    return sum / WINDOW_SIZE;
}

// Read accelerometer data
static void read_accelerometer() {
    uint8_t raw_data[6];
    int16_t acc_x, acc_y, acc_z;
    mpu6050_read(MPU6050_ACCEL_XOUT_H, raw_data, 6);

    acc_x = (raw_data[0] << 8) | raw_data[1];
    acc_y = (raw_data[2] << 8) | raw_data[3];
    acc_z = (raw_data[4] << 8) | raw_data[5];

    float accel_x_raw = (float)acc_x / 16384.0;
    accel_x[sample_index++] = moving_average_filter(accel_x_raw);
    if (sample_index >= N_SAMPLES) {
        sample_index = 0;
    }
}

// Function for calculating RMS
static float compute_rms(float *data, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += data[i] * data[i];
    }
    return sqrt(sum / size);
}

// Initialize UART for user input
static void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
}

// Read mode from UART input
static OperationMode read_mode_from_input() {
    char input;
    int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&input, 1, pdMS_TO_TICKS(100));
    if (len > 0) {
        if (input == 'n' || input == 'N') {
            return NORMAL;
        } else if (input == 'b' || input == 'B') {
            return BLOCK;
        } else if (input == 'r' || input == 'R') {
            return ROTOR_IMBALANCE;
        }
    }
    return STOP;
}

// Collect data based on the current mode
static void collect_data_for_mode(OperationMode mode) {
    if (mode == STOP) {
        ESP_LOGI(TAG, "System is in STOP mode.");
    } else if (mode == NORMAL) {
        read_accelerometer();
        ESP_LOGI(TAG, "Collecting data in NORMAL mode.");
    } else if (mode == BLOCK) {
        // Block functionality
        ESP_LOGI(TAG, "Collecting data in BLOCK mode.");
    } else if (mode == ROTOR_IMBALANCE) {
        // Rotor imbalance functionality
        ESP_LOGI(TAG, "Collecting data in ROTOR_IMBALANCE mode.");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing system...");

    // Create the event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    uart_init();
    i2c_master_init();
    wifi_init_sta();

    // Wait for Wi-Fi to connect with a timeout of 30 seconds
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected, starting MQTT...");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi within 30 seconds");
        return; // Exit if Wi-Fi connection fails
    }

    // Main loop
    while (1) {
        current_mode = read_mode_from_input();
        collect_data_for_mode(current_mode);
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
    }
}