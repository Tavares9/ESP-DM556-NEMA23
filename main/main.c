
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* =========================
   CONFIGURACAO DO MOTOR
   ========================= */

#define STEP_PIN GPIO_NUM_25
#define DIR_PIN  GPIO_NUM_27
#define BUTTON_PIN GPIO_NUM_13

#define RMT_RESOLUTION_HZ 1000000  // 1 tick = 1 us

#define SPEED_HZ 10000             // Frequencia STEP: 10 kHz
#define PULSE_HIGH_US 10           // STEP HIGH durante 10 us

#define CHUNK_SIZE 256             // Pulsos por lote RMT
#define MOTOR_RUN_TIME_MS 2000
#define BUTTON_DEBOUNCE_MS 50

#define MOTOR_PULSE_COUNT \
    ((uint32_t)(((uint64_t)SPEED_HZ * MOTOR_RUN_TIME_MS) / 1000))

/* =========================
   VARIAVEIS GLOBAIS
   ========================= */

static const char *TAG = "stepper";

static rmt_channel_handle_t rmt_channel = NULL;
static rmt_encoder_handle_t encoder = NULL;

static rmt_symbol_word_t pulse_symbols[CHUNK_SIZE];

static TaskHandle_t motor_task_handle = NULL;

static volatile TickType_t last_button_tick = 0;
static volatile bool motor_busy = false;

static void motor_task(void *arg);


/* ============================================================
   WIFI - MANTIDO DO CODIGO ORIGINAL
   ============================================================ */

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED) {

        esp_err_t err = esp_wifi_connect();

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi reconnect failed: %s",
                     esp_err_to_name(err));
        }

    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG,
                 "Connected to Wi-Fi. OTA page: http://" IPSTR "/",
                 IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t connect_wifi(void)
{
    if (CONFIG_STEPPER_WIFI_SSID[0] == '\0') {
        ESP_LOGW(TAG, "Wi-Fi SSID is not configured");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_LOGW(TAG, "Reinitializing NVS");

        ESP_ERROR_CHECK(nvs_flash_erase());

        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Could not create Wi-Fi station interface");
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            wifi_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_event_handler,
            NULL
        )
    );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_STEPPER_WIFI_SSID,
            .password = CONFIG_STEPPER_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config)
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    return esp_wifi_connect();
}


/* ============================================================
   OTA - MANTIDO DO CODIGO ORIGINAL
   ============================================================ */

static esp_err_t ota_page_handler(httpd_req_t *request)
{
    static const char page[] =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width'>"
        "<title>ESP32 firmware update</title></head><body>"
        "<h1>ESP32 firmware update</h1>"
        "<form id='form'><label>OTA password: <input id='password' type='password' required></label><br><br>"
        "<input id='firmware' type='file' accept='.bin' required><br><br>"
        "<button>Upload firmware</button></form><p id='status'></p>"
        "<script>document.getElementById('form').onsubmit=async e=>{e.preventDefault();"
        "const f=document.getElementById('firmware').files[0];"
        "const s=document.getElementById('status');s.textContent='Uploading...';"
        "try{const r=await fetch('/update',{method:'POST',headers:{'X-OTA-Password':"
        "document.getElementById('password').value},body:f});"
        "s.textContent=await r.text();}catch(_){s.textContent='Connection lost. The ESP32 may be restarting.';}};</script>"
        "</body></html>";

    httpd_resp_set_type(request, "text/html");

    return httpd_resp_send(
        request,
        page,
        HTTPD_RESP_USE_STRLEN
    );
}

static void abort_ota(esp_ota_handle_t update_handle)
{
    esp_err_t err = esp_ota_abort(update_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_abort failed: %s",
                 esp_err_to_name(err));
    }
}

static esp_err_t ota_upload_handler(httpd_req_t *request)
{
    char supplied_password[sizeof(CONFIG_STEPPER_OTA_PASSWORD)];

    size_t password_length =
        httpd_req_get_hdr_value_len(request, "X-OTA-Password");

    if (password_length != strlen(CONFIG_STEPPER_OTA_PASSWORD) ||
        password_length >= sizeof(supplied_password) ||
        httpd_req_get_hdr_value_str(
            request,
            "X-OTA-Password",
            supplied_password,
            sizeof(supplied_password)
        ) != ESP_OK ||
        strcmp(
            supplied_password,
            CONFIG_STEPPER_OTA_PASSWORD
        ) != 0) {

        httpd_resp_set_status(request, "401 Unauthorized");

        return httpd_resp_sendstr(
            request,
            "Invalid OTA password"
        );
    }

    if (request->content_len <= 0) {
        httpd_resp_set_status(request, "400 Bad Request");

        return httpd_resp_sendstr(
            request,
            "Firmware file is empty"
        );
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL ||
        (size_t)request->content_len > update_partition->size) {

        ESP_LOGE(TAG,
                 "No OTA partition available or firmware exceeds its size");

        httpd_resp_set_status(request, "413 Payload Too Large");

        return httpd_resp_sendstr(
            request,
            "Firmware does not fit in OTA partition"
        );
    }

    esp_ota_handle_t update_handle = 0;

    esp_err_t err = esp_ota_begin(
        update_partition,
        request->content_len,
        &update_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s",
                 esp_err_to_name(err));

        httpd_resp_set_status(
            request,
            "500 Internal Server Error"
        );

        return httpd_resp_sendstr(
            request,
            "Could not start firmware update"
        );
    }

    char *buffer = malloc(4096);

    if (buffer == NULL) {
        ESP_LOGE(TAG, "Could not allocate OTA upload buffer");

        abort_ota(update_handle);

        httpd_resp_set_status(
            request,
            "500 Internal Server Error"
        );

        return httpd_resp_sendstr(
            request,
            "Not enough memory"
        );
    }

    int remaining = request->content_len;

    while (remaining > 0) {
        int received = httpd_req_recv(
            request,
            buffer,
            remaining < 4096 ? remaining : 4096
        );

        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }

        if (received <= 0) {
            ESP_LOGE(TAG, "Firmware upload interrupted");

            free(buffer);
            abort_ota(update_handle);

            httpd_resp_set_status(
                request,
                "500 Internal Server Error"
            );

            return httpd_resp_sendstr(
                request,
                "Firmware upload interrupted"
            );
        }

        err = esp_ota_write(
            update_handle,
            buffer,
            received
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s",
                     esp_err_to_name(err));

            free(buffer);
            abort_ota(update_handle);

            httpd_resp_set_status(
                request,
                "500 Internal Server Error"
            );

            return httpd_resp_sendstr(
                request,
                "Could not write firmware"
            );
        }

        remaining -= received;
    }

    free(buffer);

    err = esp_ota_end(update_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Firmware validation failed: %s",
                 esp_err_to_name(err));

        httpd_resp_set_status(request, "400 Bad Request");

        return httpd_resp_sendstr(
            request,
            "Invalid firmware image"
        );
    }

    err = esp_ota_set_boot_partition(update_partition);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not select new firmware: %s",
                 esp_err_to_name(err));

        httpd_resp_set_status(
            request,
            "500 Internal Server Error"
        );

        return httpd_resp_sendstr(
            request,
            "Could not select new firmware"
        );
    }

    ESP_LOGI(TAG, "Firmware uploaded successfully; restarting");

    httpd_resp_sendstr(
        request,
        "Update successful. ESP32 is restarting."
    );

    vTaskDelay(pdMS_TO_TICKS(500));

    esp_restart();

    return ESP_OK;
}

static esp_err_t start_ota_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_handle_t server = NULL;

    esp_err_t err = httpd_start(&server, &config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start OTA web server: %s",
                 esp_err_to_name(err));

        return err;
    }

    const httpd_uri_t page_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ota_page_handler,
    };

    const httpd_uri_t upload_uri = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
    };

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(server, &page_uri)
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(server, &upload_uri)
    );

    ESP_LOGI(TAG, "OTA upload server is ready");

    return ESP_OK;
}


/* ============================================================
   CONFIGURACAO DO RMT
   ============================================================ */

static void setup_rmt(void)
{
    const uint32_t period_us = 1000000UL / SPEED_HZ;

    for (int i = 0; i < CHUNK_SIZE; i++) {
        pulse_symbols[i].level0 = 1;
        pulse_symbols[i].duration0 = PULSE_HIGH_US;

        pulse_symbols[i].level1 = 0;
        pulse_symbols[i].duration1 =
            period_us - PULSE_HIGH_US;
    }

    rmt_tx_channel_config_t tx_config = {
        .gpio_num = STEP_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };

    ESP_ERROR_CHECK(
        rmt_new_tx_channel(
            &tx_config,
            &rmt_channel
        )
    );

    rmt_copy_encoder_config_t encoder_config = {};

    ESP_ERROR_CHECK(
        rmt_new_copy_encoder(
            &encoder_config,
            &encoder
        )
    );

    ESP_ERROR_CHECK(rmt_enable(rmt_channel));

    ESP_LOGI(TAG,
             "RMT initialized: %d Hz, %d us HIGH",
             SPEED_HZ,
             PULSE_HIGH_US);
}


/* ============================================================
   ENVIO DE UMA QUANTIDADE DEFINIDA DE PULSOS
   ============================================================ */

static void send_pulses(uint32_t pulse_count)
{
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    while (pulse_count > 0) {
        uint32_t chunk = pulse_count;

        if (chunk > CHUNK_SIZE) {
            chunk = CHUNK_SIZE;
        }

        ESP_ERROR_CHECK(
            rmt_transmit(
                rmt_channel,
                encoder,
                pulse_symbols,
                chunk * sizeof(rmt_symbol_word_t),
                &tx_config
            )
        );

        ESP_ERROR_CHECK(
            rmt_tx_wait_all_done(
                rmt_channel,
                -1
            )
        );

        pulse_count -= chunk;
    }
}


/* ============================================================
   CONTROLE DE DIRECAO E MOVIMENTO
   ============================================================ */

static void set_direction(bool forward)
{
    gpio_set_level(
        DIR_PIN,
        forward ? 1 : 0
    );

    // Tempo para estabilizacao do sinal DIR.
    esp_rom_delay_us(20);

    ESP_LOGI(
        TAG,
        "Direction: %s",
        forward ? "forward" : "reverse"
    );
}

static void run_motor(void)
{
    ESP_LOGI(
        TAG,
        "Sending %lu STEP pulses",
        (unsigned long)MOTOR_PULSE_COUNT
    );

    send_pulses(MOTOR_PULSE_COUNT);
}


/* ============================================================
   INTERRUPCAO DO BOTAO
   ============================================================ */

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;

    if (motor_busy) {
        return;
    }

    TickType_t now = xTaskGetTickCountFromISR();

    if ((TickType_t)(now - last_button_tick) <
        pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
        return;
    }

    last_button_tick = now;

    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(
        motor_task_handle,
        &higher_priority_task_woken
    );

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* ============================================================
   CONFIGURACAO DO BOTAO
   ============================================================ */

static esp_err_t setup_button(void)
{
    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&button_config);

    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);

    if (err != ESP_OK) {
        return err;
    }

    // Permite aceitar o primeiro acionamento.
    last_button_tick =
        xTaskGetTickCount() -
        pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS);

    if (xTaskCreate(
            motor_task,
            "motor_task",
            4096,
            NULL,
            5,
            &motor_task_handle
        ) != pdPASS) {

        ESP_LOGE(TAG, "Could not create motor task");

        return ESP_ERR_NO_MEM;
    }

    err = gpio_isr_handler_add(
        BUTTON_PIN,
        button_isr_handler,
        NULL
    );

    if (err != ESP_OK) {
        return err;
    }

    return gpio_set_intr_type(
        BUTTON_PIN,
        GPIO_INTR_NEGEDGE
    );
}


/* ============================================================
   TAREFA PRINCIPAL DO MOTOR
   ============================================================ */

static void motor_task(void *arg)
{
    (void)arg;

    bool forward = false;

    while (1) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        motor_busy = true;
        ulTaskNotifyTake(pdTRUE, 0);

        forward = !forward;
        set_direction(forward);

        ESP_LOGI(
            TAG,
            "Button pressed. Moving %s",
            forward ? "forward" : "reverse"
        );

        run_motor();

        motor_busy = false;
        ESP_LOGI(TAG, "Movement completed");
    }
}


/* ============================================================
   APP MAIN
   ============================================================ */

void app_main(void)
{
    gpio_config_t dir_config = {
        .pin_bit_mask = 1ULL << DIR_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&dir_config));

    // Inicializa o controle do motor.
    setup_rmt();

    ESP_ERROR_CHECK(setup_button());

    // Wi-Fi: mantido do codigo original.
    esp_err_t wifi_err = connect_wifi();

    if (wifi_err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Could not start Wi-Fi connection: %s",
            esp_err_to_name(wifi_err)
        );
    }

    // OTA: mantido do codigo original.
    if (CONFIG_STEPPER_OTA_PASSWORD[0] == '\0') {
        ESP_LOGE(
            TAG,
            "Set an OTA password in menuconfig to enable firmware uploads"
        );
    } else {
        ESP_ERROR_CHECK(start_ota_server());
    }
}
