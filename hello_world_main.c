#include <stdio.h>
#include <string.h>
#include <inttypes.h>               // fixed width integer types
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"               // AP connection
#include "esp_event.h"              // system event handling (needed for WIFI and some IP configuration)
#include "esp_log.h"
#include "nvs_flash.h"              // store wifi credentials
#include "esp_http_server.h"        // server for web interface
#include "driver/ledc.h"            // LED PWM control. used for fan duty cycle adjustment.
#include "driver/pcnt.h"            // pulse counter. used for fan TACH
#include "driver/gpio.h"            // GPIO pin driver
#include "rom/ets_sys.h"            // low level system function. needed for DHT11 reading.
#include "lwip/err.h"               // IP stack codes
#include "lwip/sys.h"               // IP error functions

#define TAG "FAN_TEMP_CONTROLLER"           // temp controller app ID flag
#define WIFI_CONNECTED_BIT BIT0 
#define WIFI_FAIL_BIT      BIT1
#define WIFI_SSID_MAX_LEN  32
#define WIFI_PASS_MAX_LEN  64
#define FAN1_PWM_GPIO 18                    // first pan PWM pin
#define FAN2_PWM_GPIO 23                    // second fan PWM pin
#define FAN1_TACH_GPIO 7                    // first fan TACH pin
#define FAN2_TACH_GPIO 6                    // second fan TACH pin
#define PWM_FREQ 25000                      // standard for 4pin fans.
#define PWM_RESOLUTION LEDC_TIMER_10_BIT    // at 10 bits, we 1023 steps for duty cycles using the LED PWM controller
#define PWM_TIMER LEDC_TIMER_0              // hardware timer for PWM pulse generation
#define PWM_MODE LEDC_LOW_SPEED_MODE        // set PWM_MOD to low speed for fan control (don't need fast pulses)
#define FAN1_PWM_CHANNEL LEDC_CHANNEL_0     // fan 1 PWM channel
#define FAN2_PWM_CHANNEL LEDC_CHANNEL_1     // fan 2 PWM channel
#define SAMPLE_PERIOD_MS 1000               // read RPM from TACH every second
#define PULSES_PER_REV 2                    // PWM pulses per full revolution (standard)
#define PCNT_FILTER_VAL 1023                // pulse counter noise filter
#define FAN1_PCNT_UNIT PCNT_UNIT_0          // hardware unit for fan 1 measurement
#define FAN1_PCNT_CHANNEL PCNT_CHANNEL_0    // channel for fan 1 hardware unit
#define FAN2_PCNT_UNIT PCNT_UNIT_1          // hardware unit for fan 2 measurement
#define FAN2_PCNT_CHANNEL PCNT_CHANNEL_1    // channel for fan 2 hardware unit
#define DHT_PIN GPIO_NUM_5                  // DHT 11 data line
#define MIN_TEMP_F 75.0f                    // 30% duty at 75 degrees
#define MAX_TEMP_F 80.0f                    // 80% duty at 80 degrees
#define MIN_DUTY_PERCENT 30
#define MAX_DUTY_PERCENT 80

// INIT for WIFI + fan duty + RPM measurement
static char ssid[WIFI_SSID_MAX_LEN] = ""; 
static char password[WIFI_PASS_MAX_LEN] = "";
static EventGroupHandle_t s_wifi_event_group;
static httpd_handle_t server = NULL;
static uint32_t current_duty1 = 512;        // 50% duty at init
static uint32_t current_duty2 = 512;
static float latest_rpm1 = 0.0f, latest_rpm2 = 0.0f;
static float current_temp_f = 75.0f;
static bool auto_mode = true;               // default to temp-based fan control

// DHT11 struct
typedef struct {
    int temperature;
    int humidity;
} dht11_reading_t;

// DHT11 pinout reading
static dht11_reading_t read_dht11_data() {
    dht11_reading_t result = {-1, -1};              // INIT with invalid values
    uint8_t data[5] = {0};                          // array to store 5 bytes from DHT11 (2 humidity, 2 temp, 1 checksum)
    uint8_t retry = 0;                              // timeout
    uint8_t i, j;                                   // loop counters for bit and byte respectively

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);  // set pin as output to send start signal
    gpio_set_level(DHT_PIN, 0);                     // pull pin low
    ets_delay_us(20000);                            // hold for 20 ms (datasheet says 18 minimum)
    gpio_set_level(DHT_PIN, 1);                     // pull high to end start signal and prepare for sensor response
    ets_delay_us(40);                               // wait 40 microseconds before switching to input

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);   // begin read from DHT11

    retry = 0;
    while (gpio_get_level(DHT_PIN) == 1) {          // wait for sensor to acknowledge (pull line low)
        if (retry++ > 100) {
            return result;
        }
        ets_delay_us(1);
    }

    retry = 0;
    while (gpio_get_level(DHT_PIN) == 0) {          // wait for 80 microsecond low pulse
        if (retry++ > 100) {
            return result;
        }
        ets_delay_us(1);
    }

    retry = 0;
    while (gpio_get_level(DHT_PIN) == 1) {          // wait for 80 microsecond high pulse before data starts
        if (retry++ > 100) {
            return result;
        }
        ets_delay_us(1);
    }

    // begin 40 bit read
    for (i = 0; i < 5; i++) {                           // for the first 5 bytes
        for (j = 0; j < 8; j++) {                       // process each bit
            retry = 0;
            while (gpio_get_level(DHT_PIN) == 0) {      // each bit starts with an 80 microsecond low pulse
                if (retry++ > 100) {
                    return result;
                }
                ets_delay_us(1);
            }

            int width = 0;                              // measure high pulse width to get bit value
            retry = 0;
            while (gpio_get_level(DHT_PIN) == 1) {      // count duration: 26-28 = 0, 70 = 1
                if (retry++ > 100) {
                    return result;
                }
                width++;
                ets_delay_us(1);
            }

            if (width > 30) {                           // if pulse width exceeds 30
                data[i] |= (1 << (7 - j));              // set bit
            }
        }
    }

    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        return result;                                   // checksum, throw out bad values
    }

    result.humidity = data[0];                           // extract humidity value
    result.temperature = data[2];                        // extract temp value

    return result;
}

static uint32_t calculate_duty_from_temp(float temp_f) {
    if (temp_f < MIN_TEMP_F) temp_f = MIN_TEMP_F;                                                   // clamp temp to min 
    if (temp_f > MAX_TEMP_F) temp_f = MAX_TEMP_F;                                                   // and max threshold
    
    float temp_percent = (temp_f - MIN_TEMP_F) / (MAX_TEMP_F - MIN_TEMP_F);                         // normalized position within 75-80
    float duty_percent = MIN_DUTY_PERCENT + temp_percent * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT);   // linear between 75 and 80 in duty cycles
    uint32_t duty_value = (uint32_t)(duty_percent * 10.23f);                                        // convert percentage to 10 bit PWM value
    
    return duty_value;
}

static void init_pwm(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = PWM_MODE,                 // low speed mode (easier to control for small fans)
        .timer_num = PWM_TIMER,                 // hardware timer for PWM generation
        .duty_resolution = PWM_RESOLUTION,      // 10 bit resolution
        .freq_hz = PWM_FREQ,                    // 25kHz frequency standard for 4pin fans
        .clk_cfg = LEDC_AUTO_CLK                // automatic ESP32 clock source detection
    };
    ledc_timer_config(&timer_cfg);             

    ledc_channel_config_t ch1 = {               
        .speed_mode = PWM_MODE,
        .channel = FAN1_PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = FAN1_PWM_GPIO,
        .duty = current_duty1,
        .hpoint = 0
    };
    ledc_channel_config_t ch2 = ch1;
    ch2.channel = FAN2_PWM_CHANNEL;
    ch2.gpio_num = FAN2_PWM_GPIO;
    ch2.duty = current_duty2;

    ledc_channel_config(&ch1);
    ledc_channel_config(&ch2);
    ledc_update_duty(PWM_MODE, FAN1_PWM_CHANNEL);
    ledc_update_duty(PWM_MODE, FAN2_PWM_CHANNEL);
}

static void init_pcnt(pcnt_unit_t unit, pcnt_channel_t chan, gpio_num_t gpio) {
    pcnt_config_t pcnt_cfg = {
        .pulse_gpio_num = gpio,             // tach pin
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // no GPIO control pin
        .channel = chan,                    // pule counter unit channel
        .unit = unit,
        .pos_mode = PCNT_COUNT_DIS,         // don't count on rising edge
        .neg_mode = PCNT_COUNT_INC,         // count on falling edge
        .lctrl_mode = PCNT_MODE_KEEP,       // no control pin
        .hctrl_mode = PCNT_MODE_KEEP        // no control pin
    };
    pcnt_unit_config(&pcnt_cfg);            // apply to hardware
    pcnt_set_filter_value(unit, PCNT_FILTER_VAL);   // electrical noise filter
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);                       // pause before counter rest
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);                      // begin counting pulses
}

static void rpm_task(void *arg) {
    int16_t pulse1, pulse2;                         // pulse counts for each fan
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));    // 1 sec between measurements

        pcnt_counter_pause(FAN1_PCNT_UNIT);             // pause fan1 counter for reading
        pcnt_get_counter_value(FAN1_PCNT_UNIT, &pulse1); // read pulses from the last second
        pcnt_counter_clear(FAN1_PCNT_UNIT);
        pcnt_counter_resume(FAN1_PCNT_UNIT);

        pcnt_counter_pause(FAN2_PCNT_UNIT);             // repeat for fan 2
        pcnt_get_counter_value(FAN2_PCNT_UNIT, &pulse2);
        pcnt_counter_clear(FAN2_PCNT_UNIT);
        pcnt_counter_resume(FAN2_PCNT_UNIT);

        latest_rpm1 = (float)pulse1 * 60.0f / (PULSES_PER_REV * 1.0f); // convert pulses per second to RPM. pulses * 60 / revs. 2 pulses per revolution.
        latest_rpm2 = (float)pulse2 * 60.0f / (PULSES_PER_REV * 1.0f);    
    }
}

static void temp_control_task(void *arg) {
    while (1) {
        dht11_reading_t reading = read_dht11_data();                    // read DHT11 data
        
        if (reading.temperature != -1) {
            int fahrenheit = (reading.temperature * 9 / 5) + 32;        // convert to Fahrehnheit (we live in America)
            current_temp_f = (float)fahrenheit;                         // update temp variable
            
            if (auto_mode) {                                            // if in automatic mode
                uint32_t new_duty = calculate_duty_from_temp(current_temp_f);   // new duty = updated within spec range
                
                current_duty1 = new_duty;
                current_duty2 = new_duty;
                
                ledc_set_duty(PWM_MODE, FAN1_PWM_CHANNEL, current_duty1);       // set new PWM channel
                ledc_set_duty(PWM_MODE, FAN2_PWM_CHANNEL, current_duty2);
                ledc_update_duty(PWM_MODE, FAN1_PWM_CHANNEL);                   // apply change
                ledc_update_duty(PWM_MODE, FAN2_PWM_CHANNEL);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// WIFI stuff
static esp_err_t slider_get_handler(httpd_req_t *req) { 
    const char *html_head =
        "<style>"
        "body{font-family:Arial,sans-serif;margin:20px}"
        ".container{max-width:600px;margin:0 auto}"
        ".status{background-color:#f0f0f0;padding:15px;border-radius:5px;margin-bottom:20px}"
        ".control-panel{background-color:#e6f7ff;padding:15px;border-radius:5px}"
        ".slider-container{margin-top:10px}"
        "</style>"
        "<div class='container'>"
        "<h1>ESP32 Fan Controller</h1>";
    
    const char *html_status =
        "<div class='status'>"
        "<h2>Status</h2>"
        "<p>Server Case Temperature: <span id='temp'>%.1f</span> F</p>"
        "</div>";
    
    const char *html_control =
        "<div class='control-panel'>"
        "<h2>Control Mode</h2>"
        "<label><input type='checkbox' id='auto-mode' %s onchange='toggleMode()'> Automatic Mode</label>";
    
    const char *html_fan1 =
        "<h2>Fan 1 Control</h2>"
        "<div class='slider-container'>"
        "<input type='range' id='fan1' min='0' max='100' value='%d' step='5' style='width:300px;' %s "
        "oninput='adjust(1,this.value)'>"
        "<p>Fan 1 Speed: <span id='val1'>%d</span>%%</p>"
        "<p>Fan 1 RPM: <span id='rpm1'>%.1f</span></p>"
        "</div>";
    
    const char *html_fan2 =
        "<h2>Fan 2 Control</h2>"
        "<div class='slider-container'>"
        "<input type='range' id='fan2' min='0' max='100' value='%d' step='5' style='width:300px;' %s "
        "oninput='adjust(2,this.value)'>"
        "<p>Fan 2 Speed: <span id='val2'>%d</span>%%</p>"
        "<p>Fan 2 RPM: <span id='rpm2'>%.1f</span></p>"
        "</div>"
        "</div>";
    
    const char *html_script =
        "<script>"
        "function adjust(fan,val){"
        "fetch(`/set?fan=${fan}&duty=${Math.round(val*10.23)}`);"
        "if(fan==1)document.getElementById('val1').innerText=val;"
        "else document.getElementById('val2').innerText=val;"
        "}"
        "function toggleMode(){"
        "const auto=document.getElementById('auto-mode').checked;"
        "fetch(`/mode?auto=${auto?1:0}`);"
        "document.getElementById('fan1').disabled=auto;"
        "document.getElementById('fan2').disabled=auto;"
        "}"
        "setInterval(function(){"
        "fetch('/data').then(r=>r.text()).then(json=>{"
        "const data=JSON.parse(json);"
        "document.getElementById('rpm1').innerText=data.rpm1;"
        "document.getElementById('rpm2').innerText=data.rpm2;"
        "document.getElementById('temp').innerText=data.temp;"
        "if(data.auto){"
        "document.getElementById('fan1').disabled=true;"
        "document.getElementById('fan2').disabled=true;"
        "document.getElementById('val1').innerText=Math.round(data.duty1/10.23);"
        "document.getElementById('val2').innerText=Math.round(data.duty2/10.23);"
        "document.getElementById('fan1').value=Math.round(data.duty1/10.23);"
        "document.getElementById('fan2').value=Math.round(data.duty2/10.23);"
        "}"
        "});"
        "},1000);"
        "</script></div>";

    int percent1 = (int)(current_duty1 * 100 / 1023);
    int percent2 = (int)(current_duty2 * 100 / 1023);
    const char *checked = auto_mode ? "checked" : "";
    const char *disabled = auto_mode ? "disabled" : "";
    
    httpd_resp_set_type(req, "text/html");
    
    httpd_resp_send_chunk(req, html_head, strlen(html_head));
    
    char temp_buf[256];
    snprintf(temp_buf, sizeof(temp_buf), html_status, current_temp_f);
    httpd_resp_send_chunk(req, temp_buf, strlen(temp_buf));
    
    char control_buf[256];
    snprintf(control_buf, sizeof(control_buf), html_control, checked);
    httpd_resp_send_chunk(req, control_buf, strlen(control_buf));
    
    char fan1_buf[512];
    snprintf(fan1_buf, sizeof(fan1_buf), html_fan1, 
             percent1, disabled, percent1, latest_rpm1);
    httpd_resp_send_chunk(req, fan1_buf, strlen(fan1_buf));
    
    char fan2_buf[512];
    snprintf(fan2_buf, sizeof(fan2_buf), html_fan2, 
             percent2, disabled, percent2, latest_rpm2);
    httpd_resp_send_chunk(req, fan2_buf, strlen(fan2_buf));
    
    httpd_resp_send_chunk(req, html_script, strlen(html_script));
    
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t set_duty_get_handler(httpd_req_t *req) {
    if (auto_mode) {                    // if in auto mode
        httpd_resp_send(req, NULL, 0);  // ignore manual control
        return ESP_OK;
    }
    
    char query[64];
    size_t qlen = httpd_req_get_url_query_len(req) + 1;                         // buffer query params
    if (qlen && httpd_req_get_url_query_str(req, query, qlen) == ESP_OK) {
        char fan_val[8], duty_val[16];                                          // buffer fan # and duty
        if (httpd_query_key_value(query, "fan", fan_val, sizeof(fan_val)) == ESP_OK &&  // fan 1 or 2
            httpd_query_key_value(query, "duty", duty_val, sizeof(duty_val)) == ESP_OK) { // duty cycle value
            int fan = atoi(fan_val);                // convert fan string to int
            int duty = atoi(duty_val);              // duty string to int
            if (duty >= 0 && duty <= 1023) {        // validate duty value is within pwm range
                if (fan == 1) {
                    current_duty1 = duty;
                    ledc_set_duty(PWM_MODE, FAN1_PWM_CHANNEL, current_duty1);   // set new duty cycle
                    ledc_update_duty(PWM_MODE, FAN1_PWM_CHANNEL);               // apply to hardware
                } else if (fan == 2) {
                    current_duty2 = duty;
                    ledc_set_duty(PWM_MODE, FAN2_PWM_CHANNEL, current_duty2);
                    ledc_update_duty(PWM_MODE, FAN2_PWM_CHANNEL);
                }
            }
        }
    }
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t set_mode_get_handler(httpd_req_t *req) {
    char query[64];
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen && httpd_req_get_url_query_str(req, query, qlen) == ESP_OK) {
        char auto_val[8];
        if (httpd_query_key_value(query, "auto", auto_val, sizeof(auto_val)) == ESP_OK) {
            auto_mode = (atoi(auto_val) != 0);
            
            if (auto_mode) {
                uint32_t new_duty = calculate_duty_from_temp(current_temp_f);
                
                current_duty1 = new_duty;
                current_duty2 = new_duty;
                
                ledc_set_duty(PWM_MODE, FAN1_PWM_CHANNEL, current_duty1);
                ledc_set_duty(PWM_MODE, FAN2_PWM_CHANNEL, current_duty2);
                ledc_update_duty(PWM_MODE, FAN1_PWM_CHANNEL);
                ledc_update_duty(PWM_MODE, FAN2_PWM_CHANNEL);
            }
        }
    }
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// get data for the webapp
static esp_err_t data_get_handler(httpd_req_t *req) {
    char data_json[128];
    snprintf(data_json, sizeof(data_json), 
             "{\"rpm1\":%.1f,\"rpm2\":%.1f,\"temp\":%.1f,\"auto\":%d,\"duty1\":%lu,\"duty2\":%lu}", 
             latest_rpm1, latest_rpm2, current_temp_f, auto_mode ? 1 : 0, 
             (unsigned long)current_duty1, (unsigned long)current_duty2);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, data_json, strlen(data_json));
    return ESP_OK;
}

// webserver start from AP lab. some changes for each section
static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = slider_get_handler };
    httpd_uri_t uri_set = { .uri = "/set", .method = HTTP_GET, .handler = set_duty_get_handler };
    httpd_uri_t uri_mode = { .uri = "/mode", .method = HTTP_GET, .handler = set_mode_get_handler };
    httpd_uri_t uri_data = { .uri = "/data", .method = HTTP_GET, .handler = data_get_handler };
    
    httpd_start(&server, &config);
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_set);
    httpd_register_uri_handler(server, &uri_mode);
    httpd_register_uri_handler(server, &uri_data);
}

// WIFI event handler from AP lab
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// wifi connection stuff from AP lab
static void wifi_connect(void) {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE, pdFALSE, portMAX_DELAY);
    vEventGroupDelete(s_wifi_event_group);
}

// WIFI connection stuff from AP lab
void app_main(void) {
    nvs_flash_init();
    strcpy(ssid, "");
    strcpy(password, "");
    wifi_connect();
    gpio_reset_pin(DHT_PIN);
    init_pwm();
    init_pcnt(FAN1_PCNT_UNIT, FAN1_PCNT_CHANNEL, FAN1_TACH_GPIO);
    init_pcnt(FAN2_PCNT_UNIT, FAN2_PCNT_CHANNEL, FAN2_TACH_GPIO);
    xTaskCreate(rpm_task, "rpm_task", 4096, NULL, 5, NULL);             // create tasks for RPM 
    xTaskCreate(temp_control_task, "temp_task", 4096, NULL, 5, NULL);   // and temp
    start_webserver();
}
