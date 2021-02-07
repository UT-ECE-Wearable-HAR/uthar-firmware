#include "config.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_camera.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/time.h"
#include "time.h"

#include "mpu.h"

#define SPP_TAG "UTHAR_SPP"
#define SPP_SERVER_NAME "UTHAR_SERVER"
#define EXAMPLE_DEVICE_NAME "UTHAR_DEVICE"
#define SPP_SHOW_DATA 1
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE                                                          \
  SPP_SHOW_SPEED /*Choose show mode: show data or                              \
speed*/

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

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

    // XCLK 20MHz or 10MHz
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    //.pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    //.frame_size = FRAMESIZE_HQVGA,//QQVGA-UXGA Do not use sizes above QVGA
    // when not JPEG
    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,
    // FRAMESIZE_QCIF, // QQVGA-UXGA Do not use sizes above QVGA when not JPEG
    // FRAMESIZE_QQVGA FRAMESIZE_QVGA
    .jpeg_quality = 15, // 0-63 lower number means higher quality
    .fb_count =
        1 // if more than one, i2s runs in continuous mode. Use only with JPEG
};

static int pkt_count = 0;
static camera_fb_t *fb = NULL;
static volatile bool connected = false;
static volatile bool rcv_ready = false;
static uint32_t handle;
static const char *_STREAM_PART = "Content-Length: %u\r\n\r\n";
static uint8_t header_buf[128];

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
  case ESP_SPP_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
    esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
    break;
  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;
  case ESP_SPP_OPEN_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
    break;
  case ESP_SPP_CLOSE_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
    break;
  case ESP_SPP_START_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
    break;
  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
    break;
  case ESP_SPP_DATA_IND_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
             param->data_ind.len, param->data_ind.handle);
    esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
    if (strcmp((char *)(param->data_ind.data), "RCV_READY") == 0) {
      rcv_ready = true;
    }
    break;
  case ESP_SPP_CONG_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
    break;
  case ESP_SPP_WRITE_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
    gettimeofday(&time_old, NULL);
    connected = true;
    handle = param->data_ind.handle;
    break;
  default:
    break;
  }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
  case ESP_BT_GAP_AUTH_CMPL_EVT: {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(SPP_TAG, "authentication success: %s",
               param->auth_cmpl.device_name);
      esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
    } else {
      ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
               param->auth_cmpl.stat);
    }
    break;
  }
  case ESP_BT_GAP_PIN_REQ_EVT: {
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d",
             param->pin_req.min_16_digit);
    if (param->pin_req.min_16_digit) {
      ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
      esp_bt_pin_code_t pin_code = {0};
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
    } else {
      ESP_LOGI(SPP_TAG, "Input pin code: 1234");
      esp_bt_pin_code_t pin_code;
      pin_code[0] = '1';
      pin_code[1] = '2';
      pin_code[2] = '3';
      pin_code[3] = '4';
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
    break;
  }

#if (CONFIG_BT_SSP_ENABLED == true)
  case ESP_BT_GAP_CFM_REQ_EVT:
    ESP_LOGI(SPP_TAG,
             "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d",
             param->cfm_req.num_val);
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    break;
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d",
             param->key_notif.passkey);
    break;
  case ESP_BT_GAP_KEY_REQ_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
#endif

  default: {
    ESP_LOGI(SPP_TAG, "event: %d", event);
    break;
  }
  }
  return;
}

void mjpeg_stream(void *arg) {
  // Block for 500ms.
  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
  // Block for 10ms.
  const TickType_t delay = 1 / portTICK_PERIOD_MS;
  const TickType_t delay2 = 5 / portTICK_PERIOD_MS;
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
  mpu_init();
  ESP_LOGI("stream", "starting mjpeg stream");
  while (true) {
    if (connected) {
      ESP_LOGI("stream", "bluetooth connection established");
      while (true) {
        pkt_count = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
          ESP_LOGE("camera", "Camera capture failed");
        }
        while (mpu_read((uint8_t *)&header_buf)) {
        }
        ESP_LOGI("stream", "send header");
        size_t hlen = snprintf((char *)header_buf + PACKET_SIZE, 64,
                               _STREAM_PART, fb->len);
        esp_err_t ret;
        while (!rcv_ready) {
          vTaskDelay(delay);
        }
        rcv_ready = false;
        if ((ret = esp_spp_write(handle, hlen + PACKET_SIZE,
                                 (uint8_t *)&header_buf)) != ESP_OK) {
          ESP_LOGE(SPP_TAG, "%s content length send failed: %s\n", __func__,
                   esp_err_to_name(ret));
          return;
        }
        ESP_LOGI("stream", "send jpeg");
        if (esp_spp_write(handle, fb->len, fb->buf) != ESP_OK) {
          ESP_LOGE(SPP_TAG, "%s frame send failed: %s\n", __func__,
                   esp_err_to_name(ret));
        }
        esp_camera_fb_return(fb);
        ESP_LOGI("stream", "send complete");
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI("stream", "MJPG: %ums (%.1ffps)", (uint32_t)frame_time,
                 1000.0 / (uint32_t)frame_time);
        vTaskDelay(delay2);
      }
    }
    vTaskDelay(xDelay);
  }
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  if ((ret = esp_camera_init(&camera_config)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize camera failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_init()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

#if (CONFIG_BT_SSP_ENABLED == true)
  /* Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

  /*
   * Set default parameters for Legacy Pairing
   * Use variable pin, input pin code when pairing
   */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  esp_bt_pin_code_t pin_code;
  esp_bt_gap_set_pin(pin_type, 0, pin_code);

  vTaskDelay(500);
  xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  // xTaskCreate(&task_mpu6050, "task_mpu6050", 8192, NULL, 5, NULL);
  xTaskCreate(mjpeg_stream, "mjpeg_stream", 4096, NULL, 5, NULL);
}
