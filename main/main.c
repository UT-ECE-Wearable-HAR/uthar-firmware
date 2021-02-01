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

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /*   ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE)); */

  /*   esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
   */

  /*   if ((ret = esp_camera_init(&camera_config)) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s initialize camera failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
   */
  /*     ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_bluedroid_init()) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_bluedroid_enable()) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /*   if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) { */
  /*     ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, */
  /*              esp_err_to_name(ret)); */
  /*     return; */
  /*   } */

  /* #if (CONFIG_BT_SSP_ENABLED == true) */
  /*   /\* Set default parameters for Secure Simple Pairing *\/ */
  /*   esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE; */
  /*   esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO; */
  /*   esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t)); */
  /* #endif */

  /*   /\* */
  /*    * Set default parameters for Legacy Pairing */
  /*    * Use variable pin, input pin code when pairing */
  /*    *\/ */
  /*   esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE; */
  /*   esp_bt_pin_code_t pin_code; */
  /*   esp_bt_gap_set_pin(pin_type, 0, pin_code); */

  xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  xTaskCreate(&task_mpu6050, "task_mpu6050", 8192, NULL, 5, NULL);
  // xTaskCreate(mjpeg_stream, "mjpeg_stream", 4096, NULL, 5, NULL);
}
