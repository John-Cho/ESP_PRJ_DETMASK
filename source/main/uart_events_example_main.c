/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#if defined(FEATURE_mPERS_CameraOV3660)
#include <nvs_flash.h>
#include "include/esp_camera.h"
#endif
#if defined(FEATURE_mPERS_WPS)
#include "tcpip_adapter.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "esp_http_server.h"
#endif

static const char *TAG = "M_DET_PJ";

#define FWversion "RSK01.001"
/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

camera_fb_t *g_pFb = NULL;
camera_fb_t c_pFb;

#define UART_NUM1        (UART_NUM_1)
#define UART1_RX_PIN     (15)
#define UART1_TX_PIN     (21)

typedef struct {
    uint8_t count;                          /* Scan AP number */
    uint8_t bssid[6];                     /**< MAC address of AP */
    uint8_t ssid[33];                     /**< SSID of AP */
    uint8_t primary;                      /**< channel of AP */
    wifi_second_chan_t second;            /**< secondary channel of AP */
    int8_t  rssi;                         /**< signal strength of AP */
    wifi_country_t country;               /**< country information of AP */
} wifi_ap_info_t;

/************************
   * GPIO Configuration
*************************/
#define GPIO_INPUT_CAMWAKE       26
#define GPIO_INPUT_WIFIWAKE      33
#define GPIO_OUTPUT_CAMEN        19
#define GPIO_INPUT_PIN_SEL       ((1ULL<<GPIO_INPUT_CAMWAKE) | (1ULL<<GPIO_INPUT_WIFIWAKE))
#define GPIO_OUTPUT_CAM_ENABLE   (1ULL<<GPIO_OUTPUT_CAMEN)
/************************
   * WIFI Configuration
*************************/
#define RANIX_SSID "RANIX_WIFI_3F"
#define RANIX_PWD "5845516012"
/************************
   * Definition for Queue type.
*************************/
#define Q_TYPE_GPIO     0x1000
#define Q_TYPE_WIFI     0x0100
#define Q_TYPE_HTTP     0x0010
#define Q_TYPE_CAM      0x0001
/************************
   * Definition for Camera Task
*************************/
#define CAM_EVT_START      (0x01 << 0)
#define CAM_EVT_STOP       (0x01 << 1)
#define CAM_EVT_OK         (0x01 << 2)
#define CAM_EVT_ERR        (0x01 << 3)
/************************
   * Definition for HTTP Protocol
*************************/
#define PART_BOUNDARY "frame"/*"123456789000000000000987654321"*/
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

/************************
   * Enumeration for Camera Task
*************************/
typedef enum STREAM_STEP_E
{
   eSTREAM_WAIT_FOR_EVT = 0,
   eSTREAM_CAMERA_INIT,
   eSTREAM_CAMERA_ON,
   eSTREAM_STOP,
   eSTREAM_ERROR,
} StreamStep_e;
/************************
   * Create Event or Queue valiable.
*************************/
EventGroupHandle_t g_CameraEvent;
QueueHandle_t g_MainQueue;

void app_send_queue(uint16_t type, uint16_t val)
{
   uint32_t data = ((uint32_t)type << 16) | (val);
   xQueueSend(g_MainQueue, &data, portMAX_DELAY);
}

void app_send_queue_isr(uint16_t type, uint16_t val)
{
   uint32_t data = ((uint32_t)type << 16) | (val);
   xQueueSendFromISR(g_MainQueue, &data, NULL);
}

#if defined(FEATURE_mPERS_CameraOV3660)
bool app_camera_start(void)
{
   esp_err_t err;
   
   // Enable pin is set to High.
   gpio_set_level(GPIO_OUTPUT_CAMEN, 1);
   vTaskDelay(100 / portTICK_PERIOD_MS);

   gpio_config_t conf;
   conf.mode = GPIO_MODE_INPUT;
   conf.pull_up_en = GPIO_PULLUP_ENABLE;
   conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
   conf.intr_type = GPIO_INTR_DISABLE;
   conf.pin_bit_mask = 1LL << 13;
   gpio_config(&conf);
   conf.pin_bit_mask = 1LL << 14;
   gpio_config(&conf);

   camera_config_t camera_config = {
            .ledc_channel = LEDC_CHANNEL_0,
            .ledc_timer = LEDC_TIMER_0,
            .pin_d0 = CONFIG_D0,
            .pin_d1 = CONFIG_D1,
            .pin_d2 = CONFIG_D2,
            .pin_d3 = CONFIG_D3,
            .pin_d4 = CONFIG_D4,
            .pin_d5 = CONFIG_D5,
            .pin_d6 = CONFIG_D6,
            .pin_d7 = CONFIG_D7,
            .pin_xclk = CONFIG_XCLK,
            .pin_pclk = CONFIG_PCLK,
            .pin_vsync = CONFIG_VSYNC,
            .pin_href = CONFIG_HREF,
            .pin_sscb_sda = CONFIG_SDA,
            .pin_sscb_scl = CONFIG_SCL,
            .pin_reset = CONFIG_RESET,
            .xclk_freq_hz = CONFIG_XCLK_FREQ,
            .pin_pwdn  = CONFIG_PIN_PWDN,
            .fb_count = 1,
            .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
            .frame_size = FRAMESIZE_QVGA,         //QQVGA-QXGA Do not use sizes above QVGA when not JPEG
            .jpeg_quality = 10,   //0-63 lower number means higher quality
   };

   err = esp_camera_init(&camera_config);
   if (err != ESP_OK)
   {
      gpio_set_level(GPIO_OUTPUT_CAMEN, 0);
      return false;
   }

   return true;
}
#endif

esp_err_t app_http_resp_handler(httpd_req_t *req)
{
   static bool init = false;
   camera_fb_t * fb = NULL;
   esp_err_t res = ESP_OK;
   size_t _jpg_buf_len = 0;
   uint8_t * _jpg_buf = NULL;
   char * part_buf[64];

   res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
   if(res != ESP_OK)
   {
      return res;
   }

   if(!init && app_camera_start())
   {
      init = true;
      ESP_LOGI(TAG, "eSTREAM_CAMERA_INIT Success");
   }
   else if(!init)
   {
      ESP_LOGE(TAG, "eSTREAM_CAMERA_INIT Failed");
      return res;
   }
   while(true)
   {
      fb = esp_camera_fb_get();
      if (!fb) 
      {
         res = ESP_FAIL;
      }
      else
      {
         if(fb->width > 100)
         {
            if(fb->format != PIXFORMAT_JPEG)
            {
               res = ESP_FAIL;
            }
            else
            {
               _jpg_buf_len = fb->len;
               _jpg_buf = fb->buf;
            }
         }
      }

      ESP_LOGI(TAG, "JPEG base %08x sz %d", (uint16_t)_jpg_buf, _jpg_buf_len);
      
      if(res == ESP_OK)
      {
         size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
         res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      
      if(res == ESP_OK)
      {
         res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      
      if(res == ESP_OK)
      {
         res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      
      if(fb)
      {
         esp_camera_fb_return(fb);
         fb = NULL;
         _jpg_buf = NULL;
      }

      if(res != ESP_OK)
      {
         break;
      }

      vTaskDelay(5 / portTICK_PERIOD_MS);
   }
   
   
   return res;
}

static void app_http_server_start(void)
{
   esp_err_t err;
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();

   httpd_uri_t index_uri = {
      .uri = "/jpeg",
      .method = HTTP_GET,
      .handler = app_http_resp_handler,
      .user_ctx = NULL
   };
      
   config.server_port = 80;
   
   if((err = httpd_start(&stream_httpd, &config)) == ESP_OK)
   {
      ESP_LOGI("HTTP", "HTTP Server Start!");
      
      httpd_register_uri_handler(stream_httpd, &index_uri);
   }
   else
   {
      ESP_LOGE("HTTP", "HTTP Server Error! %d", err);
   }
   
   return err;
}


/**
* @brief   wifi_scan_done_handler
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void wifi_scan_done_handler()
{
    uint16_t apCount = 0;
    uint8_t scan_countinfo_buf[49] = {0};
    //wifi_ap_info_t t_ap_list;   //wifi ap sorting
    uint8_t a, b;
    	
    ESP_LOGI(TAG, "%s", "wifi_scan_done_handler");
    
    esp_wifi_scan_get_ap_num(&apCount);
    if (apCount == 0 || apCount == 1) {
        ESP_LOGI(TAG, "%s", "Nothing AP found or Less than 2");
        return;
    }
    
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
    if (!ap_list) {
        ESP_LOGI(TAG, "%s", "malloc error, ap_list is NULL");
        return;
    }
    
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
    wifi_ap_info_t * wps_ap_list = (wifi_ap_info_t *)malloc(apCount * sizeof(wifi_ap_info_t));
    if (!wps_ap_list) {
        if (ap_list) {
            free(ap_list);
        }
        ESP_LOGI(TAG, "%s", "malloc error, wps_ap_list is NULL");
        return;
    }
    
    for ( a = 0; a < apCount; ++a)
    {
        char *pssid = (char *)wps_ap_list[a].ssid;
        wps_ap_list[a].count = apCount;
        wps_ap_list[a].rssi = ap_list[a].rssi;
        wps_ap_list[a].primary = ap_list[a].primary;
        memcpy(wps_ap_list[a].bssid, ap_list[a].bssid, sizeof(ap_list[a].bssid));
        memcpy(wps_ap_list[a].ssid, ap_list[a].ssid, sizeof(ap_list[a].ssid));
        ESP_LOGI(TAG, "AP INFO ssid %s Ch %d Signal %d", pssid, wps_ap_list[a].primary, wps_ap_list[a].rssi);
        ESP_LOGI(TAG, "AP INFO Mac %x:%x:%x:%x:%x:%x", wps_ap_list[a].bssid[0], wps_ap_list[a].bssid[1], wps_ap_list[a].bssid[2],
        	                                                                                                                 wps_ap_list[a].bssid[3], wps_ap_list[a].bssid[4], wps_ap_list[a].bssid[5]);
    }
    
    esp_wifi_scan_stop();
    
    free(ap_list);
    free(wps_ap_list);
    
    return ;
}

/**
* @brief   app_wifi_handler
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void app_wifi_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
   switch(event_id)
   {
      case WIFI_EVENT_STA_START:
         ESP_LOGI(TAG, "WIFI_EVENT_STA_START Event %d", event_id);
         wifi_scan_config_t scanConf = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = true
         };

         ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
         break;
         
      case WIFI_EVENT_SCAN_DONE:
         ESP_LOGI(TAG, "WIFI_EVENT_SCAN_DONE Event %d", event_id);
         wifi_scan_done_handler();
         ESP_LOGI(TAG, "Try connect %d", esp_wifi_connect());
         break;

      case WIFI_EVENT_STA_CONNECTED:
         ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED Event %d", event_id);
         app_http_server_start();
         break;

      default:
         ESP_LOGI(TAG, "Unknown Event %d", event_id);
         break;
   }
}

/**
* @brief   app_wifi_init
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void app_wifi_init()
{
   tcpip_adapter_init();
   ESP_ERROR_CHECK( esp_event_loop_create_default() );

   /* Setup for WIFI Configuration */
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

   /* Registered Event Handler */
   ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &app_wifi_handler, NULL) );
   ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &app_wifi_handler, NULL) );
   ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &app_wifi_handler, NULL) );

   wifi_config_t wifi_config = {
      .sta = {
         .ssid = RANIX_SSID,
         .password = RANIX_PWD,
         .scan_method = WIFI_ALL_CHANNEL_SCAN,
         .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
         .threshold.rssi = -127,
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,
      },
   };
   ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
   
   /* WIFI Station mode is started. */
   ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
   ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
   ESP_ERROR_CHECK( esp_wifi_start());
}

/**
* @brief   app_task_streaming
* @details 
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void app_task_camera(void *pvParameters)
{
   static bool IsCamerainit = false;
   EventBits_t uxBits = 0;
   StreamStep_e step = eSTREAM_WAIT_FOR_EVT;
   
   while(true)
   {
      switch(step)
      {
         case eSTREAM_WAIT_FOR_EVT:
            uxBits = xEventGroupWaitBits(g_CameraEvent, (CAM_EVT_START | CAM_EVT_STOP), pdTRUE, pdFALSE, portMAX_DELAY);
            if(uxBits & CAM_EVT_START)
            {
               ESP_LOGI(TAG, "Receive Event CAM Start");
               step = eSTREAM_CAMERA_INIT;
            }
            else if(uxBits & CAM_EVT_STOP)
            {
               ESP_LOGI(TAG, "Receive Event CAM Stop");
               step = eSTREAM_STOP;
            }
            break;

         case eSTREAM_CAMERA_INIT:
            if(IsCamerainit || app_camera_start())
            {
               IsCamerainit = true;
               ESP_LOGI(TAG, "eSTREAM_CAMERA_INIT Success");
               step = eSTREAM_WAIT_FOR_EVT;
            }
            else
            {
               ESP_LOGE(TAG, "eSTREAM_CAMERA_INIT Failed");
               step = eSTREAM_ERROR;
            }
            break;

         case eSTREAM_STOP:
            if(esp_camera_deinit())
            {
               IsCamerainit = false;
               ESP_LOGI(TAG, "eSTREAM_STOP Success");
               step = eSTREAM_WAIT_FOR_EVT;
            }
            else
            {
               ESP_LOGE(TAG, "eSTREAM_STOP Failed");
               step = eSTREAM_ERROR;
            }
            break;
            
         case eSTREAM_ERROR:
         default:
            IsCamerainit = false;
            step = eSTREAM_WAIT_FOR_EVT;
            break;
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
   }

   vTaskDelete(NULL);
}

/**
* @brief   app_task_main
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void app_task_main(void *pvParameters)
{
   uint32_t recvQ;

   app_wifi_init();
   
   while(true)
   {
      if(xQueueReceive(g_MainQueue, &recvQ, (10/portTICK_PERIOD_MS)))
      {
         ESP_LOGI(TAG, "recv %08x", recvQ);
      }
      else
      {
      }
   }

   vTaskDelete(NULL);
}

/**
* @brief   app_task_uart
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void app_task_uart(void *pvParameters)
{
   while(true)
   {
      vTaskDelay(10 / portTICK_PERIOD_MS);
   }

   vTaskDelete(NULL);
}

/**
* @brief   nrf_uart_init
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
void nrf_uart_init()
{
//	   uart_config_t uart_config = {
//	#if defined(FEATURE_mPERS_CAM_Debug)
//	   //aiden_test 0316 old camera
//	#if 0
//	     .baud_rate = 115200,
//	#else
//	     .baud_rate = 57600,
//	#endif
//	#else
//	     .baud_rate = 115200,
//	#endif
//	     .data_bits = UART_DATA_8_BITS,
//	     .parity = UART_PARITY_DISABLE,
//	     .stop_bits = UART_STOP_BITS_1,
//	     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//	   };
//	
//	   //Configuration Uart paramater
//	   ESP_ERROR_CHECK(uart_param_config(UART_NUM1, &uart_config));   
//	
//	   //Set UART pins (using UART0 default pins ie no changes.)
//	   ESP_ERROR_CHECK(uart_set_pin(UART_NUM1, 
//	                                UART1_TX_PIN, 
//	                                UART1_RX_PIN, 
//	                                UART_PIN_NO_CHANGE, 
//	                                UART_PIN_NO_CHANGE));
//	   
//	   //Install UART driver, and get the queue.
//	   ESP_ERROR_CHECK(uart_driver_install(UART_NUM1, BUF_SIZE * 2, 0, 0, NULL, 0));
}

/**
* @brief   gpio_isr_handler
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
static void gpio_isr_handler(void* arg)
{
   uint32_t gpio_num = (uint32_t) arg;
   app_send_queue_isr(Q_TYPE_GPIO, (uint16_t)gpio_num);
}

#define ESP_INTR_FLAG_DEFAULT 0
/**
* @brief   nrf_gpio_init
* @details
* @date    2022/3/18
* @param
*
* @return
*
* @see
*
*/
void nrf_gpio_init()
{
   gpio_config_t io_conf;
   //interrupt of rising edge
   io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
   //bit mask of the pins, use GPIO4/5 here
   io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
   //set as input mode    
   io_conf.mode = GPIO_MODE_INPUT;
   //enable pull-up mode
   io_conf.pull_up_en = 1;
   gpio_config(&io_conf);
   gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

   gpio_set_level(GPIO_INPUT_CAMWAKE, 0);
   gpio_set_level(GPIO_INPUT_WIFIWAKE, 0);
   
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(GPIO_INPUT_CAMWAKE, gpio_isr_handler, (void*) GPIO_INPUT_CAMWAKE);
   gpio_isr_handler_add(GPIO_INPUT_WIFIWAKE, gpio_isr_handler, (void*) GPIO_INPUT_WIFIWAKE);

#if defined(FEATURE_mPERS_CameraOV3660)
   gpio_config_t io_conf_19;
   /// configure the PCM output pins
   //disable interrupt
   io_conf_19.intr_type = GPIO_PIN_INTR_DISABLE;
   //set as output mode
   io_conf_19.mode = GPIO_MODE_OUTPUT;
   //bit mask of the pins that you want to set,e.g.GPIO19
   io_conf_19.pin_bit_mask = GPIO_OUTPUT_CAM_ENABLE;
   //disable pull-down mode
   io_conf_19.pull_down_en = GPIO_PULLDOWN_ENABLE;
   //disable pull-up mode
   io_conf_19.pull_up_en = 0;
   //configure GPIO with the given settings
   gpio_config(&io_conf_19);
   
   // Enable the camera module.
   gpio_set_level(19, 1);
#endif
}

/**
* @brief   app_main
* @details Entry Function
* @date    2022/3/17
* @param
*
* @return
*
* @see
*
*/
void app_main()
{   
   /* Set UART log level */
   esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

   /* Configure parameters of an UART driver */
   nrf_uart_init();

   /* Configure the gpio service */
   nrf_gpio_init();

   /* Create event group for streaming */
   g_CameraEvent = xEventGroupCreate();
   if(g_CameraEvent == NULL)
   {
      ESP_LOGI(TAG,"ERROR] Create \"g_SystemEvent\"");
      return;
   }

   /* Create Queue for external event. */
   g_MainQueue = xQueueCreate(10, sizeof(uint32_t));
   if(g_MainQueue == NULL)
   {
      ESP_LOGI(TAG,"ERROR] Create \"g_MainQueue\"");
      return;
   }
   
   /* Create main task */
   xTaskCreate(app_task_main, "app_task_main", 1024*4, NULL, 10, NULL);

   /* Create camera task */
   xTaskCreate(app_task_camera, "app_task_streaming", 1024*4, NULL, 10, NULL);

   /* Create uart task. */
   //xTaskCreate(app_task_uart, "app_task_uart", 1024*4, NULL, 10, NULL);
}
