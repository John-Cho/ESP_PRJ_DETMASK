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
#endif

static const char *TAG = "uart_events";

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

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

#if defined(FEATURE_mPERS_UART)
camera_fb_t *g_pFb = NULL;
#if defined(FEATURE_mPERS_CAM_Debug)
camera_fb_t c_pFb;
#endif
#define UART_NUM1        (UART_NUM_1)
#define UART1_RX_PIN     (15)
#define UART1_TX_PIN     (21)

#define	UART_STX			0x02
#define	UART_ETX			0x03

#define	PICTURE_INFO_REQ	0x10	
#define	PICTURE_INFO_RES	0x11	
#define	PICTURE_TRANS_START	0x12
#define	PART_TRANS_RES		0x21
//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
#define	CAMERA_SENSOR_READY		0x30
#endif
#define	RESULT_OK			0x0

#if defined(FEATURE_mPERS_WPS)
static EventGroupHandle_t wifi_event_group;

#define	WPS_START_REQ	0x13	

typedef struct {
    uint8_t count;                          /* Scan AP number */
    uint8_t bssid[6];                     /**< MAC address of AP */
    uint8_t ssid[33];                     /**< SSID of AP */
    uint8_t primary;                      /**< channel of AP */
    wifi_second_chan_t second;            /**< secondary channel of AP */
    int8_t  rssi;                         /**< signal strength of AP */
    wifi_country_t country;               /**< country information of AP */
} wifi_ap_info_t;

uint8_t wifi_apinfo_send[5][8];
#endif

#define	P_UART_HEADER		0
#define	P_UART_OPCODE		1
#define	P_UART_LENGTH		2		// 2byte
#define	P_UART_DATA			4

typedef struct {
	uint16_t count;
	uint8_t buf[BUF_SIZE];
} UART_FIFO_BUF;

UART_FIFO_BUF uartFifoBuf;
uint8_t g_quality;
uint16_t g_frameSize;

#define GPIO_INPUT_CAMWAKE     26
#if defined(FEATURE_mPERS_WPS)
#define GPIO_INPUT_WIFIWAKE     33
#endif
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_CAMWAKE) | (1ULL<<GPIO_INPUT_WIFIWAKE))

#if defined(FEATURE_mPERS_CAM_Debug)
#define GPIO_OUTPUT_IO_19     19
#define GPIO_OUTPUT_PIN19_SEL  (1ULL<<GPIO_OUTPUT_IO_19)

esp_timer_handle_t periodic_timer;
#define Timer_Default_Boot                         1000000*10   //10s
bool   camera_gpio_set = 0;
uint8_t   camera_boot_count = 0;
bool   camera_boot_good = 0;
uint8_t   esp_boot_complete = 0;

//aiden_test 0302
bool is_uart_tx = false;
bool camera_boot_start = false;

uint8_t camera_quality_set = 0;
uint8_t check_model = 0;   // 1: ov3660, 2: nt99141
#endif

#if defined(FEATURE_mPERS_WPS)
bool   wifi_gpio_set = 0;
#endif
#endif

//aiden_test 1126
#if 0
uint8_t testbuf[30000];
//uint8_t testbuf[130000];
#endif

#if defined(FEATURE_mPERS_CameraOV3660)
void camera_booton (void)
{

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

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
#if defined(FEATURE_mPERS_CAM_Debug)
        .fb_count = 1,
        .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_QVGA,         //QQVGA-QXGA Do not use sizes above QVGA when not JPEG
        .jpeg_quality = camera_quality_set,   //0-63 lower number means higher quality
#else
        .fb_count = 1,
        .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_VGA, //QQVGA-QXGA Do not use sizes above QVGA when not JPEG
        .jpeg_quality = 10,                           //0-63 lower number means higher quality
#endif
    };
    
    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        camera_boot_good = 0;
        gpio_set_level(19, 0);
        return;
    }
    camera_boot_good = 1;
}
#endif

#if defined(FEATURE_mPERS_UART)
typedef enum
{
   State_Boot,
   State_CamBootWait,
  	State_Idle,
} State_Type;

State_Type  Mode_State = State_Boot;

void uart_rx_bypass_set (bool bypass)
{
    is_uart_tx = bypass;
}

bool uart_rx_bypass_get (void)
{
    return is_uart_tx;
}

void uartTx_operation (uint8_t *dataBuf, size_t dataLength)
{
	uart_write_bytes (UART_NUM_1, (const char *)dataBuf, dataLength);
}

//aiden_test 0302
void esp_start (void)
{
	 vTaskDelay (500 / portTICK_RATE_MS);

   uart_rx_bypass_set(true);

	uint8_t headerTail_buf[4] = {0};

	headerTail_buf[0] = 0xff;
	headerTail_buf[1] = 0xee;
   headerTail_buf[2] = 0xdd;
	headerTail_buf[3] = 0xcc;
   
   ESP_LOGI(TAG, "esp_start Send to nRF : %d\n", headerTail_buf[1]);

   uartTx_operation (headerTail_buf, 4);

  size_t  t_heap = heap_caps_get_free_size (MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "heap free => %d\n", t_heap);

   uart_rx_bypass_set(false);

	vTaskDelay (100 / portTICK_RATE_MS);
}

void camera_start (void)
{
	 vTaskDelay (500 / portTICK_RATE_MS);

   uart_rx_bypass_set(true);

	uint8_t headerTail_buf[8] = {0};

	headerTail_buf[0] = UART_STX;
	headerTail_buf[1] = PICTURE_INFO_RES;
	if( check_model == 1 )
      headerTail_buf[2] = 0xaa;
	else if( check_model == 2 )
      headerTail_buf[2] = 0xbb;
	else
	   headerTail_buf[2] = 0x00;
	headerTail_buf[3] = 0xff;
   headerTail_buf[5] = 0xff;
   headerTail_buf[6] = 0xff;
   headerTail_buf[7] = 0xff;
   
   ESP_LOGI(TAG, "camera_start Send to nRF : %d\n", headerTail_buf[2]);

   uartTx_operation (headerTail_buf, 8);

  size_t  t_heap = heap_caps_get_free_size (MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "heap free => %d\n", t_heap);

  uart_rx_bypass_set(false);

 vTaskDelay (100 / portTICK_RATE_MS);
}

void camera_capture_trans (uint16_t transFrame)
{

#if defined(FEATURE_mPERS_CAM_Debug)
   uint16_t frameCount = c_pFb.len / g_frameSize;
   uint16_t endFrameCount = c_pFb.len % g_frameSize;
#else
	uint16_t frameCount = g_pFb->len / g_frameSize;
	uint16_t endFrameCount = g_pFb->len % g_frameSize;
#endif
	uint16_t dataLength = 0;
	uint8_t headerTail_buf[7] = {0};
	uint16_t i = 0;

	printf ("length : %d\n", c_pFb.len);
  	printf ("frame count : %d\n", frameCount);
	printf ("endFrameCount : %d\n", endFrameCount);

   vTaskDelay (500 / portTICK_RATE_MS);

	headerTail_buf[0] = UART_STX;
	headerTail_buf[1] = PICTURE_TRANS_START;

	if (transFrame > frameCount) {
		dataLength = endFrameCount + 3;				// length

		headerTail_buf[5] = (uint8_t)endFrameCount;
		headerTail_buf[6] = (uint8_t)(endFrameCount >> 8);;
	}
	else {
		dataLength = g_frameSize + 3;				// length

		headerTail_buf[5] = (uint8_t)g_frameSize;					// length
		headerTail_buf[6] = (uint8_t)(g_frameSize >> 8);			// length
	}

	headerTail_buf[2] = (uint8_t)dataLength;				// length
	headerTail_buf[3] = (uint8_t)(dataLength >> 8);			// length
	headerTail_buf[4] = transFrame;

	//for (i = 0; i < 7; i++)
	//	headerTail_buf[7] ^= headerTail_buf[i];		// checksum xor

#if defined(FEATURE_mPERS_CAM_Debug)
	//for (i = 0; i < g_frameSize; i++)
	//	headerTail_buf[7] ^= *(c_pFb.buf + ((transFrame - 1) * g_frameSize) + i);		// checksum xor
#else
	//for (i = 0; i < g_frameSize; i++)
	//	headerTail_buf[7] ^= *(g_pFb->buf + ((transFrame - 1) * g_frameSize) + i);		// checksum xor
#endif

	//headerTail_buf[8] = UART_ETX;

   uart_rx_bypass_set(true);

	uartTx_operation (headerTail_buf, 7);
  vTaskDelay(1);

#if defined(FEATURE_mPERS_CAM_Debug)
    for( uint8_t aa = 0; aa < frameCount+1; aa++ )
    {
        if (transFrame < frameCount+1)
            uartTx_operation ((uint8_t *)c_pFb.buf + ((transFrame - 1) * g_frameSize), g_frameSize);
        else
            uartTx_operation ((uint8_t *)c_pFb.buf + ((transFrame - 1) * g_frameSize), endFrameCount);

//aiden_test 0302 camera
#if 1
        if( uart_wait_tx_done(UART_NUM1, (portTickType)(100 / portTICK_RATE_MS)) == ESP_OK )
            transFrame++;

        //vTaskDelay(1);
#else
        transFrame++;
#endif
    }

    uart_rx_bypass_set(false);

    //aiden_test 1126
    #if 0
    {
        char zz0[2+1], zz1[2+1], zz2[2+1], zz3[2+1], zz4[2+1], zz5[2+1], zz6[2+1], zz7[2+1], zz8[2+1], zz9[2+1];
        
        ESP_LOGI(TAG, "%s, %d, %d, %d, %d, %d", "TEST", frameCount, transFrame, g_frameSize, c_pFb.len, endFrameCount );
        for( uint16_t tt = 0; tt < c_pFb.len; tt++ )
            testbuf[tt] = *(c_pFb.buf + tt);

        for( uint16_t zz = 0; zz < c_pFb.len; zz++ )
        {
            vTaskDelay(1);
            
            if( testbuf[zz+0] < 0x10 )
                sprintf( zz0, "0%x", testbuf[zz+0]);
            else
                sprintf( zz0, "%x", testbuf[zz+0]);

            if( testbuf[zz+1] < 0x10 )
                sprintf( zz1, "0%x", testbuf[zz+1]);
            else
                sprintf( zz1, "%x", testbuf[zz+1]);

            if( testbuf[zz+2] < 0x10 )
                sprintf( zz2, "0%x", testbuf[zz+2]);
            else
                sprintf( zz2, "%x", testbuf[zz+2]);

            if( testbuf[zz+3] < 0x10 )
                sprintf( zz3, "0%x", testbuf[zz+3]);
            else
                sprintf( zz3, "%x", testbuf[zz+3]);

            if( testbuf[zz+4] < 0x10 )
                sprintf( zz4, "0%x", testbuf[zz+4]);
            else
                sprintf( zz4, "%x", testbuf[zz+4]);

            if( testbuf[zz+5] < 0x10 )
                sprintf( zz5, "0%x", testbuf[zz+5]);
            else
                sprintf( zz5, "%x", testbuf[zz+5]);

            if( testbuf[zz+6] < 0x10 )
                sprintf( zz6, "0%x", testbuf[zz+6]);
            else
                sprintf( zz6, "%x", testbuf[zz+6]);

            if( testbuf[zz+7] < 0x10 )
                sprintf( zz7, "0%x", testbuf[zz+7]);
            else
                sprintf( zz7, "%x", testbuf[zz+7]);

            if( testbuf[zz+8] < 0x10 )
                sprintf( zz8, "0%x", testbuf[zz+8]);
            else
                sprintf( zz8, "%x", testbuf[zz+8]);

            if( testbuf[zz+9] < 0x10 )
                sprintf( zz9, "0%x", testbuf[zz+9]);
            else
                sprintf( zz9, "%x", testbuf[zz+9]);

            ESP_LOGI(TAG, "%s %s %s %s %s %s %s %s %s %s ", zz0, zz1, zz2, zz3, zz4, zz5, zz6, zz7, zz8, zz9);        
            //ESP_LOGI(TAG, "%s%x%s%x%s%x%s%x%s%x%s%x%s%x%s%x%s%x%s%x ", 
            //testbuf[zz],testbuf[zz+1],testbuf[zz+2],testbuf[zz+3],testbuf[zz+4],testbuf[zz+5],testbuf[zz+6],testbuf[zz+7],testbuf[zz+8],testbuf[zz+9]);
            zz = zz+9;
        }
    }
#endif

#else
	if (transFrame < frameCount)
		uartTx_operation ((uint8_t *)g_pFb->buf + ((transFrame - 1) * g_frameSize), g_frameSize);
	else
		uartTx_operation ((uint8_t *)g_pFb->buf + ((transFrame - 1) * g_frameSize), endFrameCount);
#endif

	uartTx_operation (&headerTail_buf[7], 2);
	
	printf ("heap free => %d\n", heap_caps_get_free_size (MALLOC_CAP_8BIT));

   vTaskDelay (100 / portTICK_RATE_MS);

#if defined(FEATURE_mPERS_CAM_Debug)
	esp_restart();
#endif
	 
}

void camera_capture_start (void)
{
    camera_fb_t *fb = NULL;

    uart_rx_bypass_set(true);

    sensor_t * s = esp_camera_sensor_get();
#if defined(FEATURE_mPERS_CAM_Debug)
  s->set_vflip(s, 0);
  if( check_model == 1 )
      s->set_hmirror(s, 1);

  //aiden_test 4 camera
  if( check_model == 2 )
  {
      s->set_brightness(s, -2);   //-3
  }
#else
	s->set_vflip(s, 1);
#endif
	s->set_quality (s, g_quality);

	vTaskDelay (500 / portTICK_RATE_MS);

   fb = esp_camera_fb_get();
    
	esp_camera_fb_return(fb);

   ESP_LOGI(TAG, "frame size : %d\n", g_frameSize);
   ESP_LOGI(TAG, "quality : %d\n", g_quality);

	uint16_t frameCount = fb->len / g_frameSize;
	uint16_t endFrameCount = fb->len % g_frameSize;

   ESP_LOGI(TAG, "length : %d\n", fb->len);
   ESP_LOGI(TAG, "frame count : %d\n", frameCount);
   ESP_LOGI(TAG, "endFrameCount : %d\n", endFrameCount);

#if defined(FEATURE_mPERS_CAM_Debug)
  memcpy(&c_pFb, fb, sizeof(camera_fb_t));
#else
  g_pFb = fb;
#endif

	uint8_t headerTail_buf[9] = {0};
	//uint16_t i = 0;
#if defined(FEATURE_mPERS_CAM_Debug)    //aiden_test 1126
   c_pFb.len = c_pFb.len + 10;
#endif	
	headerTail_buf[0] = UART_STX;
	headerTail_buf[1] = PICTURE_INFO_RES;
	headerTail_buf[2] = (uint8_t)(c_pFb.len >> 8 );  // raw data length
	headerTail_buf[3] = (uint8_t)(c_pFb.len );  // raw data length
	headerTail_buf[4] = frameCount;			                        // frame count
	headerTail_buf[5] = endFrameCount;                    	// end frame count
	if( check_model == 1 )
      headerTail_buf[6] = 0xaa;
	else
	   headerTail_buf[6] = 0xbb;
	headerTail_buf[7] = 0xff;
   headerTail_buf[8] = 0;

   ESP_LOGI(TAG, "send to nRF : %d,%d,%d\n", headerTail_buf[2], headerTail_buf[3], headerTail_buf[6]);

#if 1
   uartTx_operation (headerTail_buf, 9);
#else
	if (endFrameCount > 0)
		headerTail_buf[6] += 1;

	for (i = 0; i < fb->len; i++)
		headerTail_buf[7] ^= (uint8_t)*(fb->buf + i);		// picture checksum xor

	for (i = 0; i < 8; i++)
		headerTail_buf[8] ^= headerTail_buf[i];

	headerTail_buf[9] = UART_ETX;

	uartTx_operation (headerTail_buf, 10);
#endif

  size_t  t_heap = heap_caps_get_free_size (MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "heap free => %d\n", t_heap);

  uart_rx_bypass_set(false);

	vTaskDelay (100 / portTICK_RATE_MS);
}

#if defined(FEATURE_mPERS_WPS)
static void wifi_scan_done_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    uint16_t apCount = 0;
    uint8_t scan_countinfo_buf[49] = {0};
    //wifi_ap_info_t t_ap_list;   //wifi ap sorting
    uint8_t a, b;
    	
    ESP_LOGI(TAG, "%s", "wifi_scan_done_handler");
    
    esp_wifi_scan_get_ap_num(&apCount);
    if (apCount == 0 || apCount == 1) {
         scan_countinfo_buf[0] = 0x00;
         scan_countinfo_buf[1] = 0x99;
         scan_countinfo_buf[2] = apCount;
         scan_countinfo_buf[3] = 0x00;
         
         uartTx_operation (scan_countinfo_buf, 4);
    	
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
        wps_ap_list[a].count = apCount;
        wps_ap_list[a].rssi = ap_list[a].rssi;
        wps_ap_list[a].primary = ap_list[a].primary;
        memcpy(wps_ap_list[a].bssid, ap_list[a].bssid, sizeof(ap_list[a].bssid));
        //memcpy(wps_ap_list[a].ssid, ap_list[a].ssid, sizeof(ap_list[a].ssid));
        ESP_LOGI(TAG, "AP INFO Ch %d Signal %d", wps_ap_list[a].primary, wps_ap_list[a].rssi);
        ESP_LOGI(TAG, "AP INFO Mac %x:%x:%x:%x:%x:%x", wps_ap_list[a].bssid[0], wps_ap_list[a].bssid[1], wps_ap_list[a].bssid[2],
        	                                                                                                                 wps_ap_list[a].bssid[3], wps_ap_list[a].bssid[4], wps_ap_list[a].bssid[5]);
    }
    esp_wifi_scan_stop();

    #if 0  //wifi ap sorting
    for(uint8_t m = 0 ; m < apCount-1 ; m ++) 
    {
        for(uint8_t n = m+1 ; n < apCount ; n ++) 
        {
            if(wps_ap_list[m].rssi < wps_ap_list[n].rssi) 
            {
                t_ap_list.rssi = wps_ap_list[n].rssi;
                t_ap_list.primary = wps_ap_list[n].primary;
                memcpy(t_ap_list.bssid, wps_ap_list[n].bssid, sizeof(wps_ap_list[n].bssid));
                
                wps_ap_list[n].rssi = wps_ap_list[m].rssi;
                wps_ap_list[n].primary = wps_ap_list[m].primary;
                memcpy(wps_ap_list[n].bssid, wps_ap_list[m].bssid, sizeof(wps_ap_list[m].bssid));
                
                wps_ap_list[m].rssi = t_ap_list.rssi;
                wps_ap_list[m].primary = t_ap_list.primary;
                memcpy(wps_ap_list[m].bssid, t_ap_list.bssid, sizeof(t_ap_list.bssid));
            }
        }
    }

    for (int i = 0; i < apCount; ++i)
    {
        ESP_LOGI(TAG, "AP INFO Ch %d Signal %d", wps_ap_list[i].primary, wps_ap_list[i].rssi);
        ESP_LOGI(TAG, "AP INFO Mac %x:%x:%x:%x:%x:%x", wps_ap_list[i].bssid[0], wps_ap_list[i].bssid[1], wps_ap_list[i].bssid[2],
        	                                                                                                                 wps_ap_list[i].bssid[3], wps_ap_list[i].bssid[4], wps_ap_list[i].bssid[5]);
    }
    #endif
    
    scan_countinfo_buf[0] = 0x00;   // Start byte
    scan_countinfo_buf[1] = 0x99;   // Start check
    if( apCount > 5 )                                 // AP count
        scan_countinfo_buf[2] = 5;
    else
        scan_countinfo_buf[2] = apCount;
    scan_countinfo_buf[3] = scan_countinfo_buf[2]*9+4;   // AP length
    for( a = 0, b = 0; a < apCount; a++)   // channel,rssi sign,rssi,mac(6byte)
    {
        scan_countinfo_buf[4+1*b] = wps_ap_list[a].primary;

        if( wps_ap_list[a].rssi <= 0 )
        {
            scan_countinfo_buf[4+1*b+1] = 0;
            scan_countinfo_buf[4+1*b+2] = ~wps_ap_list[a].rssi;
        }
        else
        {
            scan_countinfo_buf[4+1*b+1] = 1;
            scan_countinfo_buf[4+1*b+2] = wps_ap_list[a].rssi;
        }
        
        memcpy( &scan_countinfo_buf[4+1*b+3], wps_ap_list[a].bssid, sizeof(wps_ap_list[a].bssid) );
        b = b + 9;
    }    

    uartTx_operation (scan_countinfo_buf, scan_countinfo_buf[3]);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    free(ap_list);
    free(wps_ap_list);
    
    return ;
}

static void scan_initialise_wifi(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &wifi_scan_done_handler, NULL) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    wifi_scan_config_t scanConf = {
       .ssid = NULL,
       .bssid = NULL,
       .channel = 0,
       .show_hidden = true
    };

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
}

#endif

void uartRxData_frameParsingCam (UART_FIFO_BUF *uartRxBuf)
{
	uint8_t *dataBuf = (uint8_t *)malloc (BUF_SIZE);

	uint16_t i = 0;
	uint8_t checkSum = 0;
	uint32_t uartDataLen = 0;
	uint16_t dataBufCnt = 0;
	uint16_t backupCount = 0;

	memcpy ((char *)dataBuf, uartRxBuf->buf, uartRxBuf->count);
	backupCount = uartRxBuf->count;

	printf ("<<<<<<<<<< Camera Command <<<<<<<<<<<\n");
	for (i = 0; i < uartRxBuf->count; i++) 
	{
		printf ("%02x ", *(dataBuf + i));
	}

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
	if( *(dataBuf + 0) == 0x02 )
		{
				switch ( *(dataBuf + 1) )
					{
					case PICTURE_INFO_REQ:
						printf ("SHOT AND PICTURE INFO REQUEST\n");

						g_frameSize = 256;
#if defined(FEATURE_mPERS_CAM_Debug)    //aiden_test 1126
                g_quality = camera_quality_set;
#else
						g_quality = *(dataBuf + dataBufCnt + P_UART_DATA + 2);
#endif
						camera_capture_start ();
						break;
						
					case PICTURE_TRANS_START:
						printf ("PICTURE TRANSMIT START\n");
							camera_capture_trans (1);
						break;

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
					case CAMERA_SENSOR_READY:
						printf ("CAMERA SENSOR READY\n");
						camera_boot_start = true;
						break;
#endif

					default:
						break;
				}
}
#else
	do {
		if (*(dataBuf + dataBufCnt + P_UART_HEADER) == UART_STX) {
			uartDataLen = (uint32_t)*(dataBuf + dataBufCnt + P_UART_LENGTH);
			uartDataLen |= (uint32_t)((*(dataBuf + dataBufCnt + P_UART_LENGTH + 1)) << 8);
			
			checkSum = 0;
			for (i = 0; i < (uartDataLen + 4); i++) {		// stx ~ data end
				checkSum ^= *(dataBuf + dataBufCnt + i);
			}

			if (checkSum == *(dataBuf + dataBufCnt + uartDataLen + 4) && 
					UART_ETX == *(dataBuf + dataBufCnt + uartDataLen + 5)) {

				if (uartRxBuf->count < uartDataLen + 4) {
					uartRxBuf->count = backupCount;
					break;
				}
			
				switch (*(dataBuf + dataBufCnt + P_UART_OPCODE)) {
					case PICTURE_INFO_REQ:
						printf ("SHOT AND PICTURE INFO REQUEST\n");

						g_frameSize = (uint16_t)*(dataBuf + dataBufCnt + P_UART_DATA);
						g_frameSize |= (uint16_t)(*(dataBuf + dataBufCnt + P_UART_DATA + 1) << 8);

#if defined(FEATURE_mPERS_CAM_Debug)    //aiden_test 1126
                g_quality = camera_quality_set;
#else
						g_quality = *(dataBuf + dataBufCnt + P_UART_DATA + 2);
#endif
						camera_capture_start ();
						break;
						
					case PICTURE_TRANS_START:
						printf ("PICTURE TRANSMIT START\n");

						if (*(dataBuf + dataBufCnt + P_UART_DATA) == 0x01) {
							printf ("START...\n");
							camera_capture_trans (1);
						}
						break;

					case PART_TRANS_RES:
						printf ("PART TRANS RESPONSE\n");
						if (*(dataBuf + dataBufCnt + P_UART_DATA) == 0x00) {
							camera_capture_trans (1);
						}
						else {
							camera_capture_trans (*(dataBuf + dataBufCnt + P_UART_DATA) + 1);
						}
						break;

					default:
						break;
				}

				dataBufCnt += uartDataLen + 4;
				uartRxBuf->count -= uartDataLen + 4;
			}
			else {
				printf ("checksum or endcode error\n");
				if (uartRxBuf->count > 0) {
					uartRxBuf->count--;
					dataBufCnt++;
				}
			}
		}
		else {
			if (uartRxBuf->count > 0) {
				uartRxBuf->count--;
				dataBufCnt++;
			}
		}
	} while (uartRxBuf->count > 0);
#endif

	free (dataBuf);

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
  memset (&uartFifoBuf, 0, sizeof(uartFifoBuf));
#endif

	printf ("dataBuf memory free %d\n", uartRxBuf->count);
}

void uartRxData_frameParsingWiFi (UART_FIFO_BUF *uartRxBuf)
{
	uint8_t *dataBuf = (uint8_t *)malloc (BUF_SIZE);

	uint16_t i = 0;
	//uint8_t checkSum = 0;
	//uint32_t uartDataLen = 0;
	//uint16_t dataBufCnt = 0;
	//uint16_t backupCount = 0;

	memcpy ((char *)dataBuf, uartRxBuf->buf, uartRxBuf->count);
	//backupCount = uartRxBuf->count;

	printf ("<<<<<<<<<< WIFI Command <<<<<<<<<<<\n");
	for (i = 0; i < uartRxBuf->count; i++) {
		printf ("%02x ", *(dataBuf + i));
	}

	if( *(dataBuf + 0) == 0x02 && (*(dataBuf + 1) == 0x13 || *(dataBuf + 1) == 0x14) )
	{
	    switch ( *(dataBuf + 1) )
	    {
          case WPS_START_REQ:
           printf ("WPS START REQUEST\n");
           scan_initialise_wifi();
           break;
	    }
	}
	
	free (dataBuf);

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
   memset (&uartFifoBuf, 0, sizeof(uartFifoBuf));
#endif

	printf ("dataBuf memory free %d\n", uartRxBuf->count);
}

static void uart_rx_task()
{
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
   c_pFb.buf = (uint8_t*) malloc(BUF_SIZE*30);
#endif

    bzero(dtmp, RD_BUF_SIZE);
    
    while (1) 
    {
        if( uart_rx_bypass_get() == false )
        {
            const int rxBytes = uart_read_bytes(UART_NUM_1, dtmp, BUF_SIZE, 50 / portTICK_RATE_MS);
            if (rxBytes > 0) 
            {
                ESP_LOGI(TAG, "Read %d bytes", rxBytes);
                //ESP_LOG_BUFFER_HEXDUMP(TAG, dtmp, rxBytes, ESP_LOG_INFO);
                memcpy (uartFifoBuf.buf + uartFifoBuf.count, dtmp, rxBytes);
                uartFifoBuf.count += rxBytes;
    
                 if( camera_gpio_set == 1 )
                     uartRxData_frameParsingCam (&uartFifoBuf);
                 else if( wifi_gpio_set == 1 )
                 	    uartRxData_frameParsingWiFi (&uartFifoBuf);
            }
        }
    }
    
    free(dtmp);

//aiden_test 0302 camera
#if defined(FEATURE_mPERS_CAM_Debug)
   free(g_pFb->buf);
#endif
    
}

static xQueueHandle gpio_evt_queue = NULL;

#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

#if defined(FEATURE_mPERS_CAM_Debug)
            printf("GPIO[%d] interrupt, val: %d\n", io_num, gpio_get_level(io_num));
            if( gpio_get_level(io_num) == 0 )
            {
                if( io_num == GPIO_INPUT_CAMWAKE )
                {
            	       camera_gpio_set = 1;
                }
                else
                	 wifi_gpio_set = 1;
        	      //uart_write_bytes (UART_NUM_1, (void *)&camera_buffer[0], 1051);
            }
            else
            {
                if( io_num == GPIO_INPUT_CAMWAKE )
                	  camera_gpio_set = 0;
                else
                 	  wifi_gpio_set = 0;
            }
#else
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            uart_write_bytes (UART_NUM_1, (void *)&camera_buffer, 500);
#endif
        }
    }
}

#if defined(FEATURE_mPERS_CAM_Debug)
static void periodictimer_callback(void* arg)
{
    camera_booton();
    switch( Mode_State )
    {
        #if 1 //smt2
        case State_Boot :
  	         ESP_LOGI(TAG, "%s %d %d, %d", "State_Boot",camera_gpio_set, wifi_gpio_set, esp_boot_complete);
  	         esp_boot_complete++;
  	         if( esp_boot_complete == 2 )
  	         {
//aiden_test 0316 old camera
#if 0
#else
  	            esp_start();
#endif
  	            break;
  	         }
  	         if( camera_gpio_set == 1 )
            {
//aiden_test 0316 old camera
#if 0
#else
                 if( camera_boot_start )
#endif
                 {
   							camera_booton();
   							
         	          Mode_State = State_CamBootWait;
                 }
                ESP_LOGI(TAG, "%s %d %d", "Go to State_Idle",camera_gpio_set, wifi_gpio_set);
  	         }
  	         else if( wifi_gpio_set == 1 )
  	         {
                Mode_State = State_Idle;
                ESP_LOGI(TAG, "%s %d %d", "Go to State_Idle",camera_gpio_set, wifi_gpio_set);
  	         }
            break;

        case State_CamBootWait:
//aiden_test 0316 old camera
#if 0
                Mode_State = State_Idle;
#else
        	  if( camera_boot_good == 1 )
        	  {
            	  camera_start();
                Mode_State = State_Idle;
        	  }
#endif
            break;
            
        case State_Idle :
            if( camera_gpio_set == 1 || wifi_gpio_set == 1 )
            {
                Mode_State = State_Idle;
            }
            break;
        #else //smt1
        case State_Boot :
        	 if( ++camera_boot_count < 1 )
        	 	   break;

  	         if( camera_boot_good == 0 )
  	         {
  	             ESP_LOGI(TAG, "%s %d", "State_Boot",camera_gpio_set);
  	             if( camera_gpio_set == 1 )
  	             {
      	             gpio_set_level(19, 1);
      	             camera_booton();
  	             }
  	             break;
  	         }
            Mode_State = State_Idle;
            ESP_LOGI(TAG, "%s %d", "Go to State_Idle",camera_gpio_set);
            break;
            
        case State_Idle :
            if( camera_gpio_set == 1 )
            {
                Mode_State = State_Idle;
                camera_gpio_set = 0;
            }
            break;
        #endif
        
        default :
       	    Mode_State = State_Idle;
            break;
    }
}
#endif

#endif

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
#if defined(FEATURE_mPERS_CAM_Debug)
//aiden_test 0316 old camera
#if 0
        .baud_rate = 115200,
#else
        .baud_rate = 57600,
#endif
#else
        .baud_rate = 115200,
#endif
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

#if defined(FEATURE_mPERS_UART)
    uart_param_config(UART_NUM1, &uart_config);
    
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM1, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM1, BUF_SIZE * 2, 0, 0, NULL, 0);


    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_HILEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

#if defined(FEATURE_mPERS_CAM_Debug)
    gpio_set_level(GPIO_INPUT_CAMWAKE, 0);
#endif

#if defined(FEATURE_mPERS_CAM_Debug)
    gpio_set_level(GPIO_INPUT_WIFIWAKE, 0);
#endif

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_CAMWAKE, GPIO_INTR_HIGH_LEVEL);
#if defined(FEATURE_mPERS_WPS)
    gpio_set_intr_type(GPIO_INPUT_WIFIWAKE, GPIO_INTR_HIGH_LEVEL);
#endif

#if defined(FEATURE_mPERS_CAM_Debug)
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(16, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048*4, NULL, 10, NULL);
#else
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
#endif

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_CAMWAKE, gpio_isr_handler, (void*) GPIO_INPUT_CAMWAKE);
#if defined(FEATURE_mPERS_WPS)    
    gpio_isr_handler_add(GPIO_INPUT_WIFIWAKE, gpio_isr_handler, (void*) GPIO_INPUT_WIFIWAKE);
#endif   
#else
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);
#endif

#if defined(FEATURE_mPERS_CAM_Debug)
esp_timer_create_args_t timer_args = {
    .callback = &periodictimer_callback,
};

ESP_ERROR_CHECK( esp_timer_create( &timer_args, &periodic_timer ) );
#endif

#if defined(FEATURE_mPERS_CameraOV3660)

#if defined(FEATURE_mPERS_CAM_Debug)
    gpio_config_t io_conf_19;
    /// configure the PCM output pins
    //disable interrupt
    io_conf_19.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf_19.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO19
    io_conf_19.pin_bit_mask = GPIO_OUTPUT_PIN19_SEL;
    //disable pull-down mode
    io_conf_19.pull_down_en = GPIO_PULLDOWN_ENABLE;
    //disable pull-up mode
    io_conf_19.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf_19);
#endif

     gpio_set_level(19, 1);

    ESP_ERROR_CHECK(esp_timer_start_periodic( periodic_timer, Timer_Default_Boot/10 ));
#endif

#if defined(FEATURE_mPERS_UART)
    //Create a task to handler UART event from ISR
    xTaskCreate(&uart_rx_task, "uart_event_task", 1024*4, NULL, 12, NULL);
#else
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
#endif

}
