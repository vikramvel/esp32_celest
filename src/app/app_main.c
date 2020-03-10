/*
** @source __MAIN_C__
**
** __ShortDescription__
**
** @author Copyright (C) 2019 Author: Vikramvel
** @version 1.0.1.0   - Alpha release July 2019
**
** Notes: 
** - ESP Logging mechanism is not used as K8 is sensitive to UART tx 
**        packets sent, and we cannot disable logs form ESP core layers.
** - app_main - is where the core task starts.
**
**
** Happiness can be found in the darkest of times, 
** if one only remembers to read the commens and documentation.
********************************************************************/
#include "main.h"
#include "remote_log.h"

/*
 * TODO:
 * - Forward the logs to TELNET console for remote logging! - should be helpful in Remote logging
 * - Multi Socket communication ( Dynamic socket creation) ? worth it?
 * - Send Reset reason to server at start-up
 */

static short int  Error_count;
static short int  Signal_Strength;
static short int  Restart_count;
static short int  Restart_reason;
int Factory_reset_flag = 0;
int ap_test_mode_flag = 0;

int HBT_Flag            = 0;
int HBT_Pending         = 0;
int Initial_boot              = 0;
int received_command    = 0;
xTaskHandle Monitor_TaskHandle;
unsigned int MBcalculateCRC(unsigned char *data, unsigned int offset, unsigned int endNotIncl, int Standard);

//unsigned int MBcalculateCRC (unsigned char *data, unsigned int offset, int len, int custCRC);

/*
 * The Heart beat message to be sent to tserver, the format follows the existing server packet structure,
 * which fieldlink understands.
 *
 * 00: The Slave Address
 * 10: The Function Code 16 (Preset Multiple Registers, 10 hex - 16 )
 * 0014: The Data Address of the first register. (Notify Code)
 * 0005: The number of registers to write
 * 0009: The number of data bytes to follow
 * 00 00 00 00: The values - Signal Strength, Error Count, Restrat reason, Restart count
 * 00 00 : The CRC (cyclic redundancy check) for error checking.
 */
uint8_t HBT_Message[19] ;//= {0x00,0x10,0x00,0x14,0x00,0x05,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


int rssi               = 0;
signed int pkt_count    = 0;

// TAG needed for Logger
static const char *TAG = "MAIN";

//esp_timer_handle_t heartbeat_timer;

void init_uart(void) 
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // We won't use a buffer for sending data.
    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int txData(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(uart_num, data, len);
    return txBytes;
}

/* Task for receiveing UART packets */
static void rx_task(void *atx)
{
    static const char *RX_TASK_TAG = "RX_TASK";
//    printf("\n %s", RX_TASK_TAG);
//    printf("\n");
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    //uint8_t rx[122];
    while (1) 
    {
        if (!Initial_boot && (wificonfig_powerup.wifi_mode == ESP_STATION_MODE ))
        {
            rxBytes = 7;
            strcpy(rx_data,"Bootup");
            rx_data[7] = '\0';
            Initial_boot ++;
        }
        else
        {
            rxBytes = uart_read_bytes(uart_num, (uint8_t *) rx_data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
            if (rxBytes > 0) 
            {
                
       /*       for(int x=0; x<rxBytes; x++)
                {
                   printf(" [%x] %d", rx_data[x], rx_data[x]);
                }
                    printf("\n");
      */          
                rx_data[rxBytes] = 0;
        //      printf("Read %d bytes: '%s'\n", rxBytes, rx_data);
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, rx_data, rxBytes, ESP_LOG_INFO);
            }
            else
            {
                vTaskDelay(20 / portTICK_PERIOD_MS);
            }
         }
    }
    free(rx_data);
}

/* CRC map sync with K8 CRC */
const static uint8_t MapCRCHi[] =
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};

const static uint8_t MapCRCLo[] =
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};


unsigned int MBcalculateCRC(unsigned char *data, unsigned int offset, unsigned int endNotIncl, int Standard)
{
    unsigned int crc = 0xFFFF, uIndex;

    for (; offset < endNotIncl; offset++)
    {
        uIndex = (crc&0xff) ^ (0xFF & data[offset]);
        if(Standard)
        {
        	crc = (crc&0xff00)|((crc>>8) ^ MapCRCHi[uIndex]);
        	crc = (crc&0x00ff)|(MapCRCLo[uIndex]<<8);
        }
        else
        {
        	crc = (crc&0xff00)|((crc>>8) ^ uIndex);
        	crc = (crc&0x00ff)|(uIndex<<8);
        }
    } 
    return crc;
}//getCRC


/*
unsigned int MBcalculateCRC (unsigned char *data, unsigned int offset, int len, int custCRC)
{
  unsigned char crc[] = {0xFF,0xFF};
  int nextByte = 0;
  int uIndex; 
  for (int i = offset; i < offset+len && i < len; i++)
  {
    nextByte = 0xFF & ((unsigned char)data[i]);
    uIndex = crc[0] ^ nextByte; 
    if (!custCRC)
    {
      crc[0] = ( unsigned char)(crc[1] ^ uIndex);
      crc[1] = ( unsigned char)uIndex;
    }
    else
    {
      crc[0] = (unsigned char)(crc[1] ^ MapCRCHi[uIndex]);
      crc[1] = MapCRCLo[uIndex];
    }
  }
  return crc;
}//getCRC
*/
int  checkCRC(unsigned char *StartPtr, uint8_t Length)
{
	int ret = -1; // 1- success 0 -fail
	uint8_t CRCHi = 0xFF;
	uint8_t CRCLo = 0xFF;
	uint8_t Index = 0;
	uint16_t Iterator = 0;
	uint16_t CalculatedCRC = 0;
	uint16_t actualCRC = 0;

	for (Iterator = 0; Iterator <= (Length - 1); Iterator++)
	{
		Index = CRCHi ^ *StartPtr++;
		CRCHi = CRCLo ^ MapCRCHi[Index];
		CRCLo = MapCRCLo[Index];
	}

	CalculatedCRC = (uint16_t)(((uint16_t)(CRCHi) << 8) | ((uint16_t)CRCLo));

//	printf("[Length of packet is = %d] \r\n", Length);

	CRCLo = *StartPtr++;
	CRCHi = *StartPtr++;

	actualCRC = ((CRCLo << 8) | (CRCHi & 0xFF));

//	printf("[actualCRC is = %x]     \r\n", actualCRC);
//	printf("[CalculatedCRC is = %x] \r\n", CalculatedCRC);

	if (CalculatedCRC == actualCRC)
	{
		ret = 1;  // Success
	}
	else
	{
		ret = 0;  // Fail
	}

	return ret;
}

void intialize_params()
{
    version_buffer = malloc(16);
 #if 1
    sprintf(version_buffer,"1.0.1.373");
 #else
    version_buffer	        		= VERSION;
 #endif
    wifi_config_version		        = 0x98;
    g_version                     	= 0x99;
    CONNECTED_BIT                   = BIT0;
    uart_num 		            	= UART_NUM_0;
    prev_crc 		            	= 0;
    present_crc 		        	= 0;
    count 	            			= 0;
    connfd                        	= -1;
    Error_count                     = 0;
    Signal_Strength                 = 0;
    Restart_count                   = 0;
    // 0 - Trying to connect 1- Connected 2- GOT IP ADDRESS 3- Disconnected
    connection_status             	= 0; 
    close_conn                    	= 0;
    upgradeconnfd                 	= -1;
    app_watchdog_flag             	= 1;
    app_watchdog_counter          	= 0;
    server_rcu_pend_recv          	= 0;
    rcu_server_pend_recv          	= 0;
    stop_OTA_upgrade              	= 0;
    ota_upgrade_status            	= 0; // 0 - FAIL  1 - PASS
    ota_autorisation_state       	= 0; // 0 - FAIL  1 - PASS
    stop_serial_upgrade           	= 0;
    serial_upgrade_status         	= 0; // 0 - FAIL  1 - PASS
    // 0 - no connectivity  1 - connectivity
    access_point_status           	= 0;
    tx_fifo_empty                 	= 0;
    //0 - no data 1 - interrupt 2- time out go read data
    data_receive_flag             	= 0;
    led_timer_flag                	= 0;
    led_counter                   	= 0;
    led_pause                     	= 0;
    client_mac                   	= NULL;
    watchdog_timer_flag             = 0;
    ota_upgrade_timer_flag        	= 0; // 0 - not started  1 - started
    serial_upgrade_timer_flag     	= 0; // 0 - not started  1 - started
    RCU_SERVER_timer_flag           = 0;
    SERVER_RCU_timer_flag           = 0;
    HBT_timer_flag                  = 0;
    client_connected	      	    = 0;
    waiting_results                 = 0;
    ping_count                      = 1;  //how many pings per report
    ping_timeout                    = 500; //mS till we consider it timed out
    ping_delay                      = 5; //mS between pings
    gotConn                         = 0;
    success_full_login              = 0;
    wifiParamCheck                  = ERR_NW_MAX;
    RX_BUF_SIZE                     = 1024;
    rx_data                         = (char*) malloc(1025);
    mac_str                         = (char*) malloc(18);
    memset(rx_data,0,1025);
}

unsigned int CRC_Check(char *ucCRC_Buf, int ucBufLength)
{
    unsigned int uiX, uiY, uiCRC;
    unsigned char ucStart = 0;
    uiCRC = 0xFFFF; //set all 1

    if (ucBufLength <= 0 || ucStart > ucBufLength)
    {
        uiCRC = 0;
    }
    else
    {
        ucBufLength += ucStart;

        for (uiX = (unsigned int)ucStart; uiX < ucBufLength; uiX++)
        {
            uiCRC = (unsigned int)(uiCRC ^ ucCRC_Buf[uiX]);

            for (uiY = 0; uiY <= 7; uiY++)
            {
                if((uiCRC & 1) != 0)
                    uiCRC = (unsigned int)((uiCRC >> 1)^0xA001);
                else
                    uiCRC = (unsigned int)(uiCRC >> 1);
            }
        }
    }

    return uiCRC;
}

int getStrength(int points)
{
    wifi_ap_record_t wifidata;
    long rssi = 0;
    int averageRSSI=0;

    for (int i=0;i < points;i++)
    {
        if (esp_wifi_sta_get_ap_info(&wifidata)==0)
        {
            rssi += wifidata.rssi;
    //      vTaskDelay(10 / portTICK_PERIOD_MS);
    	}
    }
    averageRSSI=rssi/points;
    return averageRSSI;
}

/* This is where all the Tik-Tok happens*/
const esp_timer_create_args_t heartbeat_timer_args = 
{
   .callback = &heartbeat_timer_callback,
   .name = "heartbeat"
};
const esp_timer_create_args_t timer1_args =
{
    .callback = &app_restart,
    .name = "app_restart"
};
const esp_timer_create_args_t   timer2_args =
{
    .callback = &close_connection,
    .name = "close_connection"
};
const esp_timer_create_args_t rcu_server_pend_timer_args =
{
    .callback = &check_rcu_server_pending_receive,
    .name = "check_rcu_server_pending_receive"
};
const esp_timer_create_args_t server_rcu_pend_timer_args =
{
	.callback = &check_server_rcu_pending_receive,
	.name = "check_server_rcu_pending_receive"
};
const esp_timer_create_args_t OTA_upgrade_timer_args =
{
	.callback = &stopOTAupgrade,
	.name = "stopOTAupgrade"
};
const esp_timer_create_args_t serial_upgrade_timer_args =
{
	.callback = &stopserialupgrade,
	.name = "stopserialupgrade"
};
const esp_timer_create_args_t TX_FIFO_Empty_timer_args =
{
	.callback = &check_TX_FIFO_Empty_timer,
	.name = "check_TX_FIFO_Empty_timer"
};
const esp_timer_create_args_t data_receive_timer_args =
{
	.callback = &check_data_receive_timer,
	.name = "check_data_receive_timer"
};
const esp_timer_create_args_t led_blink_timer_args =
{
	.callback = &blink_led,
	.name = "blink_led"
};

int UART_Recv(char *rx_buffer)
{

	uint16_t index = 0;
    rx_buffer = NULL;
    rx_buffer = rx_data;
    index = strlen(rx_data);
    if(rx_buffer)
    {
        return index;
    }

	return index;
}

/* Below code is dead code block right now,
 * it is for the UART ISR based ringbuffer
 * mechanism.
 * We are using Rtos queue instead of ISR,
 * Keeping the blow chunk just in case.
 */
void check_TX_FIFO_Empty_timer(void *arg)
{
#ifdef Debug_Enable
    printf( "close_RS485_TxEN \r\n");
#endif
	tx_fifo_empty = 1;
}

void IRAM_ATTR uart_intr_handle(void *arg)
{
    uint16_t rx_fifo_len, status;
    uint16_t buf_idx = 0;
    #ifdef Debug_Enable
    printf( "\n I am in uart0_rx_intr_handler\r\n");
    #endif
    status = UART0.int_st.val; // read UART interrupt Status
	rx_fifo_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buf

	//status = UART1.int_st.val; // read UART interrupt Status
	//rx_fifo_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buf
    if(status)
	{
		// do nothing! 
	}

	while (rx_fifo_len)
	{
		ring_buffer_enqueue(rxBuff, UART0.fifo.rw_byte);
		//ring_buffer_enqueue(rxBuff, UART1.fifo.rw_byte);
		buf_idx++;
		rx_fifo_len--;
	}

	data_receive_flag = 1;
	start_data_receive_timer();

	// after reading bytes from buffer clear UART interrupt status
	uart_clear_intr_status(uart_num, 
            UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    esp_err_t err;
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
        err = esp_wifi_connect();
        if(wificonfig_powerup.ipInfo.ip.addr <= 0)
        {
             wifiParamCheck= 0;
        }
        else
        {
            wifiParamCheck = 1;
        }
		connection_status = 0;
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
        start_heartbeat_timer();
		connection_status = 1;
        gotConn = 1;
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		connection_status = 2;
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        gotConn = 1;
        ping_result();
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
        if(event->event_info.disconnected.reason == WIFI_REASON_AUTH_FAIL)
        {
            wifiParamCheck = 2;
        }
		connection_status = 3;
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_AP_START:
#ifdef Debug_Enable
		printf( "\n started AP");
#endif
		xEventGroupSetBits(wifi_event_group, BIT2);
		break;
	case SYSTEM_EVENT_AP_STACONNECTED: 
		client_mac = (char*) malloc(18);
		client_connected = 1;
		xEventGroupSetBits(wifi_event_group, BIT1);
		sprintf(client_mac,"%02x:%02x:%02x:%02x:%02x:%02x",
                    event->event_info.sta_connected.mac[0],
                        event->event_info.sta_connected.mac[1],
                            event->event_info.sta_connected.mac[2],
                                event->event_info.sta_connected.mac[3],
                                    event->event_info.sta_connected.mac[4],
                                        event->event_info.sta_connected.mac[5]);
//printf( "\n client connected [CLIENT MAC]: %s",client_mac);
      		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		xEventGroupSetBits(wifi_event_group, BIT1);
		client_connected = 0;
        	break;
       case SYSTEM_EVENT_AP_STOP:
	       break;

       default:
//I don't want do do this but Google says it's part of the code standard!
	       break;
       }
       return ESP_OK;
}

// Only way to reachout to outside world
void ping_result()
{
    tcpip_adapter_ip_info_t servipInfo;
    servipInfo.ip.addr = ipaddr_addr(ServerIpAddress);
    //	vTaskDelay(1000 / portTICK_PERIOD_MS);
    if ((gotConn)&&(!waiting_results)) {
   	    esp_ping_set_target(PING_TARGET_IP_ADDRESS_COUNT, &ping_count, sizeof(uint32_t));
   	    esp_ping_set_target(PING_TARGET_RCV_TIMEO, &ping_timeout, sizeof(uint32_t));
   	    esp_ping_set_target(PING_TARGET_DELAY_TIME, &ping_delay, sizeof(uint32_t));
   	    esp_ping_set_target(PING_TARGET_IP_ADDRESS, &servipInfo.ip.addr, sizeof(uint32_t));
   	    esp_ping_set_target(PING_TARGET_RES_FN, &pingResults, sizeof(pingResults));
   	    ////printf( "\nPinging Gateway IP WiFi rssi %d\n",getStrength(10));
   	    ping_init();
   	    waiting_results = 1;
    }
}

#if 0
void uart_ap_mode(void *pvParameters)
{
    int len = 0;
    char buf_1[1025];
    char readbuffer[16];
    
    while(1)
    {
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(buf_1,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
        }
        
        //printf("\n Data Receive Flag %d", data_receive_flag);
        if(len > 0)
        {
            printf(" \n %s - buf", buf_1);
            printf("\n");
            if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x46) && (buf_1[3] == 0x52) && (len == 5))
            {
                //printf("I Received ATFR command,Sorry I am Restarting Now \r\n");
                uart_write_bytes(uart_num,"OK", 2);
                esp_restart();
            }
            else if ((buf_1[0] == 0x2B) && (buf_1[1] == 0x2B) && (buf_1[2] == 0x2B))
            {
                uart_write_bytes(uart_num,"OK", 2);
#ifdef Debug_Enable
                printf("\n I am moving to RCU Configuration update\r\n");
#endif
            }
            else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x52) && (buf_1[3] == 0x45) && (len == 5))
            {
                //printf("I Received ATRE command,I am Resetting to Factory Default Now \r\n");
                resetwificonfiginfo();
                uart_write_bytes(uart_num,"OK", 2);
                esp_restart();
            }
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'H') && (buf_1[3] == 'V') && (buf_1[4] == 'A')  && (buf_1[5] == 'C') && (len == 7))
            {
                memset(readbuffer, 0, 256);
                sprintf(readbuffer, "%d", wificonfig_powerup.HVAC_type);
                readbuffer[strlen(readbuffer)] = '\0';
                uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
//              uart_write_bytes(uart_num, connection_status, strlen(connection_status));

            }
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'P') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
            {
                wificonfig_write.wifi_mode = ESP_ACCESSPOINT_MODE; // Set to AP mode
                savewificonfiginfo(&wificonfig_write);
                uart_write_bytes(uart_num,"OK", 2);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                current_mode = ESP_ACCESSPOINT_MODE;
			    esp_restart();
            }
            // SET STA mode - STEN
            else if ((buf_1[0] == 'S') && (buf_1[1] == 'T') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
            {
                wificonfig_write.wifi_mode = ESP_STATION_MODE; // Set to sta mode
                savewificonfiginfo(&wificonfig_write);
                uart_write_bytes(uart_num,"OK", 2);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                current_mode = ESP_STATION_MODE;
			    esp_restart();
            }
		    // SET STA mode - ATSLST
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'C') && (buf_1[3] == 'L') && (buf_1[4] == 'S') && (buf_1[5] == 'T') && (len == 7))
            {
			    if(client_connected)
			    {
	                uart_write_bytes(uart_num,"1", 1);
			    }
			    else
			    {
			        uart_write_bytes(uart_num,"0", 1);
			    }

            }
            else if ((buf_1[0] == '-') && (buf_1[1] == '.') && (buf_1[2] == '-'))
            {
                uart_write_bytes(uart_num,"3", 1);
            }
            else if (buf_1[0] == '@')
            {
                 //txData(&ip_info_json);
                 /*
                 txData(&wificonfig_powerup.ssid);
                 txData(&wificonfig_powerup.password);
                 txData(&wificonfig_powerup.ap_ssid);
                 txData(&wificonfig_powerup.ap_password);
                 txData(&wificonfig_powerup.auth_username);
                 txData(&wificonfig_powerup.auth_password);
                 txData(&wificonfig_powerup.SIpAddress);
                 txData(&wificonfig_powerup.SPort);
                 txData(&wificonfig_powerup.wifi_mode);
                 txData(&wificonfig_powerup.userpassword);
                 txData(&wificonfig_powerup.HVAC_type);
                 txData(&wificonfig_powerup.ipInfo.ip.addr);
                 txData(&wificonfig_powerup.ipInfo.gw.addr);
                 txData(&wificonfig_powerup.ipInfo.netmask.addr);
                 */
                 
            }
		    memset(buf_1,0,1025);
            len = 0;
        }
       // vTaskDelay( (TickType_t)10); /* allows the freeRTOS scheduler to take over if needed */
        //memset(rx_data,0,1025);
    }

}
#endif


/* Main Task */
/*
 * This will not start up any WiFi radio but ensure all the initialization 
 * and the bare minimum configuration and flash steps are taken care.
 * Appropriate tasks should be started depending on Station or AP mode is 
 * selected.
 */
void app_main()
{
    int len = 0;
	int j = 0, k = 0;
	char buf_1[1025];
	char readbuffer[1025];
	memset(buf_1,0,1025);

    /* Initialze all the default values for the vaiables */
	intialize_params();
	uint8_t mac[6];

   	esp_efuse_mac_get_default(mac);
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#ifdef Debug_Enable
	printf( "SDK version:%s \r\n", system_get_sdk_version());
#endif
	nvs_flash_init();

	rxBuff = ring_buffer_init(RX_RING_BUFFER_SIZE);
	//startuart();
    /* initialize UART config */
    init_uart();
    /* Start Wifi configuration feteching the values from flash */
	startwificonfig();

    /* Event for HTTP server */
	http_server_set_event_start();
    current_mode =  wificonfig_powerup.wifi_mode;

    //printf( "\n Intialized My current mode is: %d \n", wificonfig_powerup.wifi_mode);
    
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    
    esp_reset_reason_t reason = esp_reset_reason();
    
    // printf("\n Reset Reason : %X"  ,  reason);
    
    Restart_reason = (int *) reason;
   
    // 999 is used as a flag here, instead of introducing new parameter for factory reset flag,
    // I used HBTtime from the nvs already to have a check flag.
    if(Restart_reason == 4 &&  wificonfig_powerup.HBtime == 999)
    {
        // Again adding NVM flag just to catch hold a factory reset crash again from here.
        wificonfig_write.HBtime = 999;
        savewificonfiginfo(&wificonfig_write);
        resetwificonfiginfo();
    }
    
    memset(readbuffer, 0, 256);
    
    
    sprintf(readbuffer, "%X", reason);
    readbuffer[strlen(readbuffer)] = '\0';
  
    //uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
    
    //printf("\n reboot count: %d", wificonfig_powerup.Reboot_count);
    printf("\n");

    /* Count will be reset if the reboot happens through AT commands (ATFR) */
    
    Restart_count = wificonfig_powerup.Reboot_count;
    wificonfig_write.Reboot_count = wificonfig_powerup.Reboot_count + 1; // Reset the Reboot count everytime a proper software reboot happens. As this will prevent in uncontrolled increase of the count.
    savewificonfiginfo(&wificonfig_write);

    /* Hand over from HTTP server - commisioning
     * Need to set to station mode and reboot. 
     * 4 - is an arbitary value choosen out of range
     * with existing modes 
     */
    if(wificonfig_powerup.wifi_mode == 4)
    {
        wificonfig_write.wifi_mode = ESP_STATION_MODE;
        savewificonfiginfo(&wificonfig_write);         
        esp_restart();
    }
    do
	{  
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(buf_1 ,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
        }
        //printf("\n rxbytes %d - %s", len,buf_1);
        //printf("\n");
        /* List of system level AT commands which needs to be configured irrespective of Radio mode */
	    if(len > 0)
	    {
		    // Set ATACID - AP SSID
		    if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'A') && (buf_1[3] == 'C') && (buf_1[4] == 'I') && (buf_1[5] == 'D') && (len > 6))
		    {
			   for (j = 6, k = 0; j < len; j++)
			   {
                    wificonfig_write.ap_ssid[k] = buf_1[j];
			        k++;
			   }
			   wificonfig_write.ap_ssid[k - 1] = '\0';
			   uart_write_bytes(uart_num,"OK", 2);
			   MFG_SSID_SET = 1;
#ifdef Debug_Enable
			   printf( "wificonfig_write.ap_ssid is [%s] \r\n", wificonfig_write.ap_ssid);
#endif
		    }
            else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x56) && (buf_1[3] == 0x52) && (len == 5))
            {
               uart_write_bytes(uart_num,version_buffer, strlen(version_buffer));
            }
            else if ((buf_1[0] == '-') && (buf_1[1] == '.') && (buf_1[2] == '-'))
            {
               uart_write_bytes(uart_num,1, 1);
            }
		    // GET MAC address - ATMAC mac_str
            else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x4D) && (buf_1[3] == 0x41) && (buf_1[4] == 0x43) && (len == 6))
            {
#ifdef Debug_Enable
               printf( "\n My Mac address %s", mac_str);
#endif
			   uart_write_bytes(uart_num,mac_str, strlen(mac_str));
            }
		    // GET Signal Strength - ATCSQ
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'C') && (buf_1[3] == 'S') && (buf_1[4] == 'Q') && (len == 6))
            {
                memset(readbuffer, 0, 256);
                sprintf(readbuffer, "%d", getStrength(2));
                readbuffer[strlen(readbuffer)] = '\0';
                uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
#ifdef Debug_Enable
                printf( "\n My Signal Strength %s", readbuffer);
#endif
            }
		    else if ((buf_1[0] == '+') && (buf_1[1] == '+') && (buf_1[2] == '+') && (len == 3))
            {
                uart_write_bytes(uart_num,"OK", 2);
            }
            /*
             * Removed AP mode commands
		    // Set ATACPK - AP password
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'A') && (buf_1[3] == 'C') && (buf_1[4] == 'P') && (buf_1[5] == 'K') && (len > 6))
		    {
			    for (j = 6, k = 0; j < len; j++)
			    {
				    wificonfig_write.ap_password[k] = buf_1[j];
				    k++;
			    }
			    wificonfig_write.ap_password[k - 1] = '\0';
			    uart_write_bytes(uart_num,"OK", 2);
#ifdef Debug_Enable
			    printf( "wificonfig_write.ap_ssid is [%s] \r\n", wificonfig_write.ap_password);
#endif
		    }
		    // Get ATACID
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'A') && (buf_1[3] == 'C') && (buf_1[4] == 'I') && (buf_1[5] == 'D') && (len == 6))
		    {
			    memset(readbuffer, 0, 256);
			    strncpy(readbuffer, wificonfig_powerup.ap_ssid, strlen(wificonfig_powerup.ap_ssid));
			    readbuffer[strlen(readbuffer)] = '\0';
			    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
		    }
		    // Get ATACPK
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'A') && (buf_1[3] == 'C') && (buf_1[4] == 'P') && (buf_1[5] == 'K') && (len == 6))
		    {
			    memset(readbuffer, 0, 256);
			    strncpy(readbuffer, wificonfig_powerup.ap_password, strlen(wificonfig_powerup.ap_password));
			    readbuffer[strlen(readbuffer)] = '\0';
			    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
		    }
            */
		    // Write to Flash Command - ATWR
		    else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x57) && (buf_1[3] == 0x52) && (len > 5))
		    {
			    ////printf( "I Received Write to Flash command \r\n");
			    savewificonfiginfo(&wificonfig_write);
			    uart_write_bytes(uart_num,"OK", 2);
		    }
		    // ATCN Command - ATCN - Apply changes and come out of configuration mode
		    else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x43) && (buf_1[3] == 0x4e) && (len == 5))
		    {
			    ////printf( "I Received ATCN command \r\n");
			    // Apply all the changes and move out of configuration mode
			    uart_write_bytes(uart_num,"OK", 2);
		    }
		    // Restart Command - ATFR - Module Reset
		    else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x46) && (buf_1[3] == 0x52) && (len == 5))
		    {
			    ////printf( "I Received ATFR command,Sorry I am Restarting Now \r\n");
			    uart_write_bytes(uart_num,"OK", 2);
                pkt_count = 0;
                wificonfig_write.Reboot_count = 0; // Reset the Reboot count everytime a proper software reboot happens. As this will prevent in uncontrolled increase of the count.
                savewificonfiginfo(&wificonfig_write);
			    esp_restart();
		    }
		    // ATRE
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'R') && (buf_1[3] == 'E') && (len == 5))
            {
                wificonfig_write.HBtime = 999;
                uart_write_bytes(uart_num,"atre", 4);
                savewificonfiginfo(&wificonfig_write);
                resetwificonfiginfo();
                pkt_count = 0;
                uart_write_bytes(uart_num,"OK", 2);
                esp_restart();
		    }
            /*
		    // SET AP Mode - APEN
		    else if ((buf_1[0] == 'A') && (buf_1[1] == 'P') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
		    {
			    wificonfig_write.wifi_mode = ESP_ACCESSPOINT_MODE; // Set to AP mode
			    savewificonfiginfo(&wificonfig_write);
			    uart_write_bytes(uart_num,"OK", 2);
			    vTaskDelay(10 / portTICK_PERIOD_MS);
		    //  //printf( "\n Set to AP mode : %d \n",  wificonfig_write.wifi_mode);
			    current_mode = ESP_ACCESSPOINT_MODE;
			    esp_restart();
            }
            */
		    // SET STA mode - STEN
		    else if ((buf_1[0] == 'S') && (buf_1[1] == 'T') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
            {
                wificonfig_write.wifi_mode = ESP_STATION_MODE; // Set to sta mode
			    savewificonfiginfo(&wificonfig_write);
			    uart_write_bytes(uart_num,"OK", 2);
			    vTaskDelay(10 / portTICK_PERIOD_MS);
            //  //printf( "\n Set to STA mode : %d \n",  wificonfig_write.wifi_mode);
			    current_mode = ESP_STATION_MODE;
			    esp_restart();
            }
            // STCL - SET mfg test ssid and password in STA mode
		    else if ((buf_1[0] == 'S') && (buf_1[1] == 'T') && (buf_1[2] == 'C') && (buf_1[3] == 'L') && (len == 5))
		    {
			    uart_write_bytes(uart_num,"OK", 2);
			    set_mfg_test_clientmode();
		    }
            // APST - SET mfg test ssid and password in AP mode
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'P') && (buf_1[2] == 'S') && (buf_1[3] == 'T') && (len == 5))
            {
                 uart_write_bytes(uart_num,"OK", 2);
                 ap_test_mode_flag = 1;
                 set_mfg_test_apmode();
            }
	    }
	    len = 0;
        memset(buf_1,0,1025);
	    //uart_flush(uart_num);
	    vTaskDelay(20 / portTICK_PERIOD_MS);   // Have some time to breathe for the task
	} while( current_mode == ESP_MODE_UNSET);

    if( wificonfig_powerup.wifi_mode == ESP_STATION_MODE)
    {
        current_mode = ESP_STATION_MODE;
        intwifi();
        //printf( "\n I am about to start station_sm task- hello world \r\n");
        /* Start the core network state maching */
        xTaskCreate(&station_sm, "station_sm", 4096, NULL, configMAX_PRIORITIES-1, NULL);
        xTaskCreate(&monitor_task, "monitor_task", 2048, NULL, 1, &Monitor_TaskHandle );
    }
    else if( wificonfig_powerup.wifi_mode == ESP_ACCESSPOINT_MODE)
    {
        current_mode = ESP_ACCESSPOINT_MODE;
        //printf( "\n I am in AP mode");
        //printf("\n ");
        wifi_init_softap();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        while(1)
        {
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(buf_1 ,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
        }
        //printf("\n len :%d data - %s", len, buf_1);
        //printf("\n ");
        if(len > 0)
        {
            // ATFR
            if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x46) && (buf_1[3] == 0x52) && (len == 5))
            {
                //printf("I Received ATFR command,Sorry I am Restarting Now \r\n");
                uart_write_bytes(uart_num,"OK", 2);
                esp_restart();
            }
            // +++
            else if ((buf_1[0] == 0x2B) && (buf_1[1] == 0x2B) && (buf_1[2] == 0x2B))
            {
                uart_write_bytes(uart_num,"OK", 2);
#ifdef Debug_Enable
                printf("\n I am moving to RCU Configuration update\r\n");
#endif
            }
            // ATRE
            else if ((buf_1[0] == 0x41) && (buf_1[1] == 0x54) && (buf_1[2] == 0x52) && (buf_1[3] == 0x45) && (len == 5))
            {
                //printf("I Received ATRE command,I am Resetting to Factory Default Now \r\n");
                Factory_reset_flag = 1;
                 uart_write_bytes(uart_num,"atre", 4);
                wificonfig_write.HBtime = 999;
                savewificonfiginfo(&wificonfig_write);
                resetwificonfiginfo();
                uart_write_bytes(uart_num,"OK", 2);
                esp_restart();
            }
            // ATHVAC
            /*
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'H') && (buf_1[3] == 'V') && (buf_1[4] == 'A')  && (buf_1[5] == 'C') && (len == 7))
            {
                memset(readbuffer, 0, 256);
                sprintf(readbuffer, "%d", wificonfig_powerup.HVAC_type);
                readbuffer[strlen(readbuffer)] = '\0';
                uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
//              uart_write_bytes(uart_num, connection_status, strlen(connection_status));

            }
            */
            /*
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'P') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
            {
                wificonfig_write.wifi_mode = ESP_ACCESSPOINT_MODE; // Set to AP mode
                savewificonfiginfo(&wificonfig_write);
                uart_write_bytes(uart_num,"OK", 2);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                current_mode = ESP_ACCESSPOINT_MODE;
                esp_restart();
            }
            */
            // SET STA mode - STEN
            else if ((buf_1[0] == 'S') && (buf_1[1] == 'T') && (buf_1[2] == 'E') && (buf_1[3] == 'N') && (len == 5))
            {
                wificonfig_write.wifi_mode = ESP_STATION_MODE; // Set to sta mode
                savewificonfiginfo(&wificonfig_write);
                uart_write_bytes(uart_num,"OK", 2);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                current_mode = ESP_STATION_MODE;
                esp_restart();
            }
            // SET STA mode - ATCLST
            else if ((buf_1[0] == 'A') && (buf_1[1] == 'T') && (buf_1[2] == 'C') && (buf_1[3] == 'L') && (buf_1[4] == 'S') && (buf_1[5] == 'T') && (len == 7))
            {
                if(client_connected)
                {
                    uart_write_bytes(uart_num,"1", 1);
                }
                else
                {
                    uart_write_bytes(uart_num,"0", 1);
                }
            }

        }
        memset(buf_1,0,1025);
        len = 0;
        vTaskDelay(20 / portTICK_PERIOD_MS);
        }
//        xTaskCreate(&uart_ap_mode, "uart_ap_mode", 4096, NULL, 2, NULL);
//        xTaskCreate(&http_server, "http_server", 4096, NULL, 2, NULL);
    }

}

void close_connection(void *arg)
{
    connection_timer_flag = 0;
	if (1 == close_conn)
	{
		ota_autorisation_state = 0;
		close_conn = 2; // Connection is closed
		close(connfd);
		connfd = -1;
        //printf( "Connection timer is expired, please close connection\r\n");
	}
}

esp_err_t pingResults(ping_target_id_t msgType, esp_ping_found * pf)
{
//	printf( "\n AvgTime:%.1fmS Sent:%d Rec:%d Err:%d min(mS):%d max(mS):%d ", (float)pf->total_time/pf->recv_count, pf->send_count, pf->recv_count, pf->err_count, pf->min_time, pf->max_time );
//	printf( "Resp(mS):%d Timeouts:%d Total Time:%d\n",pf->resp_time, pf->timeout_count, pf->total_time);
    Error_count = Error_count + ( pf->send_count - pf->recv_count);
    if( (pf->send_count - pf->recv_count) < pf->send_count)
    {
        wifiParamCheck = 4;
        start_heartbeat_timer();
        //printf( "\n *** Ping Successfull \n ");
    }
    else if( (pf->send_count - pf->recv_count) == pf->send_count)
    {
        wifiParamCheck = 3;
        //ESP_ERROR_CHECK(esp_timer_stop(heartbeat_timer));
        //ESP_ERROR_CHECK(esp_timer_delete(heartbeat_timer));
        //printf( "\n **** error count greater than 0");
    }
	waiting_results = 0;
	return ESP_OK;
}

/* Not used for now can use for debugging on GPIO 5 */
void blink_led(void *arg)
{
	int value = 0;

	led_timer_flag = 0;

//	value = GPIO_INPUT_GET(5);

	if (0 == value)
	{
		//GPIO_OUTPUT_SET(5, 1);
	}
	else
	{
		//GPIO_OUTPUT_SET(5, 0);
	}
}

void check_rcu_server_pending_receive(void *arg)
{
    RCU_SERVER_timer_flag = 0;
    rcu_server_pend_recv = 1;
//	printf( "RCU_SERVER_pending_receive timer expired \r\n");
}

void heartbeat_timer_callback(void *arg)
{
    if( wifiParamCheck == 4)
    {
        HBT_Flag = 1;
    }
    HBT_timer_flag = 0;
    //printf("HEART BEAT HIT \r\n");

}
void check_server_rcu_pending_receive(void *arg)
{
	server_rcu_pend_recv = 1;
    SERVER_RCU_timer_flag = 0;
    uart_write_bytes(uart_num,0x00, 1);
//	printf( "Server_RCU_pending_receive timer expired \r\n");

}

void check_data_receive_timer(void *arg)
{
	data_receive_flag = 2;
//	printf( "data_receive_flag timer expired \r\n");
}

void app_restart(void *arg)
{
    watchdog_timer_flag = 0;
//	printf( " Watchdog restart");
	esp_restart();
}

void stopOTAupgrade(void *arg)
{
    ota_upgrade_timer_flag = 0;
	stop_OTA_upgrade = 1;
//	printf( "Stop OTA upgrade, timer expired \r\n");
}

void stopserialupgrade(void *arg)
{
	stop_serial_upgrade = 1;
	//printf( "Stop Serial upgrade, timer expired \r\n");
}

/* I am not sure if we need this, but ESP should handle its own 
 * reboot/ fall back mechanism
 */
void start_app_watchdog()
{
	//esp_timer_handle_t  timer1;
    if (watchdog_timer_flag)
    {
        ESP_ERROR_CHECK(esp_timer_stop(timer1));
        ESP_ERROR_CHECK(esp_timer_delete(timer1));
    }
    watchdog_timer_flag = 1;

	ESP_ERROR_CHECK(esp_timer_create(&timer1_args, &timer1));
	ESP_ERROR_CHECK(esp_timer_start_once(timer1, 31 * 1000 * 1000));// micro seconds
}

void start_OTA_upgradetimer()
{
	if (ota_upgrade_timer_flag)
	{
        ESP_ERROR_CHECK(esp_timer_stop(OTA_upgrade_timer));
        ESP_ERROR_CHECK(esp_timer_delete(OTA_upgrade_timer));
	}
	ota_upgrade_timer_flag = 1;
	ESP_ERROR_CHECK(esp_timer_create(&OTA_upgrade_timer_args, &OTA_upgrade_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(OTA_upgrade_timer, 29 * 1000 * 1000)); // 10 sec
}

void start_serial_upgradetimer()
{
	if (serial_upgrade_timer_flag)
	{
		ESP_ERROR_CHECK(esp_timer_stop(serial_upgrade_timer));
        ESP_ERROR_CHECK(esp_timer_delete(serial_upgrade_timer));

	}
	ota_upgrade_timer_flag = 1;
	ESP_ERROR_CHECK(esp_timer_create(&serial_upgrade_timer_args, &serial_upgrade_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(serial_upgrade_timer, 30 * 1000 * 1000));
}

void start_rcu_server_pend_timer()
{
    if (RCU_SERVER_timer_flag)
    {
        RCU_SERVER_timer_flag = 0;
        ESP_ERROR_CHECK(esp_timer_stop(rcu_server_pend_timer));
        ESP_ERROR_CHECK(esp_timer_delete(rcu_server_pend_timer));
    }
    RCU_SERVER_timer_flag = 1;
	ESP_ERROR_CHECK(esp_timer_create(&rcu_server_pend_timer_args, &rcu_server_pend_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(rcu_server_pend_timer, 4*1000 * 1000));
}

void start_heartbeat_timer()
{
    if (HBT_timer_flag)
    {
        HBT_timer_flag = 0;
        ESP_ERROR_CHECK(esp_timer_stop(heartbeat_timer));
        ESP_ERROR_CHECK(esp_timer_delete(heartbeat_timer));
    }
    HBT_timer_flag = 1;

    ESP_ERROR_CHECK(esp_timer_create(&heartbeat_timer_args, &heartbeat_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer, ((10) * 1000 * 1000))); // 10 sec
}

void start_server_rcu_pend_timer()
{
    if (SERVER_RCU_timer_flag)
    {
        SERVER_RCU_timer_flag = 0;

        ESP_ERROR_CHECK(esp_timer_stop(server_rcu_pend_timer));
        ESP_ERROR_CHECK(esp_timer_delete(server_rcu_pend_timer));
    }

    SERVER_RCU_timer_flag = 1;

	ESP_ERROR_CHECK(esp_timer_create(&server_rcu_pend_timer_args, &server_rcu_pend_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(server_rcu_pend_timer, 2000 * 1000));
}

/* Dead Timer Code - Should be cleaned up */
void start_TX_FIFO_Empty_timer(int delay)
{
	esp_timer_handle_t  TX_FIFO_Empty_timer;
	ESP_ERROR_CHECK(esp_timer_create(&TX_FIFO_Empty_timer_args, &TX_FIFO_Empty_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(TX_FIFO_Empty_timer, delay * 1000));
}

void start_data_receive_timer()
{  
	esp_timer_handle_t  data_receive_timer;
	ESP_ERROR_CHECK(esp_timer_create(&data_receive_timer_args, &data_receive_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(data_receive_timer, 20 * 1000));
}

void start_connection_timer()
{
    if (connection_timer_flag)
    {
        connection_timer_flag = 0;

        ESP_ERROR_CHECK(esp_timer_stop(timer2));
        ESP_ERROR_CHECK(esp_timer_delete(timer2));
    }
    connection_timer_flag = 1;
	
    ESP_ERROR_CHECK(esp_timer_create(&timer2_args, &timer2));
	ESP_ERROR_CHECK(esp_timer_start_once(timer2, 3 *1000 * 1000));
}

void start_led_timer(int delay)
{
	esp_timer_handle_t  led_blink_timer;
	ESP_ERROR_CHECK(esp_timer_create(&led_blink_timer_args, &led_blink_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(led_blink_timer, delay * 1000));
}

/* Please work */
void station_sm(void *pvParameters)
{
	int len                     = 0;
	int len1                    = 0;
	signed int servlen          = 0;
	int servlen1                = 0;
	int upgradelen              = 0;
	unsigned long filesize      = 0;
    unsigned long file_length   = 0;
    unsigned long recv_filesize = 0;
	int j = 0, k = 0, i = 0, t = 0 ;
	int ret = 0, ret1 = 0;
	static int l = 0, m = 0, n = 0;
	int sockfd                  = -1;
	int serversockfd            = -1;
	int upgradesockfd           = -1;
	struct sockaddr_in serv_addr;
	struct sockaddr_in upgrade_addr;
	char configIpAddress[16];
	char configgw[16];
	char confignetmask[16];
	char SPort[10];
    char Rwt[10];				// RCU WiFi Timeout & also used for heartbeat timepout
	char baud_rate[10];
	char ota_username[SSID_SIZE];
	char ota_password[PASSWORD_SIZE];
	static int flag = 0;
	signed long Flags;
	char *temp_addr;
	char readbuffer[1025];
	uint8_t channel_number      = 0;
	uint8_t mac[6];
	//char *mac_str = (char*) malloc(18);
	int end_status = 0;
	esp_efuse_mac_get_default( mac);
	esp_ota_handle_t update_handle = 0;
	const esp_partition_t *update_partition = NULL;
    //static uint32_t sFlashCurrentAddress;
    //printf("\n station Mode");
    //printf("\n");
	esp_partition_t *configured = NULL;
	esp_partition_t *running = NULL;	
	esp_err_t err;
	sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	serversockfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&serv_addr, 0, sizeof(serv_addr));

    // TODO: should be dynamically able to configure the sport from AT commands.
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(ServerPort);

	bind(serversockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

	/* ioctl to get current socket options */
	if ((Flags = fcntl(serversockfd, F_GETFL, 0)) == -1)
	{
	    //printf( "ETH: Fail to get socket flag option\r\n");
		close(serversockfd);
	}

	/* ioctl to set current socket as non blocking */
	if (fcntl(serversockfd, F_SETFL, Flags | O_NONBLOCK) == -1)
	{
		//printf( "ETH: Fail to set socket flag option\r\n");
		close(serversockfd);
	}

	listen(serversockfd, 5);

	upgradesockfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&upgrade_addr, 0, sizeof(upgrade_addr));

	upgrade_addr.sin_family = AF_INET;
	upgrade_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	upgrade_addr.sin_port = htons(4097);

	bind(upgradesockfd, (struct sockaddr *)&upgrade_addr, sizeof(upgrade_addr));

	/* ioctl to get current socket options */
	if ((Flags = fcntl(upgradesockfd, F_GETFL, 0)) == -1)
	{
		//printf( "ETH: Fail to get socket flag option\r\n");
		close(upgradesockfd);
	}

	/* ioctl to set current socket as non blocking */
	if (fcntl(upgradesockfd, F_SETFL, Flags | O_NONBLOCK) == -1)
	{
		//printf( "ETH: Fail to set socket flag option\r\n");
		close(upgradesockfd);
	}

	listen(upgradesockfd, 5);
    //remote_log_init();
    ////start_app_watchdog();
    // THE LOOP THAT DO EVERYTHING!!!!!!!
	while (1)
	{
	    switch (flag)
		{
		case RCU_SERVER:
		{
			len = 0;
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(buf,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
            if((buf[0] == 0) && (buf[1] == 0) && (buf[3] == 0 ))
            {
                len = 0;
            }
        }
		if (len > 0)
		{
           /* 
			printf("\n RCU_SERVER Data Read from UART len is: [len = %d]\r\n", len);
			for (i = 0; i<len; i++)
			{
				printf("%x\t", buf[i]);
			}
			printf("\r\n");
            */
			if ((buf[0] == 0x2B) && (buf[1] == 0x2B) && (buf[2] == 0x2B))
			{
				uart_write_bytes(uart_num,"OK", 2);
                //start_app_watchdog();
				//printf( "I am moving to RCU Configuration update\r\n");
				flag = RCU_CONFIG;
				break;
			}
            // STCL - SET mfg test ssid and password in STA mode
            else if ((buf[0] == 'S') && (buf[1] == 'T') && (buf[2] == 'C') && (buf[3] == 'L'))
            {
                uart_write_bytes(uart_num,"OK", 2);
                set_mfg_test_clientmode();
            }
            // APST - SET mfg test ssid and password in AP mode
            else if ((buf[0] == 'A') && (buf[1] == 'P') && (buf[2] == 'S') && (buf[3] == 'T'))
            {
                 uart_write_bytes(uart_num,"OK", 2);
                 ap_test_mode_flag = 1;
                 set_mfg_test_apmode();
            }
            //ATCLST - Client connected
            else if ((buf[0] == 'A') && (buf[1] == 'T') && (buf[2] == 'C') && (buf[3] == 'L') && (buf[4] == 'S') && (buf[5] == 'T') && (len == 7))
            {
                if(client_connected)
                {
                    uart_write_bytes(uart_num,"1", 1);
                }
                else
                {
                    uart_write_bytes(uart_num,"0", 1);
                }
            }
			else if ((buf[0] == 0x2D) && (buf[1] == 0x2D) && (buf[2] == 0x2D))
			{
				uart_write_bytes(uart_num,"OK", 2);
				//os_delay_us(5 * 1000);
				//printf( "I am moving to Fimrware Upgrade Mode over the serial\r\n");
				flag = RCU_UPGRADE_SERIAL;
				start_serial_upgradetimer();
                //start_app_watchdog();
				break;
			}
            /* Heartbeat */
            else if (buf[0] == '-' && buf[1] == '.' && buf[2] == '-')
            {
                uart_write_bytes(uart_num,"2", 1);
                //start_app_watchdog();
                break;
            }
			
//          uint16_t crc = MBcalculateCRC(buf, 0, 14, 1);
//          int r_crc = checkCRC(buf, 14);

//          HBT_Message[17] = 0x00;
//          printf("\n %d %d \n", crc, r_crc);
/*
			printf( "RCU_SERVER Data Read from UART len is: [len = %d]\r\n", len);

            for (i = 0; i<len; i++)
            {
                printf("%x\t", buf[i]);
            }
            printf("\r\n");
  */          
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd == -1)
				{
					//printf( "Failure: socket create failure \r\n");
					//len = UART_Recv((char *)&buf);
					//printf( "Clear buffer [len = %d]\r\n", len);
					memset(buf, 0, 2048);
					close(sockfd);
					sockfd = -1;
					len = 0;
					break;
				}
				else
				{
					struct sockaddr_in dest;
#if 0
                    /* ToDo:
					 * implement a way see if server port and IP is configured
					 * if not send packets to received IP?
					 * Or should a override flag needs to be implemented?
					 */
					char temp_data[128] = {0};
					int data_len = 0;
					data_len =  (buf[1] & 0xff) | ((buf[2] & 0xff) << 8);
					r_ServerPort = (buf[9] & 0xff) | ((buf[10] & 0xff) << 8);
					//printf( "Received Server Port : %d", r_ServerPort);
					//printf( "Received Ip : %d.%d.%d.%d", buf[5],buf[6],buf[7],buf[8]);
                    /*for(int y=0; y< data_len;y++)
                    {
                        //printf("%x \t", buf[10+y]);
                    }
                    //printf("\n");*/
					strncpy(&temp_data,&buf[11], data_len);
					temp_data[data_len] = '\0';

					//printf( "Data len : %d", data_len);
					//printf( "Data : %s\n", temp_data);
					//printf( "---- complete");
#endif
					dest.sin_family = AF_INET;
					dest.sin_port = htons(ServerPort);
					dest.sin_addr.s_addr = inet_addr(ServerIpAddress);
                    //printf("\n ---\n");
					if (connect(sockfd, (struct sockaddr *)&dest, sizeof(dest)) != 0)
					{
						close(sockfd);
						sockfd = -1;
						flag = SERVER_RCU;
						//printf( "connection not successful \r\n");
                        Error_count ++;
						//len = UART_Recv((char *)&buf);
						//printf( "Clear buffer [len = %d]\r\n", len);
						memset(buf, 0, 2048);
						len = 0;
                        //uart_write_bytes(uart_num,"Serv Error", 8);
						break;
					}
					else
					{
						if ((Flags = fcntl(sockfd, F_GETFL, 0)) == -1)
						{
							//printf( "ETH: Fail to get socket flag option\r\n");
							close(sockfd);
							sockfd = -1;
						}
				     	// ioctl to set current socket as non blocking 
						if (fcntl(sockfd, F_SETFL, Flags | O_NONBLOCK) == -1)
						{
							//printf( "ETH: Fail to set socket flag option\r\n");
							close(sockfd);
							sockfd = -1;
						}
                        //start_app_watchdog();
						write(sockfd, buf, len);
                        Error_count = 0;
						flag = RCU_SERVER_PENDING;
						//printf( "Move to RCU_SERVER_PENDING state \r\n");
						// Indicate you want to receive data
						rcu_server_pend_recv = 0;
						// Start a timer here for one second
						start_rcu_server_pend_timer();
					}
				}
				memset(buf, 0, BUFSIZE);
			}
            else if( HBT_Flag == 1 && !Error_count)
            {
                vTaskSuspend(Monitor_TaskHandle);
                vTaskResume(Monitor_TaskHandle);
                memset(HBT_Message,0,19);//{0x00,0x10,0x00,0x14,0x00,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
                sockfd = socket(AF_INET, SOCK_STREAM, 0);
                if (sockfd == -1)
                {
                    //printf( "Failure: socket create failure \r\n");
                    //len = UART_Recv((char *)&buf);
                    //printf( "Clear buffer [len = %d]\r\n", len);
                    memset(buf, 0, 2048);
                    HBT_Flag = 0;
                    close(sockfd);
                    sockfd = -1;
                    len = 0;
                    vTaskResume(Monitor_TaskHandle);
                    Error_count ++;
                    break;
                }
                else
                {
                    // Server connect and send frame code.
                    struct sockaddr_in dest;
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(ServerPort);
                    dest.sin_addr.s_addr = inet_addr(ServerIpAddress);
                    if (connect(sockfd, (struct sockaddr *)&dest, sizeof(dest)) != 0)
                    {
                        close(sockfd);
                        sockfd = -1;
                        //printf("connection not successful- \r\n");
                        Error_count = Error_count + 1;
                        HBT_Flag = 0;
                    }
                    else
                    {
                        if ((Flags = fcntl(sockfd, F_GETFL, 0)) == -1)
                        {
                            //printf("ETH: Fail to get socket flag option\r\n");
                            close(sockfd);
                            sockfd = -1;
                            HBT_Flag = 0;
                            Error_count ++;
                        }
                        // ioctl to set current socket as non blocking
                        if (fcntl(sockfd, F_SETFL, Flags | O_NONBLOCK) == -1)
                        {
                            //printf("ETH: Fail to set socket flag option\r\n");
                            close(sockfd);
                            sockfd = -1;
                            HBT_Flag = 0;
                            Error_count ++;
                        }
                        uint8_t t1, t2;
 //                      printf("\n %d %d %d : %x %x %x ",Error_count,Signal_Strength,Restart_count,Error_count,Signal_Strength,Restart_count); 
                        HBT_Message[0]  = 0x00;
                        HBT_Message[1]  = 0x10;
                        HBT_Message[2]  = 0x00;
                        HBT_Message[3]  = 0x14;
                        HBT_Message[4]  = 0x00;
                        HBT_Message[5]  = 0x05;
                        HBT_Message[6]  = 0x0A;
                        HBT_Message[7]  = 0x01; // Identifier
                        HBT_Message[8]  = (uint8_t) (rssi & 0xff);
                        HBT_Message[9]  = (uint8_t) (rssi >> 8) & 0xff;
                        HBT_Message[10] = Error_count;
                        HBT_Message[11] = 0x00;
                        HBT_Message[12] = Restart_count;
                        HBT_Message[13] = 0x00;
                        HBT_Message[14] = (uint8_t) Restart_reason;
                        HBT_Message[15] = (uint8_t) (ServerPort & 0xff);
                        HBT_Message[16] = (uint8_t) ((ServerPort >> 8) & 0xff);
                        uint16_t crc = MBcalculateCRC(HBT_Message, 0, 17, 1);
                        HBT_Message[17] = crc&0xff;
                        HBT_Message[18] = crc>>8;
                        //printf("\n HBT - %d %d", HBT_Message[12], HBT_Message[14]);
                        //printf("\n");
                       // printf("\n %d %d-", crc, r_crc);

/*                        
                        for( i =0; i< sizeof(HBT_Message); i++)
                        {
                             printf(" %x ", HBT_Message[i]);
                        }
                        printf(" : %d", sizeof(HBT_Message));
                        printf("\n");
                        
*/                      
                        write(sockfd, HBT_Message, sizeof(HBT_Message));
                        Error_count = 0;
//                        start_rcu_server_pend_timer();
                        //uart_write_bytes(uart_num,(char *) HBT_Message, sizeof(HBT_Message));
                        HBT_Flag = 0;
                        HBT_Pending = 1;
                        flag = RCU_SERVER_PENDING;
                        //start_app_watchdog();
                        start_rcu_server_pend_timer();
                    }
                }
            }
            else if(Error_count)
            {
                ping_result();
                if( wifiParamCheck == 4)
                {
                    Error_count = 0;
                }
                else if (wifiParamCheck == 3)
                {
                    Error_count ++;
                }
            }
			else
			{
				flag = SERVER_RCU;
			}
		}
		break;
		case RCU_SERVER_PENDING:
		{
			// Stop the timer after we receive data 
			// wait in that state until we receive data
			// after that move to rcu to server again
            //start_rcu_server_pend_timer();
			len1 = 0;
            //printf("\n :1");
			len1 = read(sockfd, buffer, sizeof(buffer));
            if (len1 >0 && !HBT_Pending)
			{
                
				/*printf( "Data received from server is [len1 = %d] \r\n", len1);
				for (i = 0; i < len1; i++)
				{
					printf("%x \t", buffer[i]);
				}
				printf("\r\n");
                */
				uart_write_bytes(uart_num,( char *)&buffer, len1);
				//check_TX_FIFO_Empty_complete(len1);
				rcu_server_pend_recv = 1;
                //start_app_watchdog();
			}
            else if (len1 > 0 && HBT_Pending)
            {
               /*
               printf("Data received from server is [len1 = %d] \r\n", len1);
               for (i = 0; i < len1; i++)
               {
                    printf("%x \t", buffer[i]);
               }
               printf("\r\n");
               */
               //uart_write_bytes(uart_num,( char *)&buffer, len1);
               pkt_count ++;
               //printf(" count %d", pkt_count);
               //check_TX_FIFO_Empty_complete(len1);
               //start_app_watchdog();
               rcu_server_pend_recv = 1;
               HBT_Pending = 0;
               HBT_Flag = 0;
               vTaskResume(Monitor_TaskHandle);
            }
			if (1 == rcu_server_pend_recv)
			{
                if(HBT_Pending)
                {
                    HBT_Pending = 0;
                    rcu_server_pend_recv = 0;
                    HBT_Flag = 0;
                    flag = RCU_SERVER;
                    memset(buffer, 0, 1024);
                    close(sockfd);
                    sockfd = -1;
                }
                else
                {
			//	    printf( "Stop waiting for the pending response from Server \r\n");
				    rcu_server_pend_recv = 0;
				    flag = SERVER_RCU;
				    memset(buffer, 0, 1024);
				    close(sockfd);
				    sockfd = -1;
                }
			}
		}
		break;
		case SERVER_RCU:
		{
			//printf( "Server RCU ");
			servlen = 0;
			if (connfd == -1)
			{
				connfd = accept(serversockfd, (struct sockaddr *)NULL, NULL);
				//As soon as connection is accepted capture the time
				if (connfd != -1)
				{
					//printf( "New Incoming connection is accepted \r\n");

					/* ioctl to get current socket options */
					if ((Flags = fcntl(connfd, F_GETFL, 0)) == -1)
					{
					//	printf( "ETH: Fail to get socket flag option\r\n");
    					close(connfd);
					}
					/* ioctl to set current socket as non blocking */
					if (fcntl(connfd, F_SETFL, Flags | O_NONBLOCK) == -1)
					{
					//	printf( "ETH: Fail to set socket flag option\r\n");
						close(connfd);
					}
                    //start_app_watchdog();
					close_conn = 1; // A new connection is established
					start_connection_timer();
				}
				else
				{
					flag = RCU_SERVER;
				}
			}
			else
			{
			    //printf( "[I am about to read data from server] \r\n");
				servlen = read(connfd, serverbuffer, sizeof(serverbuffer));
            /*    
  			printf( "Connection is accepted from server data follows [%d] \r\n", servlen);
                for (i = 0; i < servlen; i++)
                {
                   printf("%x \t", serverbuffer[i]);
                }
                printf("\r\n");
*/
				if (servlen == 0)
                {
			//		printf( "[servlen = %d] \r\n", servlen);
                }
				if (servlen > 0)
				{
					if ((serverbuffer[0] == 0x2D) && (serverbuffer[1] == 0x2D) && (serverbuffer[2] == 0x2D))
					{
						write(connfd, "OK", 2);
						memset(serverbuffer, 0, 1024);
						ota_autorisation_state = 1;
                        //start_app_watchdog();
					}
                    else if ((serverbuffer[0] == 0x2B) && (serverbuffer[1] == 0x2B) && (serverbuffer[2] == 0x2B))
                    {
						write(connfd, "OK", 2);
						memset(serverbuffer, 0, 1024);
                        //start_app_watchdog();
					}
                    else if ((serverbuffer[0] == 0x41) && (serverbuffer[1] == 0x54) && (serverbuffer[2] == 0x56) && (serverbuffer[3] == 0x52))
	                {
						//printf( "Got ATVR from SERVER");
        	            write(connfd, version_buffer, strlen(version_buffer));
                        //start_app_watchdog();
                    }
					else
					{
						start_connection_timer();
 			/*		printf( "Connection is accepted from server data follows [%d] \r\n", servlen);

						for (i = 0; i < servlen; i++)
						{
							printf("%x \t", serverbuffer[i]);
						}
						printf("\r\n");
              */       
                        // Modification - DNG-941 : AT commands for ESP shold start with [ATCOMMAND+DATA
                        if( (serverbuffer[7]=='[') && (serverbuffer[8]==0x41) && (serverbuffer[9]==0x54 ))
						{
							//printf("\n Received AT COMMAND CALLING FUNC");
                        	ota_config(&serverbuffer,servlen);
							break;
						}
						uart_write_bytes(uart_num,( char *)&serverbuffer, servlen);
						//check_TX_FIFO_Empty_complete(servlen);
						flag = SERVER_RCU_PENDING;
//						printf( "Move to SERVER_RCU_PENDING state \r\n");
						// Indicate you want to receive data
						server_rcu_pend_recv = 0;
						// Start a timer here for one second
						start_server_rcu_pend_timer();
						memset(serverbuffer, 0, 1024);
                        //start_app_watchdog();

					}
				}
				else if ((0 == servlen) && (1 == close_conn))
				{
					if (1 == ota_autorisation_state)
					{
						ota_autorisation_state = 0;
						flag = RCU_UPGRADE_AUTHORISATION;
						start_OTA_upgradetimer();
					//	printf( "I am moving to Firmware Upgrade Mode Over The Air Authorisation\r\n");
					}
					else
					{
						flag = RCU_SERVER;
					}
					close(connfd);
					connfd = -1;
					close_conn = 2; // Connection is closed
			//		printf( "Connection is closed \r\n");
				}
			}
		}
		break;
		case SERVER_RCU_PENDING:
		{
			// Stop the timer after we receive data 
			// wait in that state until we receive data
			// after that move to rcu to server again

			if (servlen1 > 0)
            start_server_rcu_pend_timer();
	        servlen1 = 0;
            //printf(" Server RCU Pending ");
             if(rxBytes)
            {
                servlen1 = rxBytes;
                memcpy(serverbuffer1,rx_data,rxBytes);
                memset(rx_data,0,strlen(rx_data));
            }

			if (servlen1 > 0)
			{
#ifdef Debug_Enable
				//printf( "Data Read from UART and send as reply to server is: %d\r\n", servlen1);
				for (i = 0; i < servlen1; i++)
				{
					//printf("%x \t", serverbuffer1[i]);
				}
				//printf("\r\n");
#endif
				write(connfd, serverbuffer1, servlen1);
				server_rcu_pend_recv = 1;
                servlen1 = 0 ;
				memset(serverbuffer1, 0, 1024);
                //start_app_watchdog();
			}

			if (1 == server_rcu_pend_recv)
			{
			//	printf( "Stop waiting for the pending response from RCU \r\n");
				server_rcu_pend_recv = 0;
                flag = SERVER_RCU;
                //close(connfd);
                //connfd = -1;
                //close_conn = 2; // Connection is closed
             //   printf( "Connection is closed \r\n");
            }
		}
		break;
		case RCU_CONFIG:
		{
			len = 0;
//ESP_LOGI(TAG, " Config Mode");
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(configbuffer ,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
        }
/*
        if(strlen(rx_data))
        {
            len = strlen(rx_data);
            strncpy(configbuffer,rx_data,strlen(rx_data));
            memset(rx_data,0,strlen(rx_data));
        }
*/
			if (len > 0)
			{
                //start_app_watchdog();
#ifdef Debug_Enable
				//printf( "Received Command len [%x]\r\n", len);
				for (i = 0; i < len; i++)
				{
					//printf("Received Command is [%x]\t", configbuffer[i]);
				}
#endif
				// Set IP Address - ATMY 130.10.2.48
				if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x4d) && (configbuffer[3] == 0x59) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						configIpAddress[k] = configbuffer[j];
						k++;
					}
					configIpAddress[k - 1] = '\0';
					// //printf( "configIpAddress is [%s] \r\n", configIpAddress);
					//wificonfig_info.ipInfo.ip.addr = ipaddr_addr(configIpAddress);
					wificonfig_write.ipInfo.ip.addr = ipaddr_addr(configIpAddress);
					uart_write_bytes(uart_num,"OK", 2);
					//os_delay_us(5 * 1000);
                    received_command = 1;
				}
                // STCL - SET mfg test ssid and password in STA mode
                else if ((configbuffer[0] == 'S') && (configbuffer[1] == 'T') && (configbuffer[2] == 'C') && (configbuffer[3] == 'L'))
                {
                    uart_write_bytes(uart_num,"OK", 2);
                    set_mfg_test_clientmode();
                    received_command = 1;
                }
                // APST - SET mfg test ssid and password in AP mode
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'P') && (configbuffer[2] == 'S') && (configbuffer[3] == 'T'))
                {
                     uart_write_bytes(uart_num,"OK", 2);
                     ap_test_mode_flag = 1;
                     set_mfg_test_apmode();
                     received_command = 1;
                }
				// Set Gateway - ATGW 130.10.2.1
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x47) && (configbuffer[3] == 0x57) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						configgw[k] = configbuffer[j];
						k++;
					}
					configgw[k - 1] = '\0';
					//wificonfig_info.ipInfo.gw.addr = ipaddr_addr(configgw);
					wificonfig_write.ipInfo.gw.addr = ipaddr_addr(configgw);
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
                    //os_delay_us(5 * 1000);
					//printf( "Configgw is [%s] \r\n", configgw);
				}
				// Set NetMask - ATMK 255.255.255.0
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x4d) && (configbuffer[3] == 0x4b) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						confignetmask[k] = configbuffer[j];
						k++;
					}
					confignetmask[k - 1] = '\0';
					//wificonfig_info.ipInfo.netmask.addr = ipaddr_addr(confignetmask);
					wificonfig_write.ipInfo.netmask.addr = ipaddr_addr(confignetmask);
					uart_write_bytes(uart_num,"OK", 2);
					//os_delay_us(5 * 1000);
					////printf( "Confignetmask is [%s] \r\n", confignetmask);
				}
				// Set ATID - marveltest SSID 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x49) && (configbuffer[3] == 0x44) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						//wificonfig_info.ssid[k] = configbuffer[j];
						wificonfig_write.ssid[k] = configbuffer[j];
						k++;
					}
					//wificonfig_info.ssid[k - 1] = '\0';
					wificonfig_write.ssid[k - 1] = '\0';
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.ssid is [%s] \r\n", wificonfig_info.ssid);
				}
				// Set ATACID - AP SSID
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'A') && (configbuffer[3] == 'C') && (configbuffer[4] == 'I') && (configbuffer[5] == 'D') && (len > 6))
                {
                    for (j = 6, k = 0; j < len; j++)
                    {
                        //wificonfig_info.ssid[k] = configbuffer[j];
                        wificonfig_write.ap_ssid[k] = configbuffer[j];
                        k++;
                    }
                    //wificonfig_info.ssid[k - 1] = '\0';
                    wificonfig_write.ap_ssid[k - 1] = '\0';
					MFG_SSID_SET = 1;
                    uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
                    //os_delay_us(5 * 1000);
                    // //printf( "wificonfig_write.ssid is [%s] \r\n", wificonfig_write.ap_ssid);
                }
				// Set ATACPK - AP SSID
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'A') && (configbuffer[3] == 'C') && (configbuffer[4] == 'P') && (configbuffer[5] == 'K') && (len > 6))
                {
                    for (j = 6, k = 0; j < len; j++)
                    {
                        //wificonfig_info.ssid[k] = configbuffer[j];
                        wificonfig_write.ap_password[k] = configbuffer[j];
                        k++;
                    }
                    //wificonfig_info.ssid[k - 1] = '\0';
                    wificonfig_write.ap_password[k - 1] = '\0';
                    uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
                    //os_delay_us(5 * 1000);
                    ////printf( "wificonfig_write.ssid is [%s] \r\n", wificonfig_write.ap_password);
                }
				// Set ATPK - marveltest.exe password 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x50) && (configbuffer[3] == 0x4b) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						//wificonfig_info.password[k] = configbuffer[j];
						wificonfig_write.password[k] = configbuffer[j];
						k++;
					}
					//wificonfig_info.password[k - 1] = '\0';
					wificonfig_write.password[k - 1] = '\0';
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.password is [%s] \r\n", wificonfig_info.password);
				}
				// Set Server IP Address - ATSI 130.10.1.153
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x53) && (configbuffer[3] == 0x49) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						//wificonfig_info.SIpAddress[k] = configbuffer[j];
						wificonfig_write.SIpAddress[k] = configbuffer[j];
						k++;
					}
					//wificonfig_info.SIpAddress[k - 1] = '\0';
					wificonfig_write.SIpAddress[k - 1] = '\0';
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.SIpAddress is [%s] \r\n", wificonfig_info.SIpAddress);
				}
				// Set Server Port - ATSP 4096
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x53) && (configbuffer[3] == 0x50) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						SPort[k] = configbuffer[j];
						k++;
					}
					SPort[k - 1] = '\0';
					//wificonfig_info.SPort = atoi(SPort);
					wificonfig_write.SPort = atoi(SPort);
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.SPort is [%d] \r\n", wificonfig_info.SPort);
				}
				// Set User name for authentication - ATAU 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x41) && (configbuffer[3] == 0x55) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						//wificonfig_info.auth_username[k] = configbuffer[j];
						wificonfig_write.auth_username[k] = configbuffer[j];
						k++;
					}
					//wificonfig_info.auth_username[k - 1] = '\0';
					wificonfig_write.auth_username[k - 1] = '\0';
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.auth_username is [%s] \r\n", wificonfig_info.auth_username);
				}
				// Set Password for authentication - ATAP 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x41) && (configbuffer[3] == 0x50) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						//wificonfig_info.auth_password[k] = configbuffer[j];
						wificonfig_write.auth_password[k] = configbuffer[j];
						k++;
					}
					//wificonfig_info.auth_password[k - 1] = '\0';
					wificonfig_write.auth_password[k - 1] = '\0';
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.auth_password is [%s] \r\n", wificonfig_info.auth_password);
				}
				else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'C') && (configbuffer[3] == 'S') && (configbuffer[4] == 'Q') && (len == 6))
        		{
                   	memset(readbuffer, 0, 256);
                    sprintf(readbuffer, "%d", getStrength(2));
                    readbuffer[strlen(readbuffer)] = '\0';
               		uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
                    received_command = 1;
                      //printf( "My Signal Strength %s", readbuffer);
                }
                //ATCLST - Client connected 
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'C') && (configbuffer[3] == 'L') && (configbuffer[4] == 'S') && (configbuffer[5] == 'T') && (len == 7))
                {
                    if(client_connected)
                    {
                        uart_write_bytes(uart_num,"1", 1);
                    }
                    else
                    {
                        uart_write_bytes(uart_num,"0", 1);
                    }
                    received_command = 1;
                }
                /*
				// Set Baudrate - ATBD 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x42) && (configbuffer[3] == 0x44) && (len > 5))
				{
					for (j = 4, k = 0; j < len; j++)
					{
						baud_rate[k] = configbuffer[j];
						k++;
					}
					baud_rate[k - 1] = '\0';
					wificonfig_write.baudrate = atoi(baud_rate);
					uart_write_bytes(uart_num,"OK", 2);
					//os_delay_us(5 * 1000);
					////printf( "wificonfig_info.baudrate is [%d] \r\n", wificonfig_info.baudrate);
				}
                */
				// SET Debug enable/disable - ATDEB 
                else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x44) && (configbuffer[3] == 0x45) && (configbuffer[4] == 0x42) && (len > 5))
                {
                    for (j = 4, k = 0; j < len; j++)
                    {
                        baud_rate[k] = configbuffer[j];
                        k++;
                    }
                    baud_rate[k - 1] = '\0';
//                    wificonfig_write.debug_Enable = atoi(baud_rate);
                    uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
                    //os_delay_us(5 * 1000);
 //                   //printf( "Debug_Enable %d \r\n", wificonfig_write.debug_Enable);
                }
				// Write to Flash Command - ATWR  
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x57) && (configbuffer[3] == 0x52) && (len > 5))
				{
					////printf( "I Received Write to Flash command \r\n");
					//savewificonfiginfo(&wificonfig_info);
					savewificonfiginfo(&wificonfig_write);
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
				}
				// ATCN Command - ATCN - Apply changes and come out of configuration mode
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x43) && (configbuffer[3] == 0x4e) && (len == 5))
				{
					////printf( "I Received ATCN command \r\n");
					// Apply all the changes and move out of configuration mode
					uart_write_bytes(uart_num,"OK", 2);
					//os_delay_us(5 * 1000);
					flag = RCU_SERVER;
                    received_command = 1;
				}
				// Restart Command - ATFR - Module Reset 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x46) && (configbuffer[3] == 0x52) && (len == 5))
				{
					////printf( "I Received ATFR command,Sorry I am Restarting Now \r\n");
					uart_write_bytes(uart_num,"OK", 2);
                    pkt_count = 0;
					//os_delay_us(5 * 1000);
					//system_restart();
                    wificonfig_write.Reboot_count = 0; // Reset the Reboot count everytime a proper software reboot happens. As this will prevent in uncontrolled increase of the count.
                    savewificonfiginfo(&wificonfig_write);
                    received_command = 1;
					esp_restart();
				}
				// Reset Settings to Factory Default - ATRE 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x52) && (configbuffer[3] == 0x45) && (len == 5))
				{
					////printf( "I Received ATRE command,I am Resetting to Factory Default Now \r\n");
                    Factory_reset_flag = 1;
                     uart_write_bytes(uart_num,"atre", 4);
                    wificonfig_write.HBtime = 999;
                    savewificonfiginfo(&wificonfig_write);
					resetwificonfiginfo();
					uart_write_bytes(uart_num,"OK", 2);
                    received_command = 1;
					//os_delay_us(5 * 1000);
					//os_delay_us(50 * 1000);
					//system_restart();
					esp_restart();
				}
				// Get IP Address - ATMY 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x4d) && (configbuffer[3] == 0x59) && (len == 5))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.ip.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					////printf( "configIpAddress is [%s] \r\n", temp_addr);
					uart_write_bytes(uart_num,temp_addr, strlen(temp_addr));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get Gateway - ATGW 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x47) && (configbuffer[3] == 0x57) && (len == 5))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.gw.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					////printf( "configGateway Address is [%s] \r\n", temp_addr);
					uart_write_bytes(uart_num,temp_addr, strlen(temp_addr));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get NetMask - ATMK 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x4d) && (configbuffer[3] == 0x4b) && (len == 5))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.netmask.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					////printf( "confignetmask Address is [%s] \r\n", temp_addr);
					uart_write_bytes(uart_num,temp_addr, strlen(temp_addr));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get ATDEB 
                else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x44) && (configbuffer[3] == 0x45) && (configbuffer[4] == 0x42) && (len > 5))
                {
			        memset(readbuffer, 0, 256);
//                    sprintf(readbuffer, "%d", wificonfig_powerup.debug_Enable);
//                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
                    //os_delay_us(5 * 1000);
//                    //printf( "Debug_Enable %d \r\n", wificonfig_powerup.debug_Enable);
                }
				// Get ATID  
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x49) && (configbuffer[3] == 0x44) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.ssid, strlen(wificonfig_powerup.ssid));
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get ATACID
				else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'A') && (configbuffer[3] == 'C') && (configbuffer[4] == 'I') && (configbuffer[5] == 'D') && (len == 6))
				{
					memset(readbuffer, 0, 256);
                    strncpy(readbuffer, wificonfig_powerup.ap_ssid, strlen(wificonfig_powerup.ap_ssid));
                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
                    //os_delay_us(5 * 1000);
				}
				else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'A') && (configbuffer[3] == 'P') && (configbuffer[4] == 'P') && (configbuffer[5] == 'K') && (len == 6))
				{
			 	    memset(readbuffer, 0, 256);
                    strncpy(readbuffer, wificonfig_powerup.ap_password, strlen(wificonfig_powerup.ap_password));
                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
				}
				// Get ATPK  
				/*else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x50) && (configbuffer[3] == 0x4b) && (len == 5))
				{
				memset(readbuffer, 0, 256);
				strncpy(readbuffer, wificonfig_powerup.password, strlen(wificonfig_powerup.password));
				readbuffer[strlen(readbuffer)] = '\0';
				uart_write_bytes(uart_num,(uint8_t *)&readbuffer, strlen(readbuffer));
				//os_delay_us(5 * 1000);
				//flag = RCU_SERVER;
				}*/
				// Get Server IP Address - ATSI 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x53) && (configbuffer[3] == 0x49) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.SIpAddress, strlen(wificonfig_powerup.SIpAddress));
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get Server Port - ATSP 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x53) && (configbuffer[3] == 0x50) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "%d", wificonfig_powerup.SPort);
					readbuffer[strlen(readbuffer)] = '\0';
					////printf( "SPort is [%s] \r\n", readbuffer);
					uart_write_bytes(uart_num,readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get User name for authentication - ATAU 
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x41) && (configbuffer[3] == 0x55) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.auth_username, strlen(wificonfig_powerup.auth_username));
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// GET MAC address - ATMAC mac_str
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x4D) && (configbuffer[3] == 0x41) && (configbuffer[4] == 0x43))
                {
					//printf( "My Mac address %s", mac_str);
                    uart_write_bytes(uart_num,mac_str, strlen(mac_str));
                    //os_delay_us(5 * 1000);
                    //flag = RCU_SERVER;
                }
				// GET STA Connection status - ATSTA
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x53) && (configbuffer[3] == 0x54) && (configbuffer[4] == 0x41) && (len == 6))
				{
					memset(readbuffer, 0, 256);
                    sprintf(readbuffer, "%d", wifiParamCheck);
                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));

//					uart_write_bytes(uart_num, connection_status, strlen(connection_status));

				}
                // GET HVAC Connection status - ATHVAC
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'H') && (configbuffer[3] == 'V') && (configbuffer[4] == 'A')  && (configbuffer[5] == 'C') && (len == 7))
                {
                    memset(readbuffer, 0, 256);
                    sprintf(readbuffer, "%d", wificonfig_powerup.HVAC_type);
                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));

//                  uart_write_bytes(uart_num, connection_status, strlen(connection_status));

                }
/*
				else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'P') && (configbuffer[2] == 'E') && (configbuffer[3] == 'N') && (len == 5))
                {
		            wificonfig_write.wifi_mode = ESP_ACCESSPOINT_MODE; // Set to AP mode
                    savewificonfiginfo(&wificonfig_write);
		            uart_write_bytes(uart_num,"OK", 2);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
//                  //printf( "Set to AP mode : %d \n",  wificonfig_write.wifi_mode);
		            current_mode = ESP_ACCESSPOINT_MODE;
                }
                */
		        // SET STA mode - STEN
                else if ((configbuffer[0] == 'S') && (configbuffer[1] == 'T') && (configbuffer[2] == 'E') && (configbuffer[3] == 'N') && (len == 5))
		        {
                	wificonfig_write.wifi_mode = ESP_STATION_MODE; // Set to sta mode
                    savewificonfiginfo(&wificonfig_write);
	                uart_write_bytes(uart_num,"OK", 2);
        	        vTaskDelay(10 / portTICK_PERIOD_MS);
                    ////printf( " Set to STA mode : %d \n",  wificonfig_write.wifi_mode);
                	current_mode = ESP_STATION_MODE;
                }
				// Get Password for authentication - ATAP 
				/*
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x41) && (configbuffer[3] == 0x50) && (len == 5))
				{
				memset(readbuffer, 0, 256);
				strncpy(readbuffer, wificonfig_powerup.auth_password, strlen(wificonfig_powerup.auth_password));
				readbuffer[strlen(readbuffer)] = '\0';
				uart_write_bytes(uart_num,(uint8_t *)&readbuffer, strlen(readbuffer));
				//os_delay_us(5 * 1000);
				//flag = RCU_SERVER;
				}
				*/
				// Get Version Info - ATVR
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x56) && (configbuffer[3] == 0x52) && (len == 5))
				{
					//memset(readbuffer, 0, 256);
					//spi_flash_read(VERSION_ADDRESS, (uint32 *)(&readbuffer[0]), strlen(version_buffer));
					//readbuffer[strlen(readbuffer)] = '\0';
					////printf( "version is [%s] \r\n", readbuffer);
					////printf( "version_buffer is [%s] \r\n", version_buffer);
					//uart_write_bytes(uart_num,(uint8_t *)&readbuffer, strlen(readbuffer));
					uart_write_bytes(uart_num,version_buffer, strlen(version_buffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				else if((configbuffer[0] == '+') && (configbuffer[1] == '+') && (configbuffer[2] == '+') && (len == 3))
				{
					uart_write_bytes(uart_num,"OK", 2);
				}	
                //Ping GW IP - ATPING
                else if((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'P') && (configbuffer[3] == 'I') && (configbuffer[4] == 'N') && (configbuffer[5] == 'G'))
                {
                    ping_result();
                    if( wifiParamCheck == 4)
                    {
                        uart_write_bytes(uart_num,"PASS", 4);
                    }
                    else if (wifiParamCheck == 3)
                    {
                        uart_write_bytes(uart_num,"FAIL", 4);
                    }
                }

				// Get Accesspoint Connection Status - ATAI
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x41) && (configbuffer[3] == 0x49) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					if (1 == connection_status || 2 == connection_status)
					{
						t = 0;
						sprintf(readbuffer, "%d", t);
					}
					else if (3 == connection_status)
					{
						t = 1;
						sprintf(readbuffer, "%d", t);
					}
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
				// Get Channel number - ATCH
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x43) && (configbuffer[3] == 0x48) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					//channel_number = wifi_get_channel();
					sprintf(readbuffer, "%d", channel_number);
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
                /*
				// Get Channel number - ATBD
				else if ((configbuffer[0] == 0x41) && (configbuffer[1] == 0x54) && (configbuffer[2] == 0x42) && (configbuffer[3] == 0x44) && (len == 5))
				{
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "%d", wificonfig_powerup.baudrate);
					readbuffer[strlen(readbuffer)] = '\0';
					uart_write_bytes(uart_num,(char *)&readbuffer, strlen(readbuffer));
					//os_delay_us(5 * 1000);
					//flag = RCU_SERVER;
				}
                */
                // GET Time period for heartbeat ATHBT
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'H') && (configbuffer[3] == 'B') && (configbuffer[4] == 'T')&& (len == 6))
                {
                    memset(readbuffer, 0, 256);
                    sprintf(readbuffer, "%d", pkt_count);//wificonfig_powerup.HBtime);
                    readbuffer[strlen(readbuffer)] = '\0';
                    uart_write_bytes(uart_num, (char *)&readbuffer, strlen(readbuffer));
#ifdef Debug_Enable
                    printf("I sent RCU Time out for WIFI Heartbeat %d",wificonfig_powerup.HBtime);
#endif
                }
                // SET Time period for heartbeat ATHBT
                else if ((configbuffer[0] == 'A') && (configbuffer[1] == 'T') && (configbuffer[2] == 'H') && (configbuffer[3] == 'B') && (configbuffer[4] == 'T'))
                {
                    for (j = 4, k = 0; j < len; j++)
                    {
                        Rwt[k] = configbuffer[j];
                        k++;
                    }
                    Rwt[k - 1] = '\0';
                    wificonfig_write.HBtime = atoi(Rwt);
                    uart_write_bytes(uart_num, "OK", 2);
#ifdef Debug_Enable
                    printf("I received RCU Time out for WIFI Heart beat %d",wificonfig_write.HBtime);
#endif
                }
                else if (!received_command)
                {
        //            uart_write_bytes(uart_num,"Command Invalid",16); 
                }
                received_command = 0;
				memset(configbuffer, 0, 256);
				len = 0, k = 0;
			}
		}
		break;
		case RCU_UPGRADE_SERIAL:
		{
            //start_app_watchdog();
			//////printf( "I am starting Firmware Upgrade of the module\r\n");
			upgradelen = 0;
        if(rxBytes)
        {
            len = rxBytes;
            memcpy(upgradebuffer ,rx_data,rxBytes);
            memset(rx_data,0,strlen(rx_data));
        }
/*
        if(strlen(rx_data))
        {
            len = strlen(rx_data);
            strncpy(upgradebuffer,rx_data,strlen(rx_data));
            memset(rx_data,0,strlen(rx_data));
        }
*/
			if (upgradelen > 0)
			{
				//printf( "Received len [%d]\r\n", upgradelen);

				start_serial_upgradetimer();

				if ((upgradelen == 230) || (filesize < 230))
				{
					//printf( "Received data len [%x]\r\n", upgradelen);

					/*for (i = 0; i < upgradelen; i++)
					{
					//printf("Received data is [%x]\r\n", upgradebuffer[i]);
					}*/

					// CHECKBIN - check binary image which is running in the device
					if ((upgradebuffer[0] == 0x43) && (upgradebuffer[1] == 0x48) && (upgradebuffer[2] == 0x45) && (upgradebuffer[3] == 0x43) && (upgradebuffer[4] == 0x4B)
						&& (upgradebuffer[5] == 0x42) && (upgradebuffer[6] == 0x49) && (upgradebuffer[7] == 0x4E) && (m == 0))
					{
						//if (system_upgrade_userbin_check())
						if(1)
						{
							uart_write_bytes(uart_num,"ONE", 3);
							//os_delay_us(5 * 1000);
						}
						else
						{
							uart_write_bytes(uart_num,"ZERO", 4);
							//os_delay_us(5 * 1000);
						}
						m++;
						memset(upgradebuffer, 0, 1026);
						break;
					}
					// Filesize - Get File Size and Erase Flash - wait here for 5 seconds at least
					if ((upgradebuffer[0] == 0x46) && (upgradebuffer[1] == 0x49) && (upgradebuffer[2] == 0x4C) && (upgradebuffer[3] == 0x45) && (l == 0))
					{
						l++;
						sscanf(upgradebuffer + 10, "%ld", &file_length);
						filesize = file_length;
						//total_file_length = file_length;
						//printf( "Total file size is %ld\r\n", file_length);
						//printf( "Total file size is %ld\r\n", filesize);
						configured = esp_ota_get_boot_partition();
						running    = esp_ota_get_running_partition();

//						if (configured != running)
						{
//							//printf( "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x\r\n", configured->address, running->address);
//							//printf( "This can happen if either the OTA boot data or preferred boot image become corrupted somehow.\r\n)");
						}
//						//printf( "Running partition type %d subtype %d (offset 0x%08x)\r\n", running->type, running->subtype, running->address);
						update_partition = esp_ota_get_next_update_partition(NULL);
						//printf( "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
						err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);

						if (err != ESP_OK)
						{
							//printf( "esp_ota_begin failed (%s)\r\n", esp_err_to_name(err));
						}
						//printf( "esp_ota_begin succeeded\r\n");
						memset(upgradebuffer, 0, 1024+2);
						uart_write_bytes(uart_num,"OK", 2);
						//os_delay_us(5 * 1000);
						break;
					}

					// CHECKSTATUS - check Upgrade Status - 
					if ((upgradebuffer[0] == 0x43) && (upgradebuffer[1] == 0x48) && (upgradebuffer[2] == 0x45) && (upgradebuffer[3] == 0x43) && (upgradebuffer[4] == 0x4B)
						&& (upgradebuffer[5] == 0x53) && (upgradebuffer[6] == 0x54) && (upgradebuffer[7] == 0x41) && (upgradebuffer[8] == 0x54) && (upgradebuffer[9] == 0x55) && (upgradebuffer[10] == 0x53) && (n == 1))
					{
						if (serial_upgrade_status)
						{
							uart_write_bytes(uart_num,"PASS", 4);
							//os_delay_us(5 * 1000);
							//system_upgrade_flag_set(2);
							//system_upgrade_reboot();							
							err = esp_ota_set_boot_partition(update_partition);
							if (err != ESP_OK)
							{
								//printf( "esp_ota_set_boot_partition failed (%s)!\r\n", esp_err_to_name(err));
								//task_fatal_error();
							}
							//printf( "Prepare to restart system!\r\n");
							esp_restart();

						}
						else
						{
							uart_write_bytes(uart_num,"FAIL", 4);
							//os_delay_us(5 * 1000);
							flag = RCU_SERVER;
							n = 0;
						}
						memset(upgradebuffer, 0, 1024+2);
						break;
					}

					err = esp_ota_write(update_handle, (const void *)upgradebuffer, upgradelen);
					if (err != ESP_OK)
					{
						//task_fatal_error();
					}

					if (filesize > 230)
					{
						recv_filesize += upgradelen;
					}
					else
					{
						recv_filesize += filesize;
					}
					filesize -= upgradelen;
					//printf( "Length of data received from server is [filesize = %ld] [recv_filesize = %ld]  \r\n", filesize, recv_filesize);
					uart_write_bytes(uart_num,"OK", 2);
					//os_delay_us(5 * 1000);

					memset(upgradebuffer, 0, 1026);

					if (file_length == recv_filesize)
					{
						//printf("Upgrade finished\r\n");
						if (esp_ota_end(update_handle) != ESP_OK)
						{
							//printf( "esp_ota_end failed!");
							//task_fatal_error();
						}

						//if (upgrade_crc_check(system_get_fw_start_sec(), file_length) != 0)
						if(0)
						{
							//printf( "upgrade crc check failed !\n");
							//system_upgrade_flag_set(0);
							//system_upgrade_deinit();
							filesize = 0;
							recv_filesize = 0;
							l = 0, m = 0; n = 1;
							serial_upgrade_status = 0;
							memset(upgradebuffer, 0, 1026);
						}
						else
						{
							//printf( "upgrade crc check Passed! I am about to reboot\n");
							n = 1;
							serial_upgrade_status = 1;
						}
					}
				}
			}
			if (1 == stop_serial_upgrade)
			{
				stop_serial_upgrade = 0;
				flag = RCU_SERVER;
				//system_upgrade_flag_set(0);
				//system_upgrade_deinit();
				filesize = 0;
				recv_filesize = 0;
				l = 0, m = 0; n = 0;
				serial_upgrade_status = 0;
				memset(upgradebuffer, 0, 1026);
				//printf( "Stop Serial upgrade, timer expired, Move to normal communication \r\n");
			}
		}
		break;
		case RCU_UPGRADE_AUTHORISATION:
		{
            //start_app_watchdog();
//			printf( "I am starting Firmware Upgrade of the module Over The Air \r\n");
			if (upgradeconnfd == -1)
			{
				upgradeconnfd = accept(upgradesockfd, (struct sockaddr *)NULL, NULL);
			}
			else
			{
				upgradelen = read(upgradeconnfd, upgradebuffer, sizeof(upgradebuffer));

				if (upgradelen > 0)
				{
/*
					printf( "Received data len [%x]\r\n", upgradelen);

					for (i = 0; i < upgradelen; i++)
					{
					printf( "Received data is [%x]\t", upgradebuffer[i]);
					}
printf("\n");
*/
					start_OTA_upgradetimer();

					memset(ota_username, 0, SSID_SIZE);
					memset(ota_password, 0, PASSWORD_SIZE);
					i = 0; j = 0; k = 0;

					for (i = 0; i < upgradelen; i++)
					{
						if ((upgradebuffer[i] == 0x3A) && (upgradebuffer[i + 1] == 0x3A))
						{
							break;
						}
						else
						{
							ota_username[i] = upgradebuffer[i];
						}
					}

					ota_username[i] = '\0';

					i = i + 2;

					for (j = i; j < upgradelen; j++)
					{
						ota_password[k] = upgradebuffer[j];
						k++;
					}
					ota_password[k] = '\0';
					//printf( "[ota_username=%s] \r\n", ota_username);

					//printf( "[ota_password=%s] \r\n", ota_password);

					//printf( "[wificonfig_powerup.auth_username=%s] \r\n", wificonfig_powerup.auth_username);

					//printf( "[wificonfig_powerup.auth_password=%s] \r\n", wificonfig_powerup.auth_password);
					ret = strcmp(wificonfig_powerup.auth_username, ota_username);

					ret1 = strcmp(wificonfig_powerup.auth_password, ota_password);

					if ((ret == 0) && (ret1 == 0))
					{
						//if (system_upgrade_userbin_check())
						if(1)
						{
							//write(upgradeconnfd, "AUTHENTICATION PASSED ONE", strlen("AUTHENTICATION PASSED ONE"));
							memset(readbuffer, 0, 256);
							strncpy(readbuffer, "AUTHENTICATION PASSED ", strlen("AUTHENTICATION PASSED "));
							//strncpy(&readbuffer[strlen(readbuffer)], (const char*)version_buffer, strlen(version_buffer));
							readbuffer[strlen(readbuffer)] = '\0';
							write(upgradeconnfd, readbuffer, strlen(readbuffer));
							//printf( " AUTHENTICATION PASSED \r\n");
						}
						else
						{
							//write(upgradeconnfd, "AUTHENTICATION PASSED ZERO", strlen("AUTHENTICATION PASSED ZERO"));
							memset(readbuffer, 0, 256);
							strncpy(readbuffer, "AUTHENTICATION PASSED", strlen("AUTHENTICATION PASSED "));
							//strncpy(&readbuffer[strlen(readbuffer)], (const char*)version_buffer, strlen(version_buffer));
							readbuffer[strlen(readbuffer)] = '\0';
							write(upgradeconnfd, readbuffer, strlen(readbuffer));
							//printf( " AUTHENTICATION PASSED ZERO \r\n");
						}
						memset(upgradebuffer, 0, 1026);
						flag = RCU_UPGRADE_OTA;
						ret = -1;
						ret1 = -1;
						break;
					}
                    // Call the police or something.
					else
					{
						write(upgradeconnfd, "AUTHENTICATION FAILED", strlen("AUTHENTICATION FAILED"));
						memset(upgradebuffer, 0, 1026);
						flag = RCU_SERVER;
						//printf( " AUTHENTICATION FAILED \r\n");
						ret = -1;
						ret1 = -1;
						close(upgradeconnfd);
						upgradeconnfd = -1;
					}
				}
			}
			if (1 == stop_OTA_upgrade)
            {	    count = 0;
                    stop_OTA_upgrade = 0;
                    flag = RCU_SERVER;
            		connfd = -1;
                    upgradeconnfd = -1;
                    ota_upgrade_status = 0;
                    memset(upgradebuffer, 0, 1026);
                    //printf( "Stop OTA upgrade, timer expired, Move to normal communication \r\n");
			        //esp_restart();
            }

		}
		break;
		case RCU_UPGRADE_OTA:
		{
            // If this code works, it was written by Vikram vel. If not, I don't know
            // who wrote it
			start_connection_timer();

            // Stop Heartbeats and suspend monitor tasks during OTA process
            if (HBT_timer_flag)
            {
                HBT_timer_flag = 0;
                ESP_ERROR_CHECK(esp_timer_stop(heartbeat_timer));
                ESP_ERROR_CHECK(esp_timer_delete(heartbeat_timer));
            }
		    upgradelen = recv(upgradeconnfd, upgradebuffer, sizeof(upgradebuffer), 0);
            vTaskSuspend(Monitor_TaskHandle);
            //start_app_watchdog();
            /*
            for (i = 0; i < upgradelen; i++)
            {
                 printf( "Received data is [%x]\t", upgradebuffer[i]);
            }
            printf("\n");
            */
			if (upgradelen > 0 ) 
			{
				prev_crc = present_crc;
			    //printf( " upgrade [0] %x upgrade [1] %x", upgradebuffer[0],upgradebuffer[1]);
				unsigned int tmp = (unsigned)upgradebuffer[1] << 8 | (unsigned)upgradebuffer[0];
				short number = tmp;
				present_crc = number;
                ////printf( " Temp: %d",tmp);
				//printf( "present_crc : %d prev_crc: %d\n ", present_crc, prev_crc);
				start_OTA_upgradetimer();
				if (upgradelen <= 1026)
				{

					if ((upgradebuffer[0] == 0x46) && (upgradebuffer[1] == 0x49) && (upgradebuffer[2] == 0x4C) && (upgradebuffer[3] == 0x45) && (l == 0))
					{
						sscanf(upgradebuffer + 10, "%ld", &file_length);
						filesize = file_length;
						write(upgradeconnfd, "OK", 2);
//						//printf( "Total file size is %ld\r\n", file_length);
						end_status=0;

						//system_upgrade_init();
						//system_upgrade_flag_set(1); // 0 - idle 1 - start 2- finish
						//system_upgrade_erase_flash();
						configured = esp_ota_get_boot_partition();

						running = esp_ota_get_running_partition();
						if (configured != running)
						{
							//printf( "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x\r\n", configured->address, running->address);
							//printf( "This can happen if either the OTA boot data or preferred boot image become corrupted somehow.\r\n)");
						}
						//printf( "Running partition type %d subtype %d (offset 0x%08x)\r\n", running->type, running->subtype, running->address);
						update_partition = esp_ota_get_next_update_partition(NULL);
						//printf( "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
						err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);

						if (err != ESP_OK)
						{
                            /**
                            * For the brave souls who get this far: You are the chosen ones,
                            * the valiant knights of programming who toil away, without rest,
                            * fixing our most awful code. To you, true saviors, kings of men,
                            * I say this: never gonna give you up, never gonna let you down,
                            * never gonna run around and desert you. Never gonna make you cry,
                            * never gonna say goodbye. Never gonna tell a lie and hurt you.
                            */
							//printf( "esp_ota_begin failed (%s)\r\n", esp_err_to_name(err));
						}
						//printf( "esp_ota_begin succeeded\r\n");
						count = 0;
						memset(upgradebuffer, 0, 1026);
						l++;
						present_crc = 0;
						break;
					}
					// CHECKSTATUS - check Upgrade Status - 
					else if ((upgradebuffer[0] == 0x43) && (upgradebuffer[1] == 0x48) && (upgradebuffer[2] == 0x45) && (upgradebuffer[3] == 0x43) && (upgradebuffer[4] == 0x4B)
						&& (upgradebuffer[5] == 0x53) && (upgradebuffer[6] == 0x54) && (upgradebuffer[7] == 0x41) && (upgradebuffer[8] == 0x54) && (upgradebuffer[9] == 0x55) && (upgradebuffer[10] == 0x53))
					{
						for (i = 0; i < upgradelen; i++)
	                    {
     				        //printf( "Received data is [%x]\t", upgradebuffer[i]);
                	    }
						count = 0;
						if (ota_upgrade_status)
						{
							write(upgradeconnfd, "PASS", 4);

							err = esp_ota_set_boot_partition(update_partition);
							if (err != ESP_OK)
							{
								 //printf( "esp_ota_set_boot_partition failed (%s)!\r\n", esp_err_to_name(err));
							}
							//printf( "Prepare to restart system!\r\n");
							esp_restart();

						}
						else
						{
							write(upgradeconnfd, "FAIL", 4);
							flag = RCU_SERVER;
							n = 0;
						}
						memset(upgradebuffer, 0, 1026);
						close(serversockfd);
						serversockfd = -1;
						break;
					}
					else if ((upgradebuffer[0] == 0x45) && (upgradebuffer[1] == 0x4E) && (upgradebuffer[2] == 0x44))
					{
#ifdef Debug_Enable
						for (i = 0; i < upgradelen; i++)
                        {
                            //printf("Received data is [%x]\t", upgradebuffer[i]);
                        }
#endif
						write(upgradeconnfd, "OK", 2);
						end_status=1;
						err = (esp_ota_end(update_handle));
                        if (err != ESP_OK)
                        {
                            ////printf( " esp_ota_end failed!");
                            break;
                        }
						ota_upgrade_status = 1;
					}
					else if (((prev_crc != present_crc) && (present_crc > 0) ))
					{
						count ++;
						if(!end_status )//&& file_length!= recv_filesize)
						{
					    	err = esp_ota_write(update_handle, &upgradebuffer[2], upgradelen-2);

				        if (err != ESP_OK) 
						{
                            // should never happen
							//printf( "######## ERROR HEX WRITE [%d] %s",count, esp_err_to_name(err));
							break;
            			}
						else if (err == ESP_OK)
						{
							//printf( "Written To FLASH Packet Number [%d] - Count [%d]", present_crc, count);
                        	
						write(upgradeconnfd, "OK", 2);
                        } 
						}
						////printf( "filesize: %ld,  recv_filesize: %ld, file_length: %ld, upgradelen: %d", filesize, recv_filesize, file_length, upgradelen);
						if (filesize > 1026)
						{
							recv_filesize += upgradelen;
						}
						else
						{
							recv_filesize += filesize;
						}
						filesize -= upgradelen;
						////printf( " count [%d] upgradelen - %d", count, upgradelen);
						////printf( " [%d] Length of data received from server is [filesize = %ld] [recv_filesize = %ld]  \r\n", count, filesize, recv_filesize);
						//strncpy(prev_upgradebuffer,upgradebuffer,sizeof(upgradebuffer));
						memset(upgradebuffer, 0, 1026);
					}
					else 
					{
						//printf( " Retry ");
					}
				}
				upgradelen = 0;
			}
			if (1 == stop_OTA_upgrade)
			{
                vTaskResume(Monitor_TaskHandle);
				count = 0;
				stop_OTA_upgrade = 0;
				flag = RCU_SERVER;
				connfd = -1;
				upgradeconnfd = -1;
				//system_upgrade_flag_set(0);
				//system_upgrade_deinit();
				filesize = 0;
				recv_filesize = 0;
				l = 0, m = 0; n = 0;
				ota_upgrade_status = 0;
				memset(upgradebuffer, 0, 1026);
				//printf( "Stop OTA upgrade, timer expired, Move to normal communication \r\n");
				//esp_restart();
			}
		}
		break;
		
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

void monitor_task(void *pvParameters)
{
    int while_loop_count = 1;
    char readbuffer[256];
    ping_result();
    while(1)
    {
        vTaskDelay(1 / portTICK_RATE_MS);
        if(while_loop_count % 100000 != 0)
        {
            while_loop_count++;
            continue;
        }
        //ping_result();
        rssi = getStrength(1) * -1;
        Signal_Strength = (short int) rssi;
        //printf("\n Count - %d", Error_count);
        while_loop_count ++;
    }
}

void startwificonfig()
{

	wifi_config_t     wifi_config;

	if (0 == getwificonfiginfo(&wificonfig_powerup)) 
	{
		memset(&wificonfig_write, 0, sizeof(wificonfig_write));
		wificonfig_write = wificonfig_powerup;
		current_mode = wificonfig_powerup.wifi_mode;
        //printf("\n MOde after power up %d", current_mode);
        //wificonfig_write.wifi_mode = wificonfig_powerup.wifi_mode;
        if(wificonfig_powerup.ipInfo.ip.addr <= 0)
        {
             wifiParamCheck= 0;
        }
/*
		//printf( "value = %d, Int value = %d\n", wificonfig_powerup.hvacCfg.HVAC_type, wificonfig_powerup.hvacCfg.Fan_speed);
		//printf( "SSID is %s %d\r\n", wificonfig_powerup.ssid, strlen(wificonfig_powerup.ssid));
		//printf( "PWD is %s %d\n\r", wificonfig_powerup.password, strlen(wificonfig_powerup.password));

		//printf( "AP SSID is %s len %d \r\n",wificonfig_powerup.ap_ssid, strlen(wificonfig_powerup.ap_ssid));
		//printf( "AP PWD is %s len %d \r\n",wificonfig_powerup.ap_password, strlen(wificonfig_powerup.ap_password));

		//printf( "ServerIpAddress is [%s] \r\n", wificonfig_powerup.SIpAddress);
		//printf( "ServerPort is [%d] \r\n", wificonfig_powerup.SPort);
		//printf( "Current Mode is [%d] \r \n", current_mode);
		//printf( "Authentication Username is %s \r\n", wificonfig_powerup.auth_username);
		//printf( "Authentication PWD is %s \n\r", wificonfig_powerup.auth_password);
		//printf( "Baudrate is [%d] \r\n", wificonfig_powerup.baudrate);
		//printf( "[wificonfig_powerup.ipInfo.ip.addr=%d \r\n", wificonfig_powerup.ipInfo.ip.addr);
		//printf( "[wificonfig_powerup.ipInfo.gw.addr =%d \r\n", wificonfig_powerup.ipInfo.gw.addr);
		//printf( "[wificonfig_powerup.ipInfo.netmask.addr =%d \r\n", wificonfig_powerup.ipInfo.netmask.addr);
*/

        /* The following code comes handy when you want to hard code the WIFI details you want during
         * debugging. and reference to the PAST when everything was working so well!
         */
        /*
		memset(wificonfig_powerup.ssid, 0, SSID_SIZE);
		memset(wificonfig_powerup.password, 0, PASSWORD_SIZE);

		strncpy(wificonfig_powerup.ssid, "INTEREL_WORKSHOP", strlen("INTEREL_WORKSHOP"));
		wificonfig_powerup.ssid[strlen(wificonfig_powerup.ssid)] = '\0';
		wificonfig_powerup.ssid[SSID_SIZE - 1] = '\0';

		strncpy(wificonfig_powerup.password, "#connectBsmart2017$", strlen("#connectBsmart2017$"));
		wificonfig_powerup.password[strlen(wificonfig_powerup.password)] = '\0';
		wificonfig_powerup.password[PASSWORD_SIZE - 1] = '\0';

		strncpy(wificonfig_powerup.SIpAddress, "130.10.1.156", strlen("130.10.1.156"));
		wificonfig_powerup.SIpAddress[strlen(wificonfig_powerup.SIpAddress)] = '\0';

		wificonfig_powerup.SPort = 4096;

		//change is required here for managing IP address from flash

		inet_pton(AF_INET, DEVICE_IP, &wificonfig_powerup.ipInfo.ip);
		inet_pton(AF_INET, DEVICE_GW, &wificonfig_powerup.ipInfo.gw);
		inet_pton(AF_INET, DEVICE_NETMASK, &wificonfig_powerup.ipInfo.netmask);

//		//printf( "[wificonfig_powerup.ipInfo.ip.addr=%d \r\n", wificonfig_powerup.ipInfo.ip.addr);
//		//printf( "[wificonfig_powerup.ipInfo.gw.addr =%d \r\n", wificonfig_powerup.ipInfo.gw.addr);
//		//printf( "[wificonfig_powerup.ipInfo.netmask.addr =%d \r\n", wificonfig_powerup.ipInfo.netmask.addr);
		wificonfig_write = wificonfig_powerup;
		
		tcpip_adapter_init();

		tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);

		tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &wificonfig_powerup.ipInfo);

		wifi_event_group = xEventGroupCreate();

		ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

		ESP_ERROR_CHECK(esp_wifi_init(&cfg));

		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

		memset(&wifi_config, 0, sizeof(wifi_config_t));

		wifi_config.sta.bssid_set = 0;
		memset((char *)wifi_config.sta.ssid, 0, 32);
		memset((char *)wifi_config.sta.password, 0, 64);

		strncpy((char *)wifi_config.sta.ssid, wificonfig_powerup.ssid, strlen(wificonfig_powerup.ssid));
		wifi_config.sta.ssid[strlen((char *)wifi_config.sta.ssid)] = '\0';

		strncpy((char *)wifi_config.sta.password, wificonfig_powerup.password, strlen(wificonfig_powerup.password));
		wifi_config.sta.password[strlen((char *)wifi_config.sta.password)] = '\0';

		strncpy(ServerIpAddress, wificonfig_powerup.SIpAddress, strlen(wificonfig_powerup.SIpAddress));
		ServerIpAddress[strlen(ServerIpAddress)] = '\0';

		ServerPort = wificonfig_powerup.SPort;

		//printf( "Successfully got WIFI config data from Flash \r\n");
		//printf( "SSID is %s \r\n", wifi_config.sta.ssid);
		
		//printf( "PWD is %s \n\r", wifi_config.sta.password);
		//printf( "ServerIpAddress is [%s] \r\n", ServerIpAddress);
		//printf( "ServerPort is [%d] \r\n", ServerPort);
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_start());*/
	}
	else 
	{
		resetwificonfiginfo();
	}
}

void intwifi()
{
    esp_err_t err;
    wifi_config_t     wifi_config;
    
    tcpip_adapter_init();
    
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
    
    err = tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &wificonfig_powerup.ipInfo);
    if( strcmp(wificonfig_write.ssid, "None") == 0)
    {   
           wifiParamCheck= 0;
//         uart_write_bytes(uart_num,"E-0",3);
//         uart_write_bytes(uart_num,"0", 1);
    }

    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    
    wifi_config.sta.bssid_set = 0;
    memset((char *)wifi_config.sta.ssid, 0, 32);
    memset((char *)wifi_config.sta.password, 0, 64);
    strncpy((char *)wifi_config.sta.ssid, wificonfig_powerup.ssid, strlen(wificonfig_powerup.ssid));
    wifi_config.sta.ssid[strlen((char *)wifi_config.sta.ssid)] = '\0';

    strncpy((char *)wifi_config.sta.password, wificonfig_powerup.password, strlen(wificonfig_powerup.password));
    wifi_config.sta.password[strlen((char *)wifi_config.sta.password)] = '\0';

    strncpy(ServerIpAddress, wificonfig_powerup.SIpAddress, strlen(wificonfig_powerup.SIpAddress));
    ServerIpAddress[strlen(ServerIpAddress)] = '\0';

    ServerPort = wificonfig_powerup.SPort;
//  //printf( "SSID %s, SSID Len %d, Password %s, Password len %d",  wifi_config.sta.ssid, strlen(wificonfig_powerup.ssid), wifi_config.sta.password, strlen(wificonfig_powerup.password));
/*
    //printf( "Successfully got WIFI config data from Flash \r\n");
    //printf( "SSID is %s \r\n", wifi_config.sta.ssid);

    //printf( "PWD is %s \n\r", wifi_config.sta.password);
    //printf( "ServerIpAddress is [%s] \r\n", ServerIpAddress);
    //printf( "ServerPort is [%d] \r\n", ServerPort);
    */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    if(strlen(wificonfig_powerup.hostname))
    {
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, wificonfig_powerup.hostname));
    }
//    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config));
//    int ch = wifi_config.sta.channel;
//    printf("\n - %d", ch);
//    printf("\n");
//    printf( " ALL GOOD STARTED WIFI");
}	

void set_mfg_test_clientmode(void)
{
    memset(wificonfig_write.ssid, 0, SSID_SIZE);
    memset(wificonfig_write.password, 0, PASSWORD_SIZE);

    strncpy(wificonfig_write.ssid, "EOS_TESTING ",strlen("EOS_TESTING "));
    wificonfig_write.ssid[strlen(wificonfig_write.ssid)-1] = '\0';
    //wificonfig_write.ssid[SSID_SIZE - 1] = '\0';
    

    strncpy(wificonfig_write.password, "qwerty123#$ ",strlen("qwerty123#$ "));
    wificonfig_write.password[strlen(wificonfig_write.password)-1] = '\0';
    //wificonfig_write.password[PASSWORD_SIZE - 1] = '\0';
    wificonfig_write.wifi_mode = ESP_STATION_MODE;
    current_mode = ESP_STATION_MODE;
    savewificonfiginfo(&wificonfig_write);
    esp_restart();
}

int  getwificonfiginfo(wificonfig_info_t *pConnectionInfo)
{
	nvs_handle handle;
	size_t size;
	esp_err_t err;
	uint32_t version;
	err = nvs_open(BOOTWIFI_NAMESPACE, NVS_READONLY, &handle);
	if (err != 0)
	{
	//	printf( "nvs_open: %x", err);
		return -1;
	}

	// Get the version that the data was saved against.
	err = nvs_get_u32(handle, KEY_VERSION, &version);
	if (err != ESP_OK)
	{
	//	printf( "No version record found (%d).", err);
		nvs_close(handle);
		return -1;
	}

	// Check the versions match
	if ((version & 0xff00) != (g_version & 0xff00))
	{
	//	printf( "Incompatible versions ... current is %x, found is %x", version, g_version);
		nvs_close(handle);
		return -1;
	}

	size = sizeof(wificonfig_info_t);
	err = nvs_get_blob(handle, KEY_CONNECTION_INFO, pConnectionInfo, &size);
	if (err != ESP_OK)
	{
	//	printf( "No connection record found (%d).", err);
		nvs_close(handle);
		return -1;
	}

	// Cleanup
	nvs_close(handle);

	// Do a sanity check on the SSID
	/*if (strlen(pConnectionInfo->ssid) == 0)
	{
		printf( "NULL ssid detected");
		return -1;
	}
    */
	return 0;
}

void savewificonfiginfo(wificonfig_info_t *pConnectionInfo)
{
	nvs_handle handle;

    ESP_ERROR_CHECK(nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle));
	ESP_ERROR_CHECK(nvs_set_blob(handle, KEY_CONNECTION_INFO, pConnectionInfo, sizeof(wificonfig_info_t)));
	ESP_ERROR_CHECK(nvs_set_u32(handle, KEY_VERSION, g_version));
	ESP_ERROR_CHECK(nvs_commit(handle));
	nvs_close(handle);
}
void erasewificonfiginfo(void)
{
    nvs_handle handle;
    ESP_ERROR_CHECK(nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle));
    //ESP_ERROR_CHECK(nvs_set_blob(handle, KEY_CONNECTION_INFO, pConnectionInfo, sizeof(wificonfig_info_t)));
    //ESP_ERROR_CHECK(nvs_set_u32(handle, KEY_VERSION, g_version));
    ESP_ERROR_CHECK(nvs_erase_all(handle));
    nvs_close(handle);
}
/*
void startuart()
{
	uart_config_t uart_config =
	{
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		//.rx_flow_ctrl_thresh = 122,
	};

	if (7 == wificonfig_powerup.baudrate)
	{
		uart_config.baud_rate = 115200;
	}
	else if (3 == wificonfig_powerup.baudrate)
	{
		uart_config.baud_rate = 9600;
	}
	else
	{
		uart_config.baud_rate = 115200;
	}

	//Set UART parameters
	uart_param_config(uart_num, &uart_config);

	// Initialise UART
//	uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
        ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	uart_driver_install(uart_num, 2048, 0, 0, NULL, 0);

	// release the pre registered UART handler/subroutine
	uart_isr_free(uart_num);

	// register new UART subroutine
	uart_isr_register(uart_num, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);

	// enable RX interrupt
	uart_enable_rx_intr(uart_num);
	
}
*/
unsigned char checksum (unsigned char *ptr, size_t sz) {
    unsigned char chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}


// TODO - Comment this function
int  check_TX_FIFO_Empty_complete(uint16_t Length)
{
	uint16_t tx_fifo_len = 0;

	tx_fifo_empty = 0;

	start_TX_FIFO_Empty_timer(Length * 300);

	while (1)
	{
		//tx_fifo_len = ((READ_PERI_REG(UART_STATUS(0)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT);

		if ((0 == tx_fifo_len) || (1 == tx_fifo_empty))
		{
			if (7 == wificonfig_powerup.baudrate)
			{
				////os_delay_us((70));//BIT_RATE_115200;   need to replace this
			}
			else if (3 == wificonfig_powerup.baudrate)
			{
				////os_delay_us((1100));// BIT_RATE_9600; need to replace this
			}
			break;
		}
	}
return 1;
}
/*
void wifi_deinit()
{
	esp_wifi_stop(); // stop station  AP  modes and free their control blocks
	sys_delay_ms(1000);
	esp_wifi_deinit(); //remove wifi driver from the application
	sys_delay_ms(1000);
}
*/

// The world is a happy place.
// initialize ESP as an access point//
void wifi_init_softap()
{
    char *ssid_mac = (char*) malloc(8);

    tcpip_adapter_init();
    // set static IP to the ESP with a value of 1.1.1.1
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP)); //stop DHCP server
    tcpip_adapter_ip_info_t ip_info;

    memset(&ip_info, 0, sizeof(ip_info));

    IP4_ADDR(&ip_info.ip,192,168,1,10); // ip address
    IP4_ADDR(&ip_info.gw,192,168,1,1); //gateway
    IP4_ADDR(&ip_info.netmask,255,255,255,0); //subnet mask
 
    // //printf( "set ip ret: %d\n", tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));
    
    //tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP); //restart DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP)); 

    wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config;
    memset((char *)wifi_config.ap.ssid, 0, 32);
    memset((char *)wifi_config.ap.password, 0, 64);

    strncpy((char *)wifi_config.ap.ssid, wificonfig_powerup.ap_ssid, strlen(wificonfig_powerup.ap_ssid));
    wifi_config.ap.ssid[strlen((char*)wifi_config.ap.ssid)] = '\0';

    ////printf( "wificonfig_powerup.ap_password %s len %d",wificonfig_powerup.ap_password, strlen(wificonfig_powerup.ap_password));
    strcpy((char *)wifi_config.ap.password, wificonfig_powerup.ap_password);

    wifi_config.ap.password[strlen((char*)wifi_config.ap.password)] = '\0';

    wifi_config.ap.ssid_len = strlen(wificonfig_powerup.ap_ssid);
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(wificonfig_powerup.ap_password) == 0) 
    {
	//printf( "Seting in open mode");
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());


//     //printf( "wifi_init_softap finished.SSID: %spassword: %slen of ssid %d & pwd %d",(char *)wifi_config.ap.ssid, (char *)wifi_config.ap.password,strlen((char *)wifi_config.ap.ssid),strlen((char *)wifi_config.ap.password));

   // char* test_str = "wifi init softap finished.\n";
   // uart_write_bytes(UART_NUM_2, (const char*)test_str, strlen(test_str));
}

void set_mfg_test_apmode(void)
{
    //printf("\n Set Ap Mode");
    memset(wificonfig_write.ap_ssid, 0, SSID_SIZE);
    memset(wificonfig_write.ap_password, 0, PASSWORD_SIZE);

    strncpy(wificonfig_write.ap_ssid, "EOS_TESTING ", strlen("EOS_TESTING "));
    wificonfig_write.ap_ssid[strlen(wificonfig_write.ap_ssid)-1] = '\0';
    //wificonfig_write.ap_ssid[SSID_SIZE - 1] = '\0';

    strncpy(wificonfig_write.ap_password, "qwerty123#$ ", strlen("qwerty123#$ "));
    wificonfig_write.ap_password[strlen(wificonfig_write.ap_password)-1] = '\0';
    //wificonfig_write.ap_password[PASSWORD_SIZE - 1] = '\0';

    wificonfig_write.wifi_mode = ESP_ACCESSPOINT_MODE;
    current_mode = ESP_ACCESSPOINT_MODE;
    savewificonfiginfo(&wificonfig_write);
    esp_restart();
}

void ota_config( char *upgradebuf, int upgradelen)
{
    char configIpAddress[16];
	char configgw[16];
	char confignetmask[16];
	char SPort[10];
	char DPort[10];
	char baud_rate[10];
	char ota_username[SSID_SIZE];
	char ota_password[PASSWORD_SIZE];
	int y = 0, z = 0, j = 0, k = 0, i = 0, t= 0;
	int flag = 0;
	char Rwt[10];                          // RCU WiFi Timeout
	char *temp_addr;
	char temp_ch[BUFSIZE];;
	char readbuffer[256];
	int channel_number =0;
	memset(configIpAddress,0,16);
#ifdef Debug_Enable
	printf("\n Inside OTA CONFIG");
	printf("\n length of upg %d",upgradelen);
#endif
	if (upgradelen > 0)
	{
        /*
                printf("\n OTA CONFIG DATA RECEIVED *****  [%d] \r\n", upgradelen);
                for (i = 0; i < upgradelen; i++)
                {
                        printf("%x \t", upgradebuf[i]);
                }
                printf("\n");
                */
				// Set IP Address - ATMY 130.10.2.48
				if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x4d) && (upgradebuf[11] == 0x59))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						configIpAddress[k] = upgradebuf[j];
						k++;
					}
					configIpAddress[k - 1] = '\0';
					wificonfig_write.ipInfo.ip.addr = ipaddr_addr(configIpAddress);
					write(connfd, "+++OK++", 7);
                    //printf("pConnectionInfo %s\n",inet_ntoa(wificonfig_write.ipInfo.ip.addr));
					//printf("configgw is [%s] \r\n", configIpAddress);
				}// Set Gateway - ATGW 130.10.2.1
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x47) && (upgradebuf[11] == 0x57))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						configgw[k] = upgradebuf[j];
						k++;
					}
					configgw[k - 1] = '\0';
					//wificonfig_info.ipInfo.gw.addr = ipaddr_addr(configgw);
					wificonfig_write.ipInfo.gw.addr = ipaddr_addr(configgw);
					write(connfd, "+++OK++", 7);
//printf("2 pConnectionInfo %s\n",inet_ntoa(wificonfig_write.ipInfo.ip.addr));
#ifdef Debug_Enable
					printf("configgw is [%s] \r\n", configgw);
#endif
				}// Set NetMask - ATMK 255.255.255.0
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x4d) && (upgradebuf[11] == 0x4b) && ( (upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						confignetmask[k] = upgradebuf[j];
						k++;
					}
					confignetmask[k - 1] = '\0';
					//wificonfig_info.ipInfo.netmask.addr = ipaddr_addr(confignetmask);
					wificonfig_write.ipInfo.netmask.addr = ipaddr_addr(confignetmask);
					write(connfd, "+++OK++", 7);
//printf("3 pConnectionInfo %s\n",inet_ntoa(wificonfig_write.ipInfo.ip.addr));
#ifdef Debug_Enable
					printf("confignetmask is [%s] \r\n", confignetmask);
#endif
				}// Set ATID - marveltest SSID 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x49) && (upgradebuf[11] == 0x44) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						//wificonfig_info.ssid[k] = upgradebuf[j];
						wificonfig_write.ssid[k] = upgradebuf[j];
						k++;
					}
					//wificonfig_info.ssid[k - 1] = '\0';
					wificonfig_write.ssid[k - 1] = '\0';
					write(connfd, "+++OK++", 7);
#ifdef Debug_Enable
					//printf("wificonfig_info.ssid is [%s] \r\n", wificonfig_info.ssid);
#endif
				}
				// Set ATPK - marveltest.exe password 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x50) && (upgradebuf[11] == 0x4b) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						//wificonfig_info.password[k] = upgradebuf[j];
						wificonfig_write.password[k]  = upgradebuf[j];
						k++;
					}
					//wificonfig_info.password[k - 1] = '\0';
					wificonfig_write.password[k - 1] = '\0';
					write(connfd, "+++OK++", 7);
#ifdef Debug_Enable
					//printf("wificonfig_info.password is [%s] \r\n", wificonfig_info.password);
#endif
				}
				// Set Server IP Address - ATSI 130.10.1.153
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x53) && (upgradebuf[11] == 0x49) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						//wificonfig_info.SIpAddress[k] = upgradebuf[j];
						wificonfig_write.SIpAddress[k] = upgradebuf[j];
						k++;
					}
					//wificonfig_info.SIpAddress[k - 1] = '\0';
					wificonfig_write.SIpAddress[k - 1] = '\0';
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.SIpAddress is [%s] \r\n", wificonfig_info.SIpAddress);
				}
				// Set Server Port - ATSP 4096
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x53) && (upgradebuf[11] == 0x50) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						SPort[k] = upgradebuf[j];
						k++;
					}
					SPort[k - 1] = '\0';
					//wificonfig_info.SPort = atoi(SPort);
					wificonfig_write.SPort = atoi(SPort);
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.SPort is [%d] atoi %d \r\n", wificonfig_write.SPort,atoi(SPort));
				}
                /*
               	else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x44) && (upgradebuf[10] == 0x50) && ((upgradelen-9) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						DPort[k] = upgradebuf[j];
						k++;
					}
					DPort[k - 1] = '\0';
					//wificonfig_info.DPort = atoi(DPort);
					wificonfig_write.DPort = atoi(DPort);
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.DPort is [%d] atoi %d \r\n", wificonfig_write.SPort,atoi(SPort));
				}
                */
				// Set User name for authentication - ATAU 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x41) && (upgradebuf[11] == 0x55) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						//wificonfig_info.auth_username[k] = upgradebuf[j];
						wificonfig_write.auth_username[k] = upgradebuf[j];
						k++;
					}
					//wificonfig_info.auth_username[k - 1] = '\0';
					wificonfig_write.auth_username[k - 1] = '\0';
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.auth_username is [%s] \r\n", wificonfig_info.auth_username);
				}
				// Set Password for authentication - ATAP 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x41) && (upgradebuf[11] == 0x50) && ((upgradelen-10) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						//wificonfig_info.auth_password[k] = upgradebuf[j];
						wificonfig_write.auth_password[k] = upgradebuf[j];
						k++;
					}
					//wificonfig_info.auth_password[k - 1] = '\0';
					wificonfig_write.auth_password[k - 1] = '\0';
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.auth_password is [%s] \r\n", wificonfig_info.auth_password);
				}
                /*
				// Set Baudrate - ATBD 
				else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x42) && (upgradebuf[10] == 0x44) && ((upgradelen-9) > 5))
				{
					for (j = 11, k = 0; j <upgradelen-2; j++)
					{
						baud_rate[k] = upgradebuf[j];
						k++;
					}
					baud_rate[k - 1] = '\0';
					wificonfig_write.baudrate = atoi(baud_rate);
					write(connfd, "+++OK++", 7);
					//printf("wificonfig_info.baudrate is [%d] \r\n", wificonfig_info.baudrate);
				}
                */
				// Write to Flash Command - ATWR  
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x57) && (upgradebuf[11] == 0x52) && (upgradelen == 15))
				{
//printf("pConnectionInfo %s\n",inet_ntoa(wificonfig_write.ipInfo.ip.addr));
//					printf("I Received Write to Flash command \r\n");
					//savewificonfiginfo(&wificonfig_info);
                    savewificonfiginfo(&wificonfig_write);
//					savewificonfiginfo(&wificonfig_write, &ext_config_write);
					write(connfd, "+++OK++", 7);
				}
				// ATRT - SET
/*
                                else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x52) && (upgradebuf[10] == 0x54) && ((upgradelen-9) > 5))
                                {
                                        for (j = 11, k = 0; j < upgradelen-2; j++)
                                        {
                                                Rwt[k] = upgradebuf[j];
                                                k++;
                                        }
                                        Rwt[k - 1] = '\0';
                                        wificonfig_write.RCU_WiFi_Timeout = atoi(Rwt);
					write(connfd, "+++OK++", 7);
#ifdef Debug_Enable
                                        printf("\n I received RCU Time out for WIFI %d",wificonfig_write.RCU_WiFi_Timeout);
#endif
                                }
*/
				// ATCN Command - ATCN - Apply changes and come out of configuration mode
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x43) && (upgradebuf[11] == 0x4e) && (upgradelen == 15))
				{
					//printf("I Received ATCN command \r\n");
					// Apply all the changes and move out of configuration mode
					write(connfd, "+++OK++", 7);
					flag = RCU_SERVER;
				}
				// Restart Command - ATFR - Module Reset 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x46) && (upgradebuf[11] == 0x52))
				{
					//printf("I Received ATFR command,Sorry I am Restarting Now \r\n");
					write(connfd, "+++OK++", 7);
					system_restart();
				}
				// Get IP Address - ATMY 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x4d) && (upgradebuf[11] == 0x59) && (upgradelen == 15))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.ip.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "+++%s++",temp_addr);
					//printf("configIpAddress is [%s] \r\n", temp_addr);
					write(connfd, readbuffer,strlen(temp_addr)+5);
				}
				// Get Gateway - ATGW 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x47) && (upgradebuf[11] == 0x57) && (upgradelen == 15))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.gw.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					//printf("configGateway Address is [%s] \r\n", temp_addr);
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "+++%s++",temp_addr);
                                        write(connfd, readbuffer, strlen(temp_addr)+5);
				}
				// Get NetMask - ATMK 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x4d) && (upgradebuf[11] == 0x4b) && (upgradelen == 15))
				{
					temp_addr = inet_ntoa(wificonfig_powerup.ipInfo.netmask.addr);
					temp_addr[strlen(temp_addr)] = '\0';
					//printf("confignetmask Address is [%s] \r\n", temp_addr);
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "+++%s++",temp_addr);
					write(connfd, readbuffer, strlen(temp_addr)+5);
				}
				// Get ATID  
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x49) && (upgradebuf[11] == 0x44) && (upgradelen == 15))
				{
					  char temp_read[256];
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.ssid, strlen(wificonfig_powerup.ssid));
					readbuffer[strlen(readbuffer)] = '\0';
					memset(temp_read, 0, 256);
					sprintf(temp_read, "+++%s++",readbuffer);
					write(connfd, temp_read, strlen(readbuffer)+5);
					//flag = RCU_SERVER;
				}
				// Get Server IP Address - ATSI 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x53) && (upgradebuf[11] == 0x49) && (upgradelen == 15))
				{	//printf("\n ATSI");
					  char temp_read[256];
					memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.SIpAddress, strlen(wificonfig_powerup.SIpAddress));
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read, "+++%s++",readbuffer);
					write(connfd, temp_read, strlen(readbuffer)+5);
				}
				// Get Server Port - ATSP 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x53) && (upgradebuf[11] == 0x50) && (upgradelen == 15))
				{
					  char temp_read[256];
                                        memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "%d", wificonfig_powerup.SPort);
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read, "+++%s++",readbuffer);
					//printf("SPort is [%s] \r\n", readbuffer);
					write(connfd,temp_read, strlen(readbuffer)+5);
				}
				// Get User name for authentication - ATAU 
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x41) && (upgradebuf[11] == 0x55) && (upgradelen == 15))
				{
					  char temp_read[256];
                                        memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					strncpy(readbuffer, wificonfig_powerup.auth_username, strlen(wificonfig_powerup.auth_username));
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read, "+++%s++",readbuffer);
					write(connfd,temp_read, strlen(readbuffer));
					//flag = RCU_SERVER;
				}
				// Get Version Info - ATVR
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x56) && (upgradebuf[11] == 0x52) && (upgradelen == 15))
				{       sprintf(readbuffer,"+++%s++",version_buffer);
					//printf("version_buffer is [%s] %d\r\n", readbuffer,strlen(version_buffer)+5);
					write(connfd,readbuffer, strlen(version_buffer)+5);
				}
				// Get Accesspoint Connection Status - ATAI
				else if ((upgradebuf[7] == '[') && (upgradebuf[8] == 0x41) && (upgradebuf[9] == 0x54) && (upgradebuf[10] == 0x41) && (upgradebuf[11] == 0x49) && (upgradelen == 15))
				{
					  char temp_read[256];
					memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					if (1 == access_point_status)
					{
						t = 0;
						sprintf(readbuffer, "%d", t);
					}
					else
					{
						t = 1;
						sprintf(readbuffer, "%d", t);
					}
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read,"+++%s++",readbuffer);
					write(connfd,temp_read, strlen(readbuffer+5));
					//flag = RCU_SERVER;
				}
				// Get Channel number - ATCH
				else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x43) && (upgradebuf[10] == 0x48) && (upgradelen == 14))
				{
					  char temp_read[256];
					memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					channel_number = 9;
					sprintf(readbuffer, "%d", channel_number);
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read,"+++%s++",readbuffer);
					write(connfd,(char *)&temp_read, strlen(readbuffer)+5);
					//flag = RCU_SERVER;
				}
                /*
				// Get Channel number - ATBD
				else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x42) && (upgradebuf[10] == 0x44) && (upgradelen == 14))
				{
					  char temp_read[256];
                                        memset(temp_read, 0, 256);
					memset(readbuffer, 0, 256);
					sprintf(readbuffer, "%d", wificonfig_powerup.baudrate);
					readbuffer[strlen(readbuffer)] = '\0';
					sprintf(temp_read,"+++%s++",readbuffer);
					write(connfd,(uint8_t *)&temp_read, strlen(readbuffer)+5);
					//flag = RCU_SERVER;
				} 
                */
				// ATRT GET
/*
				else if ((upgradebuf[7] == 0x41) && (upgradebuf[8] == 0x54) && (upgradebuf[9] == 0x52) && (upgradebuf[10] == 0x54) && (upgradelen == 14))
                                {
					  char temp_read[256];
                                        memset(temp_read, 0, 256);
                                        memset(readbuffer, 0, 256);
                                        sprintf(readbuffer, "%d", wificonfig_powerup.RCU_WiFi_Timeout);
                                        readbuffer[strlen(readbuffer)] = '\0';
                                        sprintf(temp_read,"+++%s++",readbuffer);
                                        write(connfd,temp_read, strlen(readbuffer)+5);

#ifdef Debug_Enable
                                        printf("\n I sent RCU Time out for WIFI %d",wificonfig_powerup.RCU_WiFi_Timeout);
#endif
				}
*/
                /*
				// GET Time period for heartbeat ATHBT
                                else if ((upgradebuf[7] == 'A') && (upgradebuf[8] == 'T') && (upgradebuf[9] == 'H') && (upgradebuf[10] == 'B') && (upgradebuf[11] == 'T'))
                                {
                                          char temp_read[256];
                                        memset(temp_read, 0, 256);
                                        memset(readbuffer, 0, 256);
//                                        sprintf(readbuffer, "%d", ext_config_powerup.HBtime);
                                        readbuffer[strlen(readbuffer)] = '\0';
                                        sprintf(temp_read,"+++%s++",readbuffer);
                                        write(connfd,(uint8_t *)&temp_read, strlen(readbuffer)+5);
                                        //flag = RCU_SERVER;
//                                        printf("I sent RCU Time out for WIFI Heartbeat %d",ext_config_powerup.HBtime);
                                }
				// SET Time period for heartbeat ATHBT
                                else if ((upgradebuf[7] == 'A') && (upgradebuf[8] == 'T') && (upgradebuf[9] == 'H') && (upgradebuf[10] == 'B') && (upgradebuf[11] == 'T'))
                                {
	                                for (j = 11, k = 0; j <upgradelen-2; j++)
                                        {
                                                Rwt[k] = upgradebuf[j];
                                                k++;
                                        }
                                        Rwt[k - 1] = '\0';
                                        ext_config_write.HBtime = atoi(Rwt);
                                        write(connfd, "+++OK++", 7);
                                        printf("I received RCU Time out for WIFI Heart beat %d",ext_config_write.HBtime);
                                }
*/

				memset(upgradebuf, 0, 256);
				upgradelen = 0, k = 0;
			}
}
void resetwificonfiginfo()
{
    uint8_t mac[6];
    char *ssid_mac = (char*) malloc(8);

	memset(&wificonfig_write, 0, sizeof(wificonfig_write));
    esp_efuse_mac_get_default(mac);

    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(ssid_mac, "EOS-%02x%02x ",mac[4], mac[5]);

    strncpy(wificonfig_write.ap_password, "qwerty123#$ ", strlen("qwerty123#$ "));
    wificonfig_write.ap_password[strlen(wificonfig_write.ap_password)-1] = '\0';

    memset(wificonfig_write.ap_ssid, 0, SSID_SIZE);
    strncpy(wificonfig_write.ap_ssid, ssid_mac, strlen(ssid_mac));
    wificonfig_write.HBtime = 999; 
    wificonfig_write.ap_ssid[strlen(wificonfig_write.ap_ssid)-1] = '\0';
    wificonfig_write.wifi_mode = ESP_MODE_UNSET; 
	wificonfig_write.SPort = atoi("4096");
	wificonfig_write.baudrate = atoi("7"); 
     
    strncpy(wificonfig_write.auth_username, "ADMIN ", strlen("ADMIN "));
    wificonfig_write.auth_username[strlen(wificonfig_write.auth_username) - 1] = '\0';

    strncpy(wificonfig_write.auth_password, "ADMIN ", strlen("ADMIN "));
    wificonfig_write.auth_password[strlen(wificonfig_write.auth_password) - 1] = '\0';

    strncpy(wificonfig_write.userpassword, "admin", strlen("admin"));
    wificonfig_write.userpassword[PASSWORD_SIZE - 1] = '\0';

	strncpy(wificonfig_write.ssid, "None", strlen("None"));
	wificonfig_write.ssid[SSID_SIZE - 1] = '\0';

    strncpy(wificonfig_write.hostname, ssid_mac, strlen(ssid_mac));
    wificonfig_write.hostname[strlen(ssid_mac) - 1] = '\0';

	strncpy(wificonfig_write.SIpAddress, "0.0.0.0", strlen("0.0.0.0"));
	wificonfig_write.SIpAddress[15] = '\0';

    wificonfig_write.Reboot_count = 0;

	//printf( "I am resetting WIFI configuration\r\n");
	//printf( "[SSID=%s] \r\n", wificonfig_write.ssid);
	//printf( "[PWD=%s] \n\r", wificonfig_write.password);
	//printf( "[auth_username=%s] \r\n", wificonfig_write.auth_username);
	//printf( "[auth_password=%s] \n\r", wificonfig_write.auth_password);
	//printf( "[ServerIpAddress=%s] \r\n", wificonfig_write.SIpAddress);
	//printf( "[ServerPort=%d] \r\n", wificonfig_write.SPort);
	//printf( "[Write wificonfig_write.ipInfo.ip.addr=%d \r\n", wificonfig_write.ipInfo.ip.addr);
	//printf( "[Write wificonfig_write.ipInfo.gw.addr =%d \r\n", wificonfig_write.ipInfo.gw.addr);
	//printf( "[Write wificonfig_write.ipInfo.netmask.addr =%d \r\n", wificonfig_write.ipInfo.netmask.addr);
	//printf( "[baudrate is = %d] \r\n", wificonfig_write.baudrate);
    //erasewificonfiginfo();
	savewificonfiginfo(&wificonfig_write);
}
// zzzzZZZZzzzz....
