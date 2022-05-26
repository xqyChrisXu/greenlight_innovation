/*
 * wifi.c
 *
 *  Created on: Apr 11, 2019
 *      Author: Sangita Sridar
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "main.h"
#include "wifi.h"
#include "spi.h"
#include "picohttpparser.h"
#include "errorLogger.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include <sodium.h>

#include "esp_wifi.h"
#include "esp_event_loop.h"

#include <string.h>
#include <stdint.h>

#include "esp_wpa2.h"
#include "tcpip_adapter.h"
#include <stdio.h>
#include <stdbool.h>


const char *reason_phrase;
size_t reason_phrase_len;
struct phr_header headers[128];
size_t num_headers;
const char *buffer;
const char *recv_buf;

int pos, prev_pos, minor_http_version, status;

#define SSID_LENGTH 32
#define PASSPHRASE_LENGTH 64
#define MESSAGE_LENGTH 256
#define MESSAGE_LENGTH1 256
#define SERVER_IP_LENGTH 16

#define MAX_APs 16

static const char *TAG = "tcp_client";

#define MESSAGE_LEN 23
#define CIPHERTEXT_LEN (crypto_secretbox_MACBYTES + MESSAGE_LEN)

uint8_t decrypted[MESSAGE_LEN];

unsigned char key[crypto_secretbox_KEYBYTES] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
unsigned char nonce[crypto_secretbox_NONCEBYTES];
unsigned char nonce_first_read[crypto_secretbox_NONCEBYTES];
unsigned char nonce_second_read[crypto_secretbox_NONCEBYTES];
unsigned char ciphertext[CIPHERTEXT_LEN];


static char TCPServerIP[SERVER_IP_LENGTH] = "192.168.134.23";
static char SSID[SSID_LENGTH] = "BP";
static char PASSPHRASE[PASSPHRASE_LENGTH] = "mmhg-/lxqub3";

//static char TCPServerIP[SERVER_IP_LENGTH] = "192.168.134.23";
//char SSID[32] = "Not Set";
//static char PASSPHRASE[64] = "Not Set";
//static char USERNAME[32] = "Not Set";

char TCP_MESSAGE[MESSAGE_LENGTH]="POST /duo-device/ HTTP/1.1\r\nHost: 192.168.134.23:18000\r\nContent-Length: 54\r\n\r\n";

uint8_t n0[5] = {
		(uint8_t) 'D',
		(uint8_t) 'E',
		(uint8_t) 'V',
		(uint8_t) 'I',
		(uint8_t) 'D',
};

uint8_t n1[4] =  {0x00,0x00,0x00,0X01};

uint8_t n2[5] = {
		(uint8_t) 'R',
		(uint8_t) 'X',
		(uint8_t) 'C',
		(uint8_t) 'M',
		(uint8_t) 'D',
};


static char TCP_MESSAGE1[MESSAGE_LENGTH] = "POST /duo-device/ HTTP/1.1\r\nHost: 192.168.134.23:18000\r\nContent-Length: 81\r\n\r\n";

uint8_t m0[5] = {
		(uint8_t) 'D',
		(uint8_t) 'E',
		(uint8_t) 'V',
		(uint8_t) 'I',
		(uint8_t) 'D',
};

uint8_t m1[4] =  {0x00,0x00,0x00,0X01};

uint8_t m2[10] = {
		(uint8_t) 'T',
		(uint8_t) 'X',
		(uint8_t) 'D',
		(uint8_t) 'A',
		(uint8_t) 'T',
		(uint8_t) 'M',
		(uint8_t) 'E',
		(uint8_t) 'A',
		(uint8_t) 'S',
		(uint8_t) 'R',
};



static wifi_ap_record_t ap_records[MAX_APs];
static uint16_t ap_num;
tcpip_adapter_ip_info_t ip;
static esp_err_t event_handler(void *ctx, system_event_t *event);
static char* getAuthModeName(wifi_auth_mode_t auth_mode);


#define EXAMPLE_EAP_METHOD 1	//CONFIG_EAP_METHOD	// EAP method TLS: 0, PEAP: 1, TTLS: 2
/* Constants that aren't configurable in menuconfig */
#define EAP_PEAP 1
#define EAP_TTLS 2

extern uint8_t ca_pem_start[] asm("_binary_wpa2_ca_pem_start");
extern uint8_t ca_pem_end[]   asm("_binary_wpa2_ca_pem_end");
extern uint8_t client_crt_start[] asm("_binary_wpa2_client_crt_start");
extern uint8_t client_crt_end[]   asm("_binary_wpa2_client_crt_end");
extern uint8_t client_key_start[] asm("_binary_wpa2_client_key_start");
extern uint8_t client_key_end[]   asm("_binary_wpa2_client_key_end");

void WiFiTCPTask(void *pvParam) {
	struct sockaddr_in tcpServerAddr;

	tcpServerAddr.sin_addr.s_addr = inet_addr(TCPServerIP);
	tcpServerAddr.sin_family = AF_INET;
	tcpServerAddr.sin_port = htons(18000);
	int32_t serverConnection, replyLength;

	char *recv_buf = (char *)malloc(4096*sizeof(char)); //free(recv_buf);

	ESP_LOGI(TAG, "tcp_client task started \n");

	FOREVER {
		printf("Entered forever loop\r\n");
		newEventMask = xEventGroupWaitBits(wifi_event_group, (SCAN_START_BIT|WRITE_RXCMD_BIT|WRITE_TXCMD_DATA_BIT|WRITE_TXCMD_BIT), NOCLEAR, NOWAIT,portMAX_DELAY);
		printf("newEventMask Satisfied\r\n");
		/*Case 1: SCAN_START_BIT*/
		if ((newEventMask & SCAN_START_BIT) != 0) {
			printf("newEventMask&Scan_START_BIT are Satisfied\r\n");
			wifi_scan_config_t scanConf = {
					.ssid = NULL,
					.bssid = NULL,
					.channel = 0,
					.show_hidden = 1,
					.scan_type = WIFI_SCAN_TYPE_PASSIVE,
					.scan_time.active.min = 100,
					.scan_time.active.max = 1200,
					.scan_time.passive = 1200,
			};

			ap_num = MAX_APs;
			ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, 0));
			printf("SCAN_START_BIT:\r\n");
			xEventGroupClearBits( wifi_event_group, SCAN_START_BIT);
		}

		/*Case 2: WRITE_RXCMD_BIT*/
		if ((newEventMask & WRITE_RXCMD_BIT) != 0){
			printf("WRITE_RXCMD BIT");
			serverConnection = socket(AF_INET, SOCK_STREAM, 0);
			if (serverConnection == -1) {
				ESP_LOGE(TAG, "... Failed to allocate socket.\n");
				vTaskDelay(1 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... allocated socket\n");
			if (connect(serverConnection, (struct sockaddr *) &tcpServerAddr,
					sizeof(tcpServerAddr)) != 0) {
				ESP_LOGE(TAG, "... socket connect failed errno=%d \n", errno);
				close(serverConnection);
				vTaskDelay(4 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... connected \n");

			uint8_t MESSAGE0[78+14+24+16]={0x00};
			memcpy(MESSAGE0, TCP_MESSAGE, strlen(TCP_MESSAGE));
			memcpy(MESSAGE0 + 78, n0, 5);
			memcpy(MESSAGE0 + 78 + 5 , n1, 4);

			randombytes_buf(nonce, sizeof nonce);

			if (crypto_secretbox_easy(ciphertext, n2, 5, nonce, key) != 0) {
				printf("\n\n ciphertext is %s", ciphertext);
			}

			memcpy(MESSAGE0 + 78 + 5 + 4 , nonce, sizeof(nonce));
			memcpy(MESSAGE0 + 78 + 5 + 4 + sizeof(nonce) , ciphertext, 5+16);

			printf("TCP_MESSAGE is %s", MESSAGE0);

			if (write(serverConnection, MESSAGE0, 78+14+ sizeof(nonce) + 16) < 0) {

				ESP_LOGE(TAG, "... Send failed \n");
				close(serverConnection);
				vTaskDelay(4 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... socket send success");

			int bytes_parsed = 0;
			pos = 0;
			while (1) {
				bzero(recv_buf, 4096);


				printf("Reading.");
				replyLength = read(serverConnection, recv_buf+pos, 4096 - 1 -pos);

				printf("replyLength is %d \n", replyLength);

				if (replyLength <= 0) {
					// Handle read error
					printf("Read Error");
					break;
				}
				prev_pos = pos;
				pos += replyLength;

				num_headers = sizeof(headers) / sizeof(headers[0]); // Set to maximum allowed number of headers
				printf("num_headers is %d \n", num_headers);

				bytes_parsed = phr_parse_response(recv_buf, pos, &minor_http_version, &status, &reason_phrase, &reason_phrase_len, headers, &num_headers, prev_pos); //prev_pos = 0;
				// Request was incomplete
				if (bytes_parsed == -2) {
					if (pos == sizeof(recv_buf)) {
						// Request was bigger than receive buffer, handle error

						printf("Message is too big");
						break;
					}
					printf("Message is incompleted");
					continue;
				}

				if (bytes_parsed == -1) {
					// Handle parse error

					printf("HTTP parser is error");
					break;
				}

				if (bytes_parsed < -2 || bytes_parsed == 0) {
					// ?
					printf("unknown state");
					break;
				}
				int Content_Length = -1;


				//const char cmp_header;
				for(int j=0; j < num_headers; j++ ){

					if(strncasecmp(headers[j].name, "Content-Length", headers[j].name_len) == 0){

						Content_Length= mem_tozd_rjzf(headers[j].value, headers[j].value_len);
						printf("Content_Length in the for loop is %d \n\n", Content_Length);

					}

				}
				if(Content_Length == -1){
					printf("No Content_Length Header.");
					break;

				}else if(Content_Length + bytes_parsed <= replyLength){
					int total_length =  bytes_parsed + Content_Length;
					printf("Content_Length is %d \n", Content_Length);
					//						Response_Successed = true;

					for (int k=0; k < Content_Length; k++ ){
						printf("%02X", recv_buf[total_length-Content_Length+k]);// command that send from server and ready to send out to NXP

					}

					break;

				}else if(Content_Length + bytes_parsed > replyLength){
					continue;
				}

			}


			printf("\nwhile loop ends here\n");

			for (int index = 0; index < 24; index++){
				nonce_first_read[index]=(uint8_t) recv_buf[index + bytes_parsed];
			}

			for (int index = 0; index < 50; index++){   // 34 message + 16 MAC = 50
				ciphertext[index]=(uint8_t) recv_buf[index + bytes_parsed+24]; //24 for nonce
			}

			if (crypto_secretbox_open_easy(decrypted, ciphertext, 50, nonce_first_read, key) != 0) {
				printf("\nFailed on decryption.\n   ");
			}

			for(int b=0; b<50; b++){
				printf("%02x", decrypted[b]);
			}

			for (int index = 0; index < 22; index++){
				spi_send_data[index]=decrypted[index+12];
			}

			printf("\nspi_send_data is here:   ");

			for(int b=0; b<22; b++){
				printf("%02x", spi_send_data[b]);
			}

			ESP_LOGI(TAG,

					"... done reading from socket. Last read return=%d errno=%d\r\n",
					replyLength, errno);
			close(serverConnection);
			ESP_LOGI(TAG, "... new request in 5 seconds");
			vTaskDelay(5 * SECONDS);

			printf("Connect finished");
			global_variable_rxcmd=0;
			xEventGroupSetBits(spi_event_group, WRITE_RXCMD_BIT);//confirmation to the spi task that it finshed rxcmd and received the data and it sent notification to the mcu
		}

		/*	Case 3: WRITE_TXCMD_BIT	 */
		if ((newEventMask & WRITE_TXCMD_BIT) != 0){
			serverConnection = socket(AF_INET, SOCK_STREAM, 0);
			if (serverConnection == -1) {
				ESP_LOGE(TAG, "... Failed to allocate socket.\n");
				vTaskDelay(1 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... allocated socket\n");
			if (connect(serverConnection, (struct sockaddr *) &tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
				ESP_LOGE(TAG, "... socket connect failed errno=%d \n", errno);
				close(serverConnection);
				vTaskDelay(4 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... connected \n");

			uint8_t MESSAGE1[78+19+22+16+24]={0x00};
			memcpy(MESSAGE1, TCP_MESSAGE1, strlen(TCP_MESSAGE1));
			memcpy(MESSAGE1 + 78, m0, 5);
			memcpy(MESSAGE1 + 78 + 5, m1, 4);

			uint8_t MESSAGE2[10+22]={0x00};
			memcpy(MESSAGE2, m2, 10);
			memcpy(MESSAGE2 + 10, spi_received_data, 22);

			// wifi encryption here

			randombytes_buf(nonce, sizeof nonce);
			if (crypto_secretbox_easy(ciphertext, MESSAGE2, 10+22, nonce, key) != 0) {
				printf("\nFailed on encryption.\n");
			}
			printf("\nMessage2 is:\n");
			for(int b=0; b<32; b++){
				printf("%d %02x\n", b, MESSAGE2[b]);
			}

			printf("\nAfter encrypted:\n");
			for(int b=0; b<32+16; b++){
				printf("%d %02x\n", b, ciphertext[b]);
			}

			memcpy(MESSAGE1 + 78 + 9, nonce, 24);
			memcpy(MESSAGE1 + 78 + 9 + 24, ciphertext, 10+22+16);

			printf("TCP_MESSAGE1 is %s", MESSAGE1);

			printf("\nIn TCP Task, spi_received_data are\n");

			for(int b=0; b<22; b++){
				printf("%02x", spi_received_data[b]);
			}

			if (write(serverConnection, MESSAGE1, 78+19+22+16+24) < 0) {

				ESP_LOGE(TAG, "... Send failed \n");
				close(serverConnection);
				vTaskDelay(4 * SECONDS);
				continue;
			}

			ESP_LOGI(TAG, "... socket send success");

			int bytes_parsed = 0;
			pos = 0;
			minor_http_version = 0;
			status =0;
			prev_pos = 0;


			while(1){

				bzero(recv_buf, 4096);

				replyLength = read(serverConnection, recv_buf+pos, 4096-pos);

				//printf("replyLength for the second database responds is %d \n", replyLength);

				if (replyLength <= 0) {
					// Handle read error
					printf("Read Error");
					break;
				}
				prev_pos = pos;
				pos += replyLength;

				num_headers = sizeof(headers) / sizeof(headers[0]); // Set to maximum allowed number of headers
				//printf("num_headers for the second database responds is %d \n", num_headers);

				bytes_parsed = phr_parse_response(recv_buf, pos, &minor_http_version, &status, &reason_phrase, &reason_phrase_len, headers, &num_headers, prev_pos); //prev_pos = 0;
				//printf("bytes_parsed is %d \n", bytes_parsed);


				// Request was incomplete
				if (bytes_parsed == -2) {
					if (pos == sizeof(recv_buf)) {
						// Request was bigger than receive buffer, handle error

						printf("Message is too big");
						break;
					}
					printf("Message is incompleted");
					continue;
				}

				if (bytes_parsed == -1) {
					// Handle parse error

					printf("HTTP parser is error");
					break;
				}

				if (bytes_parsed < -2 || bytes_parsed == 0) {
					// ?
					printf("unknown state");
					break;
				}
				int Content_Length = -1;


				for(int j=0; j < num_headers; j++ ){
					if(strncasecmp(headers[j].name, "Content-Length", headers[j].name_len) == 0){

						Content_Length= mem_tozd_rjzf(headers[j].value, headers[j].value_len);
					}

				}

				if(Content_Length == -1){
					printf("No Content_Length Header.");
					break;

				}else if(Content_Length + bytes_parsed <= replyLength){
					int total_length =  bytes_parsed + Content_Length;
					printf("Content_Length for the second database responds is %d \n", Content_Length);

					for (int k=0; k < Content_Length; k++ ){
						printf("%02X", recv_buf[total_length-Content_Length+k]); // command that send from server and ready to send out to NXP

					}

					break;

				}else if(Content_Length + bytes_parsed > replyLength){
					continue;
				}

			}

			printf("\nSecond while loop ends here\n");

			for (int index = 0; index < 24; index++){
				nonce_second_read[index]=(uint8_t) recv_buf[index + bytes_parsed];
			}

			for (int index = 0; index < 23; index++){   // 7 message + 16 MAC = 23
				ciphertext[index]=(uint8_t) recv_buf[index + bytes_parsed+24]; //24 for nonce
			}

			if (crypto_secretbox_open_easy(decrypted, ciphertext, 23, nonce_second_read, key) != 0) {
				printf("\nFailed on decryption.\n   ");
			}

			for(int b=0; b<23; b++){
				printf("%02x", decrypted[b]);
			}

			for (int index = 0; index < 7; index++){
				spi_send_data[index]=decrypted[index];
			}

			printf("\n*****Final result***** spi_send_data for the second database responds is here:   ");

			for(int b=0; b<22; b++){
				printf("%02x", spi_send_data[b]);
			}

			ESP_LOGI(TAG,

					"... done reading from socket. Last read return=%d errno=%d\r\n",
					replyLength, errno);
			close(serverConnection);
			ESP_LOGI(TAG, "... new request in 5 seconds");
			vTaskDelay(5 * SECONDS);
			global_variable_txcmd=0;
			//xEventGroupClearBits( wifi_event_group, SCAN_START_BIT);
			xEventGroupSetBits(spi_event_group, WRITE_TXCMD_BIT);
		}

	}

	free(recv_buf);
}

void WiFiInitialize(void) {

	esp_log_level_set("wifi", ESP_LOG_NONE); // disable wifi driver logging

	// initialize the tcp stack
	tcpip_adapter_init();

	wifi_event_group = xEventGroupCreate();

	// initialize the wifi event handler
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	// configure, initialize and start the wifi driver
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_connect(void) {
	wifi_config_t cfg = { .sta = { .ssid[0] = 0 } };
	strlcpy((char*) cfg.sta.ssid, SSID, SSID_LENGTH);
	strlcpy((char*) cfg.sta.password, PASSPHRASE, PASSPHRASE_LENGTH);

	ESP_ERROR_CHECK(esp_wifi_disconnect());
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg));
	ESP_ERROR_CHECK(esp_wifi_connect());
}

//void wifi_connect(void) {
//
//	uint8_t SSID_LENGTH_RECEIVED = spi_usb_received_data[1];
//
//	for(int i = 0; i < SSID_LENGTH_RECEIVED; i++)
//	{
//		SSID[i] = spi_usb_received_data[i+2];
//	}
//
//	uint8_t PASSPHRASE_LENGTH_RECEIVED = 0;
//	if(spi_usb_received_data[0] == 5){
//		PASSPHRASE_LENGTH_RECEIVED = spi_usb_received_data[1 + SSID_LENGTH_RECEIVED];  	//+1; Removed added 1 byte // Adding one null byte in the end of password, as per password requires
//	}
//	else{
//		PASSPHRASE_LENGTH_RECEIVED = spi_usb_received_data[1 + SSID_LENGTH_RECEIVED]+1;
//	}
//
//	//PASSPHRASE_LENGTH_RECEIVED = spi_usb_received_data[1 + SSID_LENGTH_RECEIVED]+1;
//
//	for(int i = 0; i < PASSPHRASE_LENGTH_RECEIVED; i++){
//
//		PASSPHRASE[i] = spi_usb_received_data[i + 2 + SSID_LENGTH_RECEIVED];
//
//	}
//
//	printf("Entering wifi.c:\r\n");
//
//	printf("\nSSID_LENGTH is: %d\n", SSID_LENGTH_RECEIVED);
//
//	printf("\nSSID is: %s\n", SSID);
//
//	printf("\nPASSPHRASE_LENGTH is: %d\n", PASSPHRASE_LENGTH_RECEIVED);
//
//	for(int b=0; b< PASSPHRASE_LENGTH_RECEIVED ; b++){
//		printf("%02x", PASSPHRASE[b]);
//	}
//
//	if(spi_usb_received_data[0] == 5){
//
//		uint8_t USERNAME_LENGTH = spi_usb_received_data[2 + SSID_LENGTH_RECEIVED + PASSPHRASE_LENGTH_RECEIVED];
//
//		printf("\nUSERNAME_LENGTH is: %d\n", USERNAME_LENGTH);
//
//		for(int i = 0; i < USERNAME_LENGTH; i++){
//
//			USERNAME[i] = spi_usb_received_data[i+ 3 + SSID_LENGTH_RECEIVED + PASSPHRASE_LENGTH_RECEIVED];
//		}
//		printf("\r\nUsername in wifi.c is:\r\n");
//		for(int b=0; b< USERNAME_LENGTH ; b++){
//			printf("%02x", USERNAME[b]);
//		}
//		printf("\r\n");
//
//		/*Enterprise Configurations*/
//		esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
//
//		wifi_config_t wifi_config = {
//				.sta = {
//						.ssid[0] = SSID[0],
//						.ssid[1] = SSID[1],
//						.ssid[2] = SSID[2],
//						.ssid[3] = SSID[3],
//						.ssid[4] = SSID[4],
//						.ssid[5] = SSID[5],
//						.ssid[6] = SSID[6],
//						.ssid[7] = SSID[7],
//						.ssid[8] = SSID[8],
//						.ssid[9] = SSID[9],
//						.ssid[10] = SSID[10],
//						.ssid[11] = SSID[11],
//						.ssid[12] = SSID[12],
//						.ssid[13] = SSID[13],
//						.ssid[14] = SSID[14],
//						.ssid[15] = SSID[15],
//						.ssid[16] = SSID[16],
//						.ssid[17] = SSID[17],
//						.ssid[18] = SSID[18],
//						.ssid[19] = SSID[19],
//						.ssid[20] = SSID[20],
//						.ssid[21] = SSID[21],
//						.ssid[22] = SSID[22],
//						.ssid[23] = SSID[23],
//						.ssid[24] = SSID[24],
//						.ssid[25] = SSID[25],
//						.ssid[26] = SSID[26],
//						.ssid[27] = SSID[27],
//						.ssid[28] = SSID[28],
//						.ssid[29] = SSID[29],
//						.ssid[30] = SSID[30],
//						.ssid[31] = SSID[31],
//				}
//		};
//
//		memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
//		ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
//		ESP_ERROR_CHECK(esp_wifi_disconnect());
//		ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
//		printf("esp_wifi_set_config\r\n");
//		ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_username((uint8_t *)USERNAME, USERNAME_LENGTH) );
//		ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_password((uint8_t *)PASSPHRASE, PASSPHRASE_LENGTH_RECEIVED) );
//		printf("esp_wifi_sta_wpa2_ent_set_username&set_password\r\n");
//		ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_enable(&config) );
//
//		ESP_ERROR_CHECK(esp_wifi_connect());
//		vTaskDelay(2000 / portTICK_PERIOD_MS);
//
//	}else{
//		wifi_config_t cfg = { .sta = { .ssid[0] = 0 } };
//
//		printf("\nEntering !@cfg@!:\r\n");
//
//		printf("\nSSID_LENGTH is: %d\n", SSID_LENGTH_RECEIVED);
//
//		printf("\nSSID is: %s\n", SSID);
//
//		printf("\nPASSPHRASE_LENGTH is: %d\n", PASSPHRASE_LENGTH_RECEIVED);
//
//		for(int b=0; b< PASSPHRASE_LENGTH_RECEIVED ; b++){
//			printf("%02x", PASSPHRASE[b]);
//		}
//
//		strlcpy((char*) cfg.sta.ssid, SSID, SSID_LENGTH_RECEIVED);
//		strlcpy((char*) cfg.sta.password, PASSPHRASE, PASSPHRASE_LENGTH_RECEIVED);
//		ESP_LOGI(TAG, "\r\nSetting WiFi configuration SSID %s...\r\n", cfg.sta.ssid);
//		ESP_LOGI(TAG, "Setting WiFi configuration PW %s...\r\n", cfg.sta.password);
//
//
//		ESP_ERROR_CHECK(esp_wifi_disconnect());
//		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg));
//		ESP_ERROR_CHECK(esp_wifi_connect());
//	}
//	printf("\nConnecting WiFi\r\n");
//}

uint16_t wifi_get_quan_APs( void) {
	return( ap_num);
}

static esp_err_t event_handler(void *ctx, system_event_t *event) {

	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		printf("SYSTEM_EVENT_STA_START:\r\n");
		break;

	case SYSTEM_EVENT_STA_GOT_IP:
		//		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		/*Sangita: The Rx and Tx cmd sent by case SPI_COMMAND_AP_START_SCAN
		 */
		printf("global variable");
		printf("global variable rx %d\r\n",global_variable_rxcmd);
		printf("global variable tx %d\r\n",global_variable_txcmd);
		if(global_variable_rxcmd==1){
			printf("RXCMDBIT set");
			xEventGroupSetBits(wifi_event_group, WRITE_RXCMD_BIT);
		}
		else if(global_variable_txcmd==1){
			xEventGroupSetBits(wifi_event_group, WRITE_TXCMD_BIT);
		}
		break;
		////////////////next case was initially commented out//////////////////
		//		case SYSTEM_EVENT_STA_CONNECTED: {
		//			wifi_scan_config_t scanConf = { .ssid = NULL, .bssid = NULL, .channel =
		//					0, .show_hidden = 1, .scan_type = WIFI_SCAN_TYPE_ACTIVE };
		//			printf("WiFi Connected\r\n");
		//			ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, 0));
		//		}
		//		break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
		if(disconnect_check_bit == 1)
		{
			printf("WiFi Disconnected, Flash Erased.\r\n");
		}
		else
		{
			esp_wifi_connect();
			if(global_variable_rxcmd==1){
				xEventGroupClearBits(wifi_event_group, WRITE_RXCMD_BIT);
			}
			else if(global_variable_txcmd==1){
				xEventGroupClearBits(wifi_event_group, WRITE_TXCMD_BIT);
			}

			printf("WiFi Disconnected\r\n");
			spi_usb_send_data[0] = 3; //The connection failed
		}
		break;

	case SYSTEM_EVENT_SCAN_DONE:
		// get the list of APs found in the last scan
		ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

		ErrorLoggerSend(ERROR_ACTION_SET, ERROR_BITS_SCAN_FINISHED);

		// print the list
		printf("Found %d access points:\n", ap_num);
		printf("\n");
		printf(
				"               SSID              | Channel | RSSI |   Auth Mode \n");
		printf(
				"----------------------------------------------------------------\n");
		struct duplicate_ssid
		{
			char show_ssid[32];
		};

		struct duplicate_ssid ds[16];

		//struct duplicate_ssid show_ssid[32] = {0};

		int y = 0;
		for(int i = 0; i < ap_num; i++)// && (y < 36*ap_num); i++, y+=36)
		{
			printf("%32s | %7d | %4d | %12s\n", (char *) ap_records[i].ssid,
					ap_records[i].primary, ap_records[i].rssi,
					getAuthModeName(ap_records[i].authmode));
			/*first spi_usb_send_data contains total length of all ssid*/

			for(int x = 0; x < 32; x++){
				//spi_usb_send_data[i+4+x] = ap_records[i].ssid[x];
				ds[i].show_ssid[x] = (char *) ap_records[i].ssid[x];
			}

			printf("\nShow ssid%d is %32s\n", i, ds[i].show_ssid);

			int ret = 0;
			volatile bool ret_flag = true;
			char null_array[32];
			volatile bool null_flag = true;

			memset(null_array, 0, 32);

			if(memcmp(ds[i].show_ssid, null_array, 32) != 0)
			{
				null_flag = true;
			}else
			{
				null_flag = false;
			}
			printf("null_flag is %d\r\n", null_flag);

			for(int k = 0; k < ap_num; k++){

				if(k != i){

					ret = memcmp(ds[i].show_ssid, ds[k].show_ssid, 32);

					if(ret == 0){
						ret_flag = false;
						printf("ret_flag = false\r\n");
					}
				}
			}

			//ret = memcmp(ds[i].show_ssid, 0, 32);

			if(ret_flag == true && null_flag == true)
			{
				printf("ret_flag = true\r\n");
				global_ap_number++;

				spi_usb_send_data[y] = ap_records[i].authmode;

				split_var_two = convertFrom16To8(ap_records[i].rssi);
				spi_usb_send_data[1+y] = split_var_two[0];
				spi_usb_send_data[2+y] = split_var_two[1];

				//spi_usb_send_data[i+3] = strnlen((char *)ap_records[i].ssid, 32);

				spi_usb_send_data[3+y] = 32;
				//printf("\nspi_usb_send_data (ssid_length%d): %d\n", i, spi_usb_send_data[3+y]);



				for(int x = 0; x < 32; x++)
				{
					spi_usb_send_data[y+4+x] = (uint8_t)ds[i].show_ssid[x];
				}

				y+=36;

				printf("\nShow ssid%d is %32s\n", i, ds[i].show_ssid);
				//				printf("\n@@@@SPI buf ssid%d is %32x\n", i, spi_usb_send_data[y+4+x]);
			}
			//			else{
			//				ap_num--;
			//
			//			}

			ret_flag = true;

		}

		printf(
				"----------------------------------------------------------------\n");

		for (int i = 0; i < 36*ap_num; i+=36){
			printf("\n\n spi_usb_send_data (type%d): %d\n", i, spi_usb_send_data[i]);
			printf("\n spi_usb_send_data (rssi%d): %d\n", i, spi_usb_send_data[i+1]);
			printf("\n spi_usb_send_data (rssi%d): %d\n", i, spi_usb_send_data[i+2]);
			printf("\n spi_usb_send_data (ssid_length%d): %d\n", i, spi_usb_send_data[i+3]);
			printf("\n spi_usb_send_data (ssid): ");

			for(int x = 0; x < 32; x++)
			{
				printf("%d", spi_usb_send_data[i+4+x]);
			}
			printf("\n\n");
		}
		break;

	default:
		printf("got an event %d\r\n", event->event_id);
		/* Do nothing */
		break;
	}
	return ESP_OK;
}


void WiFiStartScan( void) {
	xEventGroupSetBits(wifi_event_group, SCAN_START_BIT);
	printf("\n WiFiStartScan, SCAN_START_BIT sets in wifi.c");


}
void WriteMessage( void) {
	xEventGroupSetBits(wifi_event_group, WRITE_BIT);
}


char* WiFiGetSSID( uint8_t which) {
	return( (char*) ap_records[which].ssid);
}

static char* getAuthModeName(wifi_auth_mode_t auth_mode) {

	static const char *names[] = { "OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK",
			"MAX" };
	if (auth_mode < sizeof(names)) {
		return( names[auth_mode]);
	} else {
		return( NULL);
	}
}

int mem_tozd_rjzf(const char *buf, int len)        // digits only
{
	int n=0;

	while (len--)
		n = n*10 + *buf++ - '0';

	return n;
}





