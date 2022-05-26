/*
 * ble.c
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
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_event_loop.h"
#include "main.h"
#include "ble.h"
#include "spi.h"

static uint8_t adv_config_done = 0;


static uint8_t manufacturer[4]={'M', 'M', 'H', 'G'}; //Commend out due to Feb 06 2020 testing

//static uint8_t manufacturer[34]={0xFF, 0xFF, '1', 'M', 'H', 'G', '2', 'M', 'H', 'G', '3', 'M', 'H', 'G', '4', 'M', 'H', 'G','5', 'M', 'H', 'G', '6', 'M', 'H', 'G', '7', 'M', 'H', 'G', '8', 'M', 'H', 'G'};

//custom uuid:d1bf0000-d6fe-4db3-821f-d253a707bde9
static uint8_t sec_service_uuid[16] = {
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		//0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
		0xe9, 0xbd, 0x07, 0xa7, 0x53, 0xd2, 0x1f, 0x82, 0xb3, 0x4d, 0xfe, 0xd6, 0x00, 0x00, 0xbf, 0xd1
};

/*
GAP config variables, functions
 */
// config adv data
static esp_ble_adv_data_t blood_pressure_adv_config = {

		//Commend out due to Feb 06 2020 testing
				.set_scan_rsp = false,
				.include_txpower = true,
				.min_interval = 0x100,
				.max_interval = 0x100,
				.appearance = 0x00,
				.manufacturer_len = 0, //manufacturer_DATA_LEN,
				.p_manufacturer_data =  NULL, //&manufacturer[0],
				.service_data_len = 0,
				.p_service_data = NULL,
				.service_uuid_len = sizeof(sec_service_uuid),
				.p_service_uuid = sec_service_uuid,
				.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),


//		.set_scan_rsp = false,
//		.include_name = false,
//		//.include_name = false,
//		.manufacturer_len = sizeof(manufacturer),
//		.p_manufacturer_data = manufacturer,

};
// config scan response data
static esp_ble_adv_data_t blood_pressure_scan_rsp_config = {

		//Commend out due to Feb 06 2020 testing
				.set_scan_rsp = true,
				.include_name = true,
				//.include_name = false,
				.manufacturer_len = sizeof(manufacturer),
				.p_manufacturer_data = manufacturer,


//		.set_scan_rsp = true,
//		.include_txpower = true,
//		.min_interval = 0x100,
//		.max_interval = 0x100,
//		.appearance = 0x00,
//		.manufacturer_len = 0, //manufacturer_DATA_LEN,
//		.p_manufacturer_data =  NULL, //&manufacturer[0],
//		.service_data_len = 0,
//		.p_service_data = NULL,
//		.service_uuid_len = sizeof(sec_service_uuid),
//		.p_service_uuid = sec_service_uuid,
//		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),


};

static esp_ble_adv_params_t blood_pressure_adv_params = {
		.adv_int_min        = 0x100,
		.adv_int_max        = 0x100,
		.adv_type           = ADV_TYPE_IND,	//Commend out due to Feb 06 2020 testing
		//.adv_type           = ADV_TYPE_NONCONN_IND,	//Changed adv_type for Feb 06 2020 testing
		.own_addr_type      =  BLE_ADDR_TYPE_PUBLIC,
		.channel_map        = ADV_CHNL_ALL,
		.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};



/*
GATT related definitions
 */
#define BPPS_HT_MEAS_MAX_LEN            (500)
///Attributes State Machine

/*
 *  Blood Pressure PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Blood pressure Service

static  uint8_t blood_pressure_svc_a[16] = {0xe9, 0xbd, 0x07, 0xa7, 0x53, 0xd2, 0x1f, 0x82, 0xb3, 0x4d, 0xfe, 0xd6, 0x00, 0x01, 0xbf, 0xd1};
static  uint8_t blood_pressure_svc_b[16] = {0xe9, 0xbd, 0x07, 0xa7, 0x53, 0xd2, 0x1f, 0x82, 0xb3, 0x4d, 0xfe, 0xd6, 0x00, 0x02, 0xbf, 0xd1};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_indicate = ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_read_indicate = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_indicate = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_INDICATE;

/* Characteristic/descriptor values: BP values */
/*
uuid-bp-values
 */
static uint8_t blood_pressure_bp_values_uuid[16] = {0xe9, 0xbd, 0x07, 0xa7, 0x53, 0xd2, 0x1f, 0x82, 0xb3, 0x4d, 0xfe, 0xd6, 0x01, 0x01, 0xbf, 0xd1};
//static const uint16_t blood_pressure_bp_values_uuid = ESP_GATT_BLOOD_PRESSURE_BP_VAL;
// Characteristic Presentation Format: 1.SEQUENCE_NUMBER(2 BYTES) 2.SERIES_ID(4 BYTES) 3.TIME_STAMP(4 BYTES) 4.STATUS(1 BYTES) 5.STATUS FLAGS(1 BYTES)6.SBP(MMHG) (2 BYTES) 7.DBP(MMHG) (2 BYTES) 8. MAP(MMHG) (2 BYTES) 9.HR (2 BYTES) 10. WAVEFORM SAMPLES (2 BYTES).
uint8_t blood_pressure_service_char_bp_values_int[22] = {00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00};
// Client Characteristic Configuration: 0x00: notifications disabled; 0x00: indications disabled
static const uint8_t blood_pressure_service_char_bp_values_descr_config_str[2] = {0x00,0x00};


/* Characteristic/descriptor values: Measurement info */
/*
uuid-bp-values
 */
static uint8_t blood_pressure_meas_info_uuid[16] = {0xe9, 0xbd, 0x07, 0xa7, 0x53, 0xd2, 0x1f, 0x82, 0xb3, 0x4d, 0xfe, 0xd6, 0x01, 0x02, 0xbf, 0xd1};
//static const uint16_t blood_pressure_meas_info_uuid = ESP_GATT_BLOOD_PRESSURE_MEAS_INFO;
// Characteristic Presentation Format:1.SEQUENCE_NUMBER(2 BYTES) 2.SERIES_ID(4 BYTES) 3.USER_ID(4 BYTES) 4.MEASUREMENT_SET_TYPE(1 BYTES) 5.MEASUREMENT_SET_LENGTH(1 BYTES)6.DAY_START_OFFSET(2 BYTES) 7.DAY_FREQUENCY(1 BYTES)  8.NIGHT_START_OFFSET (2 BYTES) 9.NIGHT_FREQUENCY(1 BYTES) 10. MEASUREMENT_SET_START_TIME (4 BYTES)
uint8_t blood_pressure_service_char_meas_info_int[20] ={00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00};
// Client Characteristic Configuration: 0x00: notifications disabled; 0x00: indications disabled
static const uint8_t blood_pressure_service_char_meas_info_descr_config_str[2] = {0x00,0x00};





/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t blood_pressure_a_gatt_db[BP_IDX_NB_A] =
{
		// Blood pressure Service Declaration
		[BP_IDX_SVC_A]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,sizeof(uint16_t), sizeof(blood_pressure_svc_a),blood_pressure_svc_a}},


		// Blood pressure service-BP values Characteristic Declaration
		[BP_IDX_BP_VAL_CHAR]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_indicate}},

		// Blood pressure service-BP values Characteristic Value
		[BP_IDX_BP_VAL_CHAR_VAL]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, blood_pressure_bp_values_uuid, ESP_GATT_PERM_READ_ENCRYPTED,BPPS_HT_MEAS_MAX_LEN ,sizeof(blood_pressure_service_char_bp_values_int),(uint8_t *)blood_pressure_service_char_bp_values_int}},


		// Blood pressure service-BP values Characteristic Client Config value
		[BP_IDX_BP_VAL_CHAR_CONFIG_VAL]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED,BPPS_HT_MEAS_MAX_LEN,sizeof(blood_pressure_service_char_bp_values_descr_config_str), (uint8_t *)blood_pressure_service_char_bp_values_descr_config_str}},

};
static const esp_gatts_attr_db_t blood_pressure_b_gatt_db[BP_IDX_NB_B] =
{
		// Blood pressure Service Declaration
		[BP_IDX_SVC_B]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,sizeof(uint16_t), sizeof(blood_pressure_svc_b), blood_pressure_svc_b}},

		// Blood pressure service-Measurement info Characteristic Declaration
		[BP_IDX_MEAS_INFO_CHAR]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_indicate}},

		// Blood pressure service-Measurement info Characteristic Value
		[BP_IDX_MEAS_INFO_CHAR_VAL]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, blood_pressure_meas_info_uuid, ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED,BPPS_HT_MEAS_MAX_LEN ,sizeof(blood_pressure_service_char_meas_info_int),(uint8_t *) blood_pressure_service_char_meas_info_int}},


		// Blood pressure service-Measurement info Characteristic Client Config value
		[BP_IDX_MEAS_INFO_CHAR_CONFIG_VAL]={{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED,BPPS_HT_MEAS_MAX_LEN ,sizeof(blood_pressure_service_char_meas_info_descr_config_str), (uint8_t *)blood_pressure_service_char_meas_info_descr_config_str}},


};



static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
	char *key_str = NULL;
	switch(key_type) {
	case ESP_LE_KEY_NONE:
		key_str = "ESP_LE_KEY_NONE";
		break;
	case ESP_LE_KEY_PENC:
		key_str = "ESP_LE_KEY_PENC";
		break;
	case ESP_LE_KEY_PID:
		key_str = "ESP_LE_KEY_PID";
		break;
	case ESP_LE_KEY_PCSRK:
		key_str = "ESP_LE_KEY_PCSRK";
		break;
	case ESP_LE_KEY_PLK:
		key_str = "ESP_LE_KEY_PLK";
		break;
	case ESP_LE_KEY_LLK:
		key_str = "ESP_LE_KEY_LLK";
		break;
	case ESP_LE_KEY_LENC:
		key_str = "ESP_LE_KEY_LENC";
		break;
	case ESP_LE_KEY_LID:
		key_str = "ESP_LE_KEY_LID";
		break;
	case ESP_LE_KEY_LCSRK:
		key_str = "ESP_LE_KEY_LCSRK";
		break;
	default:
		key_str = "INVALID BLE KEY TYPE";
		break;

	}

	return key_str;
}

static void show_bonded_devices(void)
{
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);

	ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);
	for (int i = 0; i < dev_num; i++) {
		esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
	}

	free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
																																																																																																		{
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
		esp_ble_remove_bond_device(dev_list[i].bd_addr);
	}

	free(dev_list);
																																																																																																		}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

	switch (event) {
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&blood_pressure_adv_params);
			printf("ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT ADV_START:");
		}
		printf("ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:");
		break;
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CONFIG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&blood_pressure_adv_params);
			printf("ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT ADV_START");
		}
		printf("ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:");
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
			break;
		}
		printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT:");
		ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
		break;
	case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
		//esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0x00);

		printf("ESP_GAP_BLE_PASSKEY_REQ_EVT:");
		break;
	case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
		printf("ESP_GAP_BLE_OOB_REQ_EVT:");
		break;
	case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
		printf("ESP_GAP_BLE_LOCAL_IR_EVT:");
		break;
	case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
		printf("ESP_GAP_BLE_LOCAL_ER_EVT:");
		break;
	case ESP_GAP_BLE_NC_REQ_EVT:
		/* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer deivce. */
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
		printf("ESP_GAP_BLE_NC_REQ_EVT:");
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		/* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should sent the security response with negative(false) accept value*/
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		printf("ESP_GAP_BLE_SEC_REQ_EVT:");
		break;
	case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
		///show the passkey number to the user to input it in the peer deivce.
		ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%d", param->ble_security.key_notif.passkey);
		printf("ESP_GAP_BLE_PASSKEY_NOTIF_EVT:");
		break;
	case ESP_GAP_BLE_KEY_EVT:
		//shows the ble key info share with peer device to the user.
		ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
		printf("ESP_GAP_BLE_KEY_EVT:");
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT: {
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x",\
				(bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				(bd_addr[4] << 8) + bd_addr[5]);
		ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
		show_bonded_devices();
		printf("ESP_GAP_BLE_AUTH_CMPL_EVT: pairing success ");
		break;
	}
	case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
		ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
		ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
		esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
		printf("ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:");
		break;
	}
	case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
		if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
			ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
			break;
		}

		esp_err_t ret = esp_ble_gap_config_adv_data(&blood_pressure_adv_config);
		if (ret){
			ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
		}else{
			adv_config_done |= ADV_CONFIG_FLAG;
		}

		ret = esp_ble_gap_config_adv_data(&blood_pressure_scan_rsp_config);
		if (ret){
			ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
		}else{
			adv_config_done |= SCAN_RSP_CONFIG_FLAG;
		}
		printf("ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:");
		break;
	default:
		break;
	}
}


/*
GATT config variables, functions
 */


static void gatts_profile_event_a_handler(esp_gatts_cb_event_t event,esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n",event);
	switch (event) {
	case ESP_GATTS_REG_EVT:
		esp_ble_gap_set_device_name(DEVICE_NAME);
		//generate a resolvable random address
		esp_ble_gap_config_local_privacy(true);
		esp_ble_gatts_create_attr_tab(blood_pressure_a_gatt_db, gatts_if,BP_IDX_NB_A, BLOOD_PRESSURE_SVC_INST_ID_A);
		printf("ESP_GATTS_REG_EVT:\r\n");
		break;
	case ESP_GATTS_READ_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "gatts_read_value_handler: handle %d\n", param->read.handle);
		ESP_LOGI(GATTS_TABLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		printf("ESP_GATTS_READ_EVT:");
		break;
	case ESP_GATTS_WRITE_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
		esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
		ESP_LOGI(GATTS_TABLE_TAG, "gatts_write_value_handler: handle %d\n", param->write.handle);
		if (blood_pressure_handle_table_A[BP_IDX_BP_VAL_CHAR_CONFIG_VAL]== param->write.handle && param->write.len == 2){
			uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
			if (descr_value == 0x0002){
				ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
				uint8_t indicate_data[20]={0};
				for(int app_send_count=0;app_send_count<20;app_send_count++)
				{
					indicate_data[app_send_count]=spi_received_data[app_send_count];
				}
				//the size of indicate_data[] need less than MTU size which is 20 bytes
				esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, blood_pressure_handle_table_A[BP_IDX_BP_VAL_CHAR_VAL],sizeof(indicate_data), indicate_data, true);
				printf("indicate sent");
				for(int app_send_count=0;app_send_count<20;app_send_count++)
				{
					spi_received_data[app_send_count]=0;
				}
				xEventGroupSetBits(ble_event_group, BLE_SENT_BIT);
			}
		}
		printf("ESP_GATTS_WRITE_EVT:");
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		printf("ESP_GATTS_EXEC_WRITE_EVT:");
		break;
	case ESP_GATTS_MTU_EVT:
		printf("ESP_GATTS_MTU_EVT:");
		break;
	case ESP_GATTS_CONF_EVT:
		printf("ESP_GATTS_CONF_EVT:");
		break;
	case ESP_GATTS_UNREG_EVT:
		printf("ESP_GATTS_UNREG_EVT:");
		break;
	case ESP_GATTS_DELETE_EVT:
		printf("ESP_GATTS_DELETE_EVT:");
		break;
	case ESP_GATTS_START_EVT:
		printf("ESP_GATTS_START_EVT:");
		break;
	case ESP_GATTS_STOP_EVT:
		printf("ESP_GATTS_STOP_EVT:");
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");

		blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].conn_id = param->connect.conn_id;
		printf("blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].conn_id = %d\r\n", blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].conn_id);

		/* start security connect with peer device when receive the connect event sent by the master */
		esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
		printf("ESP_GATTS_CONNECT_EVT:");
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
		/* start advertising again when missing the connect */
		esp_ble_gap_start_advertising(&blood_pressure_adv_params);
		printf("ESP_GATTS_DISCONNECT_EVT:");
		break;
	case ESP_GATTS_OPEN_EVT:
		printf("ESP_GATTS_OPEN_EVT:");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		printf("ESP_GATTS_CANCEL_OPEN_EVT:");
		break;
	case ESP_GATTS_CLOSE_EVT:
		printf("ESP_GATTS_CLOSE_EVT:");
		break;
	case ESP_GATTS_LISTEN_EVT:
		printf("ESP_GATTS_LISTEN_EVT:");
		break;
	case ESP_GATTS_CONGEST_EVT:
		printf("ESP_GATTS_CONGEST_EVT:");
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
		ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x",param->add_attr_tab.num_handle);
		if (param->create.status == ESP_GATT_OK){
			if(param->add_attr_tab.num_handle == BP_IDX_NB_A) {
				memcpy(blood_pressure_handle_table_A, param->add_attr_tab.handles,sizeof(blood_pressure_handle_table_A));
				esp_ble_gatts_start_service(blood_pressure_handle_table_A[BP_IDX_SVC_A]);
				printf("service A started");
			}else{
				ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to BP_IDX_NB_A(%d)",param->add_attr_tab.num_handle, BP_IDX_NB_A);
			}
		}else{
			ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
		}
		printf("ESP_GATTS_CREAT_ATTR_TAB_EVT:");
		break;
	}

	default:
		break;
	}
}


static void gatts_profile_event_b_handler(esp_gatts_cb_event_t event,esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n",event);
	switch (event) {
	case ESP_GATTS_REG_EVT:
		esp_ble_gap_set_device_name(DEVICE_NAME);
		//generate a resolvable random address
		esp_ble_gap_config_local_privacy(true);
		esp_ble_gatts_create_attr_tab(blood_pressure_b_gatt_db, gatts_if,BP_IDX_NB_B, BLOOD_PRESSURE_SVC_INST_ID_B);
		printf("ESP_GATTS_REG_EVT:");
		break;
	case ESP_GATTS_READ_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "gatts_read_value_handler: handle %d\n", param->read.handle);
		printf("ESP_GATTS_READ_EVT:");
		ESP_LOGI(GATTS_TABLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		break;
	case ESP_GATTS_WRITE_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
		esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
		ESP_LOGI(GATTS_TABLE_TAG, "gatts_write_value_handler: handle %d\n", param->write.handle);
		if (blood_pressure_handle_table_B[BP_IDX_MEAS_INFO_CHAR_VAL]== param->write.handle){
			spi_send_data[1]=param->write.value[0];
			spi_send_data[2]=param->write.value[1];
			spi_send_data[3]=param->write.value[2];
			spi_send_data[4]=param->write.value[3];
			spi_send_data[5]=param->write.value[4];
			spi_send_data[6]=param->write.value[5];
			spi_send_data[7]=param->write.value[6];
			spi_send_data[8]=param->write.value[7];
			spi_send_data[9]=param->write.value[8];
			spi_send_data[10]=param->write.value[9];
			spi_send_data[11]=param->write.value[10];
			spi_send_data[12]=param->write.value[11];
			spi_send_data[13]=param->write.value[12];
			spi_send_data[14]=param->write.value[13];
			spi_send_data[15]=param->write.value[14];
			spi_send_data[16]=param->write.value[15];
			spi_send_data[17]=param->write.value[16];
			spi_send_data[18]=param->write.value[17];
			spi_send_data[19]=param->write.value[18];
			spi_send_data[20]=param->write.value[19];
			//			for(int i=0;i<19;i++){
			//				printf("%d\r\n", param->write.value[i]);
			//			}
			//			for(int i=1;i<22;i++){
			//				printf("%02X\r\n", spi_send_data[i]);
			//			}
			xEventGroupSetBits(ble_event_group, BLE_RECEIVED_BIT);
		}
		printf("ESP_GATTS_WRITE_EVT:");
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		printf("ESP_GATTS_EXEC_WRITE_EVT:");
		break;
	case ESP_GATTS_MTU_EVT:
		printf("ESP_GATTS_MTU_EVT:");
		break;
	case ESP_GATTS_CONF_EVT:
		printf("ESP_GATTS_CONF_EVT:");
		break;
	case ESP_GATTS_UNREG_EVT:
		printf("ESP_GATTS_UNREG_EVT:");
		break;
	case ESP_GATTS_DELETE_EVT:
		printf("ESP_GATTS_DELETE_EVT:");
		break;
	case ESP_GATTS_START_EVT:
		printf("ESP_GATTS_START_EVT:");
		break;
	case ESP_GATTS_STOP_EVT:
		printf("ESP_GATTS_STOP_EVT:");
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
		/* start security connect with peer device when receive the connect event sent by the master */
		esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
		printf("ESP_GATTS_CONNECT_EVT:");
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
		/* start advertising again when missing the connect */
		esp_ble_gap_start_advertising(&blood_pressure_adv_params);
		printf("ESP_GATTS_DISCONNECT_EVT:");
		break;
	case ESP_GATTS_OPEN_EVT:
		printf("ESP_GATTS_OPEN_EVT:");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		printf("ESP_GATTS_CANCEL_OPEN_EVT:");
		break;
	case ESP_GATTS_CLOSE_EVT:
		printf("ESP_GATTS_CLOSE_EVT:");
		break;
	case ESP_GATTS_LISTEN_EVT:
		printf("ESP_GATTS_LISTEN_EVT:");
		break;
	case ESP_GATTS_CONGEST_EVT:
		printf("ESP_GATTS_CONGEST_EVT:");
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
		ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x",param->add_attr_tab.num_handle);
		if (param->create.status == ESP_GATT_OK){
			if(param->add_attr_tab.num_handle == BP_IDX_NB_B) {
				memcpy(blood_pressure_handle_table_B, param->add_attr_tab.handles,sizeof(blood_pressure_handle_table_B));
				esp_ble_gatts_start_service(blood_pressure_handle_table_B[BP_IDX_SVC_B]);
				printf("service B started");
			}else{
				ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to BP_IDX_NB_B(%d)",param->add_attr_tab.num_handle, BP_IDX_NB_B);
			}
		}else{
			ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
		}
		printf("ESP_GATTS_CREAT_ATTR_TAB_EVT:");
		break;
	}

	default:
		break;
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			printf("At param->reg.app_id = %d\r\n", param->reg.app_id);

			blood_pressure_profile_tab[param->reg.app_id].gatts_if = gatts_if;

			gatt_if_bit= gatts_if;
			//blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].gatts_if = gatts_if;
			//blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_B].gatts_if = gatts_if;

			printf("blood_pressure_profile_tab[param->reg.app_id].gatts_if = %d\r\n", blood_pressure_profile_tab[param->reg.app_id].gatts_if);
			printf("blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].gatts_if = %d\r\n", blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].gatts_if);

		} else {
			ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id,param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < BLOOD_PRESSURE_PROFILE_NUM; idx++) {
			printf("\r\nInside a/b switch loop at event handler, idx = %d\r\n", idx);

			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == blood_pressure_profile_tab[idx].gatts_if) {
				if (blood_pressure_profile_tab[idx].gatts_cb) {
					printf("At blood_pressure_profile_tab[%d]\r\n", idx);
					printf("blood_pressure_profile_tab[%d].gatts_if = %d\r\n", idx, gatts_if);
					if(write_event_bit == 1)// && idx == 1)
					{
						printf("write_event_bit = 1 event = 2\r\n");
						event = 2;

						blood_pressure_profile_tab[idx].gatts_cb(event, gatts_if, param);
						write_event_bit = 0;
					}
					else{
						printf("write_event_bit = 0\r\n");
						blood_pressure_profile_tab[idx].gatts_cb(event, gatts_if, param);
					}
				}
			}
		}
	} while (0);
}

void BLE_initialize(void){
	esp_err_t ret;
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
	ble_event_group = xEventGroupCreate();

}
void BLE_register(void)
{
	esp_err_t ret;

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
		return;
	}
	//	ret = esp_ble_gatts_app_register(ESP_BLOOD_PRESSURE_APP_ID);
	//	if (ret){
	//		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
	//		return;
	//	}
	ret = esp_ble_gatts_app_register(BLOOD_PRESSURE_PROFILE_APP_IDX_A);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	printf("\r\esp_ble_gatts_app_register\r\n");


	ret = esp_ble_gatts_app_register(BLOOD_PRESSURE_PROFILE_APP_IDX_B);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	printf("\r\esp_ble_gatts_app_register\r\n");



	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;           //set the IO capability to No output No input
	uint8_t key_size = 16;      //the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK|ESP_BLE_CSR_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK|ESP_BLE_CSR_KEY_MASK;
	//set static passkey
	uint32_t passkey = 123456;
	uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
	uint8_t oob_support = ESP_BLE_OOB_DISABLE;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
	/* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you, and the response key means which key you can distribut to the Master; */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}
void BLE_unregister(void){
	esp_ble_gatts_app_unregister(blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_A].gatts_if);
	esp_ble_gatts_app_unregister(blood_pressure_profile_tab[BLOOD_PRESSURE_PROFILE_APP_IDX_B].gatts_if);
}

void BLEtask(void *pvParam) {

	FOREVER {
		bleEventMask = xEventGroupWaitBits(ble_event_group, (ADV_BIT|BLE_RECEIVED_BIT|BLE_SENT_BIT|UNREGIS_BIT), CLEAR, NOWAIT,portMAX_DELAY);
		//		vTaskDelay(10 / portTICK_PERIOD_MS);//added to prevent idle task error
		if ((bleEventMask & ADV_BIT) != 0) {
			BLE_register();
			printf("ble registered");
		}
		if ((bleEventMask & BLE_RECEIVED_BIT) != 0) {
			xEventGroupSetBits(spi_event_group, BLE_RECEIVED_BIT);
		}
		if ((bleEventMask & BLE_SENT_BIT) != 0) {
			xEventGroupSetBits(spi_event_group, BLE_SENT_BIT);
		}
		if ((bleEventMask & UNREGIS_BIT) != 0) {
			//			BLE_unregister();
			//			printf("ble unregistered");
		}
	}
}



