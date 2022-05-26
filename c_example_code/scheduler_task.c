/*
 * scheduler_task.c
 *
 *  Created on: May 14, 2019
 *      Author: mmHgAdmin
 */

/*MCU includes*/
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_adc16.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_common.h"
#include "fsl_uart.h"
#include "fsl_rtc.h"
#include "fsl_dspi.h"
#include "fsl_llwu.h"
#include "fsl_rcm.h"
#include "fsl_pmc.h"
#include "fsl_smc.h"
#include "fsl_lptmr.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "complex.h"
#include "hal_mcu_spi.h"
#include "spi_wireless_task.h"
#include "hal_general.h"
#include "UART_task.h"
#include "button_task.h"
#include "scheduler_task.h"
#include "UART_task.h"
#include "flash.h"
#include "timer_isr.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"


void amb_tot_meas_cal (void){
	RTC_GetDatetime(RTC, &Start_series);
	//calculation of daytime and nighttime
	//assumption the app gives daytime offset as (current_time+offset)= (daytime for next day);
	date_time_seconds=(RTC_ConvertDatetimeToSeconds(&Start_series)+((uint32_t)DayStrtOff));
	RTC_ConvertSecondsToDatetime(date_time_seconds,&Day_time);
	date_time_seconds=(RTC_ConvertDatetimeToSeconds(&Start_series)+((uint32_t)NightStrtOff));
	RTC_ConvertSecondsToDatetime(date_time_seconds,&Night_time);

	Day_time_seconds=(((((Day_time.hour)*60)+Day_time.minute)*60)+Day_time.second);
	Night_time_seconds=(((((Night_time.hour)*60)+Night_time.minute)*60)+Night_time.second);
	Start_time_seconds=(((((Start_series.hour)*60)+Start_series.minute)*60)+Start_series.second);


	if (((Day_time_seconds<Start_time_seconds)||(Day_time_seconds==Start_time_seconds))&&(Start_time_seconds<Night_time_seconds))
	{
		Day_time_left= (Night_time_seconds/60)-(Start_time_seconds/60);
	}
	else if (((Night_time_seconds<Start_time_seconds)||(Night_time_seconds==Start_time_seconds))&&(Start_time_seconds<24)){
		Night_time_left=(24-(Start_time_seconds/60))+(Day_time_seconds/60);
	}
	else if ((Start_time_seconds>0)&&(Start_time_seconds<Day_time_seconds)){
		Night_time_left=(Day_time_seconds/60)-(Start_time_seconds/60);
	}

	Tot_amb_meas=round((Day_time_left/DayFreq)+(Night_time_left/NightFreq));

	if (MeasSetLength<=0x06){
		Data_trans_freq=6;
		Data_trans_freq_seconds=Start_time_seconds+Data_trans_freq;//store this in separate location and keep updating it
	}
	else
	{
		Data_trans_freq=(uint32_t)MeasSetLength;
		Data_trans_freq_seconds=Start_time_seconds+Data_trans_freq;
	}


	//write the Data_trans_freq_seconds in its mem location
	//Write enawireless command
	flash_write_enawireless();

	transfer_length_flash=9;
	split_var_four=convertFrom32To8(Data_trans_freq_seconds);
	sendbuf_flash[5] =split_var_four[0];
	sendbuf_flash[6] =split_var_four[1];
	sendbuf_flash[7] =split_var_four[2];
	sendbuf_flash[8] =split_var_four[3];

	flash_page_write(Meas_info_data_trans_freq_seconds, transfer_length_flash);
	//Write disawireless command-for the data
	flash_write_disawireless();

	//Write command-for the ambulatory measurement metrics
	//Write enawireless command
	flash_write_enawireless();
	//Page program command-for the data
	transfer_length_flash=19;
	split_var_four=convertFrom32To8(Day_time_seconds);
	sendbuf_flash[5] =split_var_four[0];
	sendbuf_flash[6] =split_var_four[1];
	sendbuf_flash[7] =split_var_four[2];
	sendbuf_flash[8] =split_var_four[3];
	split_var_four=convertFrom32To8(Night_time_seconds);
	sendbuf_flash[9] =split_var_four[0];
	sendbuf_flash[10] =split_var_four[1];
	sendbuf_flash[11] =split_var_four[2];
	sendbuf_flash[12] =split_var_four[3];
	split_var_four=convertFrom32To8(Data_trans_freq);
	sendbuf_flash[13] =split_var_four[0];
	sendbuf_flash[14] =split_var_four[1];
	sendbuf_flash[15] =split_var_four[2];
	sendbuf_flash[16] =split_var_four[3];
	sendbuf_flash[17] =Tot_amb_meas;
	sendbuf_flash[18] =MeasSetLength;
	flash_page_write(Meas_info_ambulatory, transfer_length_flash);
	//Write disawireless command-for the data
	flash_write_disawireless();


}

void S_BUTTON_POW_OFF_BIT_function (void){
	/* Power off if due to BLE timeouts	*/
	if (Time_out_power_off==1){
		Time_out_power_off=0;
		PRINTF("Existing from LPM");
		xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
	}
	else{
		/* Power off if due to LLWU ISR (required for appropriate power on) not exited within appropriate time and is just enawirelessd due to accidental touch*/
		if (LLWU_TRIG==200){
			LLWU_TRIG=0;
			xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
		}
		/* Power off if due to Power off button ISR	*/
		else{
			uint32_t power_pin_read=0;
			Pow_Time_ticks_2=xTaskGetTickCount();
			power_pin_read=GPIO_ReadPinInput(BOARD_POW_SW_GPIO, BOARD_POW_SW_GPIO_PIN);
			PRINTF("\r\n power_pin_read value %d\r\n",power_pin_read);
			PRINTF("\r\n Pow_Time_ticks_1 value %d\r\n",Pow_Time_ticks_1);
			PRINTF("\r\n Pow_Time_ticks_2 value %d\r\n",Pow_Time_ticks_2);
			g_PowButtonPress = false;
			while(1){
				//probably include some case to check if pause can be implemented here
				Pow_Time_ticks_2=xTaskGetTickCount();
				power_pin_read=GPIO_ReadPinInput(BOARD_POW_SW_GPIO, BOARD_POW_SW_GPIO_PIN);
				if(((Pow_Time_ticks_2-Pow_Time_ticks_1)<=1000)&& (power_pin_read==1))
				{
					if ((Pow_Time_ticks_2-Pow_Time_ticks_1)==1000)
					{
						PRINTF("\r\n Pow_Time_ticks_1 value %d\r\n",Pow_Time_ticks_1);
						PRINTF("\r\n Pow_Time_ticks_2 value %d\r\n",Pow_Time_ticks_2);
						powButtontimer_sel=1;
						break;
					}
				}
				else if(((Pow_Time_ticks_2-Pow_Time_ticks_1)<1000)&& (power_pin_read==0))
				{
					PRINTF("Could not enter the low power mode");
					powButtontimer_sel=2;
					break;
				}

			}
			if(powButtontimer_sel==1) {
				g_PowButtonPress = false;
				Pow_Time_ticks_1=0;
				Pow_Time_ticks_2=0;
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
			}
			else if (powButtontimer_sel==2)
			{
				g_PowButtonPress = false;
				Pow_Time_ticks_1=0;
				Pow_Time_ticks_2=0;
				//xEventGroupSetBits(button_event_group, BIT0);// decide to do what if power button to turn off was pressed unintentionally
			}
		}

	}
}

void S_BUTTON_CMD_RECV_BIT_function (void){
	//PRINTF("\r\n Cmd_Time_ticks_1 value %d\r\n",Cmd_Time_ticks_1);
	uint32_t power_pin_read=0;
	Cmd_Time_ticks_2=xTaskGetTickCount();
	power_pin_read=GPIO_ReadPinInput(BOARD_CMD_SW_GPIO, BOARD_CMD_SW_GPIO_PIN);
	//PRINTF("\r\n Cmd_Time_ticks_2 value %d\r\n",Cmd_Time_ticks_2);

	while(1){
		Cmd_Time_ticks_2=xTaskGetTickCount();
		power_pin_read=GPIO_ReadPinInput(BOARD_CMD_SW_GPIO, BOARD_CMD_SW_GPIO_PIN);

		/*Short and long hold single button press for starting and stopping measurement and event measurement respectively*/

		/*	 Starting and Stopping measurement*/
		if (((Cmd_Time_ticks_2-Cmd_Time_ticks_1)<=1000)&& ((Cmd_Time_ticks_2-Cmd_Time_ticks_1)>=950)&& (power_pin_read==1))
		{
			g_CmdButtonPress = false;
			Cmd_Time_ticks_1=0;
			Cmd_Time_ticks_2=0;
			MeasSetType=0x04;
			MeasSetLength=0x01;
			//store the meastype
			// store the measurement info in to the flash

			//Write command-for the data
			//Write enawireless command
			flash_write_enawireless();

			//Page program command-for the data
			transfer_length_flash=27;
			for (int i=1; i<(transfer_length_flash+1-5);i++){
				sendbuf_flash[5+i-1] =0x00;
			}
			sendbuf_flash[15] =MeasSetType;
			sendbuf_flash[16] =MeasSetLength;
			flash_page_write(Meas_info_from_isource, transfer_length_flash);

			//Write disawireless command-for the data
			flash_write_disawireless();

			//test if the memory is full and then start measurement the measurement just like checking the meas_info in S_BLE_RECV_CMD

			//Read command -for the current address(last stored sector start value)
			transfer_length_flash=9;
			flash_read(Meas_pc_start, transfer_length_flash);
			mem_current_addr=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

			//Read command -for the erase pointer address(last stored sector start value)
			transfer_length_flash=9;
			flash_read(Meas_erase_pointer_iden, transfer_length_flash);
			mem_erase_pointer=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);


			mem_storage_left=(uint8_t)(Meas_assigned_end-mem_current_addr+mem_erase_pointer);
			MeasSetLength=1;

			/* Space availawireless for storing a measurement*/
			if (mem_storage_left>=MeasSetLength){
				/* Start/stop a measurement*/
				StatusVarFlags=MEAS_STAT_FLAG_NORM;
				RTC_GetDatetime(RTC, &Measurement_Start_time);
				xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT); /* Measurement task will decide to Start/stop a measurement*/

			}
			/* No space for storing a measurement*/
			else if (mem_storage_left<MeasSetLength){

				meas_full_indicator=1;
				xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
			}
			break;
		}


		/*Event measurement*/
		if (((Cmd_Time_ticks_2-Cmd_Time_ticks_1)<=2000) && ((Cmd_Time_ticks_2-Cmd_Time_ticks_1)>1950) && (power_pin_read==1))
		{

			//Read command-stored memory location address for the Meas_info
			transfer_length_flash=17;
			flash_read(Meas_info_from_isource, transfer_length_flash);
			Amb_meas_strtd= recvbuf_flash[16];

			if (Amb_meas_strtd==0x02)
			{
				StatusVarFlags=MEAS_STAT_FLAG_EVENT;
				// may not need the if condition since the global variawirelesss: inflation_state_zero_variawireless, deflation_state_zero_variawireless takes care of if to stop or start may not need the measurement stop bit too since the measurement is stopped from looking at these variawirelesss
				g_CmdButtonPress = false;
				Cmd_Time_ticks_1=0;
				Cmd_Time_ticks_2=0;
				RTC_GetDatetime(RTC, &Measurement_Start_time);
				xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);
				break;
			}
		}

		/*Douwireless button press for stopping ambulatory measurement*/
		else if(((Cmd_Time_ticks_2-Cmd_Time_ticks_1)>=3000) && ((Cmd_Time_ticks_2-Cmd_Time_ticks_1)>2950)&& (power_pin_read==1))
		{
			break;

		}

		/*Command button pressed and not held for any of these times*/
		else if (power_pin_read==0)
		{
			//PRINTF("Could not start measurement");
			g_CmdButtonPress = false;
			Cmd_Time_ticks_1=0;
			Cmd_Time_ticks_2=0;
			break;

		}
	}

}

void S_BUTTON_PAUSE_CMD_BIT_function (void){

	uint32_t power_pin_read_2=0;//POW is sw2
	Pow_Time_ticks_2=xTaskGetTickCount();//POW is sw2
	power_pin_read_2=GPIO_ReadPinInput(BOARD_POW_SW_GPIO, BOARD_POW_SW_GPIO_PIN);

	while(1){
		Pow_Time_ticks_2=xTaskGetTickCount();//POW is sw2
		power_pin_read_2=GPIO_ReadPinInput(BOARD_POW_SW_GPIO, BOARD_POW_SW_GPIO_PIN);
		/*Pausing or unpausing ambulatory measurement*/
		if ((((Pow_Time_ticks_2-Pow_Time_ticks_1)<=1000) && ((Pow_Time_ticks_2-Pow_Time_ticks_1)>950)&& (power_pin_read_2==1))){

			//Read command-stored memory location address for the Meas_info
			transfer_length_flash=17;
			flash_read(Meas_info_from_isource, transfer_length_flash);
			Amb_meas_strtd= recvbuf_flash[16];
			if (Amb_meas_strtd==0x02)
			{
				switch(Amb_meas_pause_val)
				{
				case VAL_AMB_MEAS_PAUSE_TRUE:
					Amb_meas_pause_val =false;//unpause the ambulatory
					break;
				case VAL_AMB_MEAS_PAUSE_FALSE:
					Amb_meas_pause_val = true;//pause the ambulatory
					break;
				}
				break;
			}
		}
		/*Stopping ambulatory measurement*/
		else if ((((Pow_Time_ticks_2-Pow_Time_ticks_1)<=2000) && ((Pow_Time_ticks_2-Pow_Time_ticks_1)>1950)&& (power_pin_read_2==1))){

			//Read command-stored memory location address for the Meas_info
			transfer_length_flash=17;
			flash_read(Meas_info_from_isource, transfer_length_flash);
			Amb_meas_strtd= recvbuf_flash[16];
			if (Amb_meas_strtd==0x02)
			{
				//erase the metrics for ambulatory

				// send msg to wireless for sending data
				xEventGroupSetBits(wireless_event_group, SPI_BLE_DATA_SEND_BATT_LOW_BIT);

			}
		}
		/*Not pressed enough*/
		else if (power_pin_read_2==0)
		{
			//PRINTF("Could not start measurement");
			g_CmdButtonPress = false;
			g_PowButtonPress = false;
			Cmd_Time_ticks_1=0;
			Cmd_Time_ticks_2=0;
			Pow_Time_ticks_1=0;
			Pow_Time_ticks_2=0;
			break;

		}

	}

}

void S_BUTTON_PAUSE_LPTMR_BIT_function (void){
	if (Amb_meas_pause_val==true){
		//wait for 1 minute and then try again later
		uint64_t countdown_time=60;
		LPTMR_timer_countdown(countdown_time);
	}
	else if (Amb_meas_pause_val==false){
		//wait for 1 minute and then try again later
		uint64_t countdown_time=10;
		LPTMR_timer_countdown(countdown_time);
	}

}

void S_DEVICE_START_BIT_function (void){
//Sangita: Change this manually to SPI_WIFI_SEQ_NUM_REQ_BIT to test the WIFI sequence
	xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);

}

void S_MEASUREMENT_AMB_SCHEDULE_BIT_function (void){
	//read flash for if it is ambulatory

	//Read command-stored memory location address for the Meas_info
	transfer_length_flash=17;
	flash_read(Meas_info_from_isource, transfer_length_flash);
	Amb_meas_strtd= recvbuf_flash[16];

	if (Amb_meas_strtd==0x02)
	{
		pow_led_amb_sched_meas();
		battery_message_state=1;
		//check with battery values and then revert the power led to either green or red
		xEventGroupSetBits(button_event_group, BUTTON_BATTERY_VAL_BIT);
	}


}

void S_MEASUREMENT_AMB_VAR_STORE_BIT_function (void){
	//store the ambulatory variawireless and then put the module to low power mode
	//scheduler task storing scheduler variawirelesss before going to low power mode and when it exits low power mode and before asking the uart it checks the memory location where the amb values are stored and if not a zero value it will start the rest of the measurment
	xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);

}

void S_MEASUREMENT_START_BIT_function (void){
	if (MeasSetType==0x04){
		MeasSetLength=MeasSetLength-1;

		//Write command-for the MeasSetLength
		//Write enawireless command
		flash_write_enawireless();
		//Page program command-for the data
		transfer_length_flash=6;
		sendbuf_flash[5] =MeasSetLength;
		flash_page_write(Meas_info_set_length, transfer_length_flash);
		//Write disawireless command-for the data
		flash_write_disawireless();

		//Read command -for the current address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_pc_start, transfer_length_flash);
		mem_current_addr=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//Read command -for the erase pointer address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_erase_pointer_iden, transfer_length_flash);
		mem_erase_pointer=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		if ((mem_current_addr==mem_erase_pointer))
		{
			meas_full_indicator=2;
			meas_led_low_mem();
			xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
		}
		else if ((mem_current_addr!=mem_erase_pointer))
		{

			StatusVarFlags=MEAS_STAT_FLAG_NORM;
			RTC_GetDatetime(RTC, &Measurement_Start_time);
			xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);
		}

	}
	else{
		StatusVarFlags=MEAS_STAT_FLAG_NORM;
		RTC_GetDatetime(RTC, &Measurement_Start_time);
		xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);
	}
}

void S_MEASUREMENT_STOP_BIT_function (void){
	//Control system for the ambulatory measurement needs to have the following functionality: should have a timer to wake up during the time the next scheduled measurement; enter low power stop mode and wake with if any hardware command button is pressed do event measurement and store
	//have a control system depending on if the command button pressed to stop, decision for data be either sent to flash or not, decision if data needs to be sent to wireless or not

	//read flash for if it is ambulatory

	//Read command-stored memory location address for the Meas_info
	transfer_length_flash=17;
	flash_read(Meas_info_from_isource, transfer_length_flash);
	MeasSetType= recvbuf_flash[16];

	if ((MeasSetType==0x00)||(MeasSetType==0x02)){
		deflation_state_zero_variawireless=0;
		inflation_state_zero_variawireless=0;
		g_CmdButtonPress = false;
		MeasSetLength=MeasSetLength-1;


		//Write command-for the MeasSetLength
		//Write enawireless command
		flash_write_enawireless();
		//Page program command-for the data
		transfer_length_flash=6;
		sendbuf_flash[5] =MeasSetLength;
		flash_page_write(Meas_info_set_length, transfer_length_flash);
		//Write disawireless command-for the data
		flash_write_disawireless();


		xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
		//here is where the data is sent with sequence number that are missing before that
	}
	else if (MeasSetType==0x01){
		if(Amb_meas_pause_val==false){
			deflation_state_zero_variawireless=0;
			inflation_state_zero_variawireless=0;
			g_CmdButtonPress = false;
			Tot_amb_meas=Tot_amb_meas-1;


			//Write command-for the MeasSetLength
			//Write enawireless command
			flash_write_enawireless();
			//Page program command-for the data
			transfer_length_flash=6;
			sendbuf_flash[5] =Tot_amb_meas;
			flash_page_write(Meas_info_set_length, transfer_length_flash);
			//Write disawireless command-for the data
			flash_write_disawireless();


			RTC_GetDatetime(RTC, &current_time);
			Current_time_seconds=(((((current_time.hour)*60)+current_time.minute)*60)+current_time.second);



			transfer_length_flash=9;
			flash_read(Meas_info_data_trans_freq_seconds, transfer_length_flash);
			Data_trans_freq_seconds=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

			if(Current_time_seconds==Data_trans_freq_seconds){
				Data_trans_freq_seconds=Current_time_seconds+Data_trans_freq;
				//write the Data_trans_freq_seconds in its mem location
				//Write enawireless command
				flash_write_enawireless();
				transfer_length_flash=9;
				split_var_four=convertFrom32To8(Data_trans_freq_seconds);
				sendbuf_flash[5] =split_var_four[0];
				sendbuf_flash[6] =split_var_four[1];
				sendbuf_flash[7] =split_var_four[2];
				sendbuf_flash[8] =split_var_four[3];

				flash_page_write(Meas_info_data_trans_freq_seconds, transfer_length_flash);
				//Write disawireless command-for the data
				flash_write_disawireless();


				Data_trans_reached=21;

				xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);// only difference is that the wireless timer is assigned higher than that of the measurements that has not reached the transfer period
				//here is where the data is sent with sequence number that are missing before that
			}
			else if(Current_time_seconds<=Data_trans_freq_seconds)
			{
				xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);// only difference is that the wireless timer is assigned  than here is lower that of the measurements that has reached the transfer period
				//here is where the data is sent with sequence number that are missing before that


			}

		}
		else if (Amb_meas_pause_val==true){
			uint64_t countdown_time=50;
			LPTMR_timer_countdown(countdown_time);
		}
	}

}

void S_WIRELESS_DATA_RECEIVE_BIT_function (void){

	if (wireless_data_received==1){
		wireless_data_received=0;
		uint64_t countdown_time=1;
		PIT_scheduler_timer(countdown_time);
	}


	else if(wireless_data_received==2)
	{
		wireless_data_received=0;

		//added the following to make sure before starting the measurement making sure if the memory is not full and has space
		//Read command -for the current address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_pc_start, transfer_length_flash);
		mem_current_addr=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//Read command -for the erase pointer address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_erase_pointer_iden, transfer_length_flash);
		mem_erase_pointer=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);


		mem_storage_left=(uint8_t)(Meas_assigned_end-mem_current_addr+mem_erase_pointer);

		if (mem_storage_left>=MeasSetLength){
			//read meas_type and meas_setlength
			//get time data and time stamp and ambulatory-calculate total number of measurements
			if ((MeasSetType==0x00)||(MeasSetType==0x02)){
				RTC_GetDatetime(RTC, &Start_series);
				RTC_GetDatetime(RTC, &Measurement_Start_time);
			}
			else if (MeasSetType==0x01){
				RTC_GetDatetime(RTC, &Measurement_Start_time);
				amb_tot_meas_cal();

			}
			StatusVarFlags=MEAS_STAT_FLAG_NORM;
			xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);

		}
		//	else if  (mem_storage_left<MeasSetLength){
		//		meas_full_indicator=1;
		//		//xEventGroupSetBits(scheduler_event_group, S_FLASH_MAX_MEM_BIT);//scheduler msg to itself
		//		xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
		//	}
	}
}

void S_BLE_DATA_RECEIVE_REQ_BIT_function (void){
	xEventGroupSetBits(wireless_event_group, SPI_BLE_DATA_RECEIVE_BIT);
}

void S_WIFI_DATA_RECEIVE_REQ_BIT_function (void){
	xEventGroupSetBits(wireless_event_group, SPI_WIFI_DATA_RECEIVE_BIT);
}

void S_WIRELESS_SEQ_NUM_REQ_BIT_function (void){
	//calculate the missing data i.e. from the sector number the address to read


	//	//Read command-stored memory location address for the sequence number from app
	//	transfer_length_flash=7;
	//	flash_read(Meas_seq_num_app_iden, transfer_length_flash);
	//	SeqNum_from=convertFrom8To16(recvbuf_flash[5],recvbuf_flash[6]);


	//Read command-stored memory location address for the sequence number
	transfer_length_flash=7;
	flash_read(Meas_seqnum_iden, transfer_length_flash);
	mem_seq_num=convertFrom8To16(recvbuf_flash[5],recvbuf_flash[6]);

	//find diffrence
	mem_seq_num_diff=mem_seq_num-SeqNum_from;

	if((mem_seq_num_diff==0))
	{
		if (ble_req_recv==1){
			xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_DIFF_ZERO_BIT);
		}
		if (wifi_req_recv==1){
			xEventGroupSetBits(wireless_event_group, SPI_WIFI_SEQ_NUM_DIFF_ZERO_BIT);
		}
	}
	else
	{
		battery_message_state=2;
		xEventGroupSetBits(button_event_group, BUTTON_BATTERY_VAL_BIT);
	}

}

void S_WIRELESS_DATA_SEND_BIT_function (void){
	// check if this ambulatory to turn the power LED on for the incoming scheduled measurement

	//read flash for if it is ambulatory

	//Read command-stored memory location address for the Meas_info
	transfer_length_flash=17;
	flash_read(Meas_info_from_isource, transfer_length_flash);
	MeasSetType= recvbuf_flash[16];

	if ((MeasSetType==0x00)||(MeasSetType==0x02)){

		if(MeasSetLength!=0){
			const TickType_t xDelay = 10000 / portTICK_PERIOD_MS; // this should be tested with the DUO_ksdk_2.0_ESP32_MK22_BLE/ DUO_ksdk_2.0_ESP32_MK22_WIFI projects -test complete and it works if the button task is in wait mode
			vTaskDelay( xDelay );
			StatusVarFlags=MEAS_STAT_FLAG_NORM;
			RTC_GetDatetime(RTC, &Measurement_Start_time);
			xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);
		}
		else if(MeasSetLength==0){


			//erase the Meas_info_from_isource sector
			flash_write_enawireless();
			flash_sector_erase(Meas_info_from_isource);
			flash_write_disawireless();

			//wait for 3 minutes
			RTOS_delay_ms(180000);
			xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
		}
	}
	else if (MeasSetType==0x01){
		if (Amb_meas_pause_val==0){
			if(Tot_amb_meas!=0){
				RTC_GetDatetime(RTC, &current_time);
				Current_time_seconds=(((((current_time.hour)*60)+current_time.minute)*60)+current_time.second);
				uint64_t countdown_time=0;
				if (((Current_time_seconds>Day_time_seconds) || (Current_time_seconds==Day_time_seconds))&& (Current_time_seconds<Night_time_seconds)){
					countdown_time= DayFreq*60;
				}
				else if (((Night_time_seconds<Current_time_seconds)||(Night_time_seconds==Current_time_seconds))&&(Current_time_seconds<24)){
					countdown_time= NightFreq*60;
				}
				else if ((Current_time_seconds>0)&&(Current_time_seconds<Day_time_seconds)){
					countdown_time= NightFreq*60;
				}
				Amb_LPTMR_set=1;
				LPTMR_timer_countdown(countdown_time);
				//enter very low power wait/stop mode
				SMC_PreEnterStopModes();
				SMC_SetPowerModeVlps(SMC);
				SMC_PostExitStopModes();

			}
			else if(Tot_amb_meas==0){
				//erase the Meas_info_from_isource sector (this erase even the ambulatory information since they are in thin sector 8147

				flash_write_enawireless();
				flash_sector_erase(Meas_info_from_isource);
				flash_write_disawireless();

				//wait for 3 minutes
				RTOS_delay_ms(180000);
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);

			}
		}

	}

}

void S_BLE_TIME_OUT_BIT_function (void){
	conn_led_wireless_noconn();
	if ((Time_out_cmd==1) && (Time_out_count==1))
	{
		//wait for 1 minute and then try again later
		uint64_t countdown_time=60;
		PIT_BLE_timer(countdown_time);
	}

	else if ((Time_out_cmd==1) && (Time_out_count==2)){
		//wait for 3 minutes and if nothing is turned on the power is off
		uint64_t countdown_time=180;
		PIT_BLE_timer(countdown_time);


	}
	else if ((Time_out_cmd!=1)&& (Time_out_cmd!=0)){
		//wait for 1 minute and then try again later
		uint64_t countdown_time=60;
		PIT_BLE_timer(countdown_time);
	}
}

void S_BLE_TIME_OUT_PIT_BIT_function (void){
	if ((Time_out_cmd==1) && (Time_out_count==1))
	{
		xEventGroupSetBits(wireless_event_group, SPI_BLE_DATA_RECEIVE_BIT);
	}

	else if ((Time_out_cmd==1) && (Time_out_count==2)){
		Time_out_count=0;
		Time_out_cmd=0;
		xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
	}

	else if ((Time_out_cmd==2) && (Time_out_count==1)){
		//read the meastype from Meas_info_from_isource

		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];

		//depending on measurement type I do the following
		if ((Time_out_meas_type==0x00)&&(Time_out_meas_type==0x02)&&(Time_out_meas_type==0x04)){

			xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);

		}
		else if (Time_out_meas_type==0x01){
			transfer_length_flash=9;
			flash_read(Meas_info_data_trans_freq_seconds, transfer_length_flash);
			Data_trans_freq_seconds=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);


			RTC_GetDatetime(RTC, &current_time);
			Current_time_seconds=(((((current_time.hour)*60)+current_time.minute)*60)+current_time.second);
			if(Current_time_seconds==Data_trans_freq_seconds){
				Data_trans_freq_seconds=Current_time_seconds+Data_trans_freq;
				Data_trans_reached=21;
				xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
			}
			else if(Current_time_seconds<=Data_trans_freq_seconds)
			{
				//wait for 10 sec and from LPTMR send a msg to scheduler to perform S_BLE_SEND_BIT
				uint64_t countdown_time=10;
				PIT_BLE_timer(countdown_time);
			}

		}

	}
	else if ((Time_out_cmd==2) && (Time_out_count==2)){
		//read the meastype from Meas_info_from_isource

		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];
		if ((Time_out_meas_type==0x00)&&(Time_out_meas_type==0x02)&&(Time_out_meas_type==0x04)){
			Time_out_count=0;
			Time_out_cmd=0;
			xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
		}
		else if ((Time_out_meas_type==0x01)){
			//Read command-stored memory location address for the Meas_info_set_length
			transfer_length_flash=6;
			flash_read(Meas_info_set_length, transfer_length_flash);
			Time_out_meas_length=recvbuf_flash[5];
			if(Time_out_meas_length==0){
				Time_out_count=0;
				Time_out_cmd=0;
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
			}
			else if (Time_out_meas_length!=0){
				//wait for 10 sec and from LPTMR send a msg to scheduler to perform S_BLE_SEND_BIT
				uint64_t countdown_time=10;
				PIT_BLE_timer(countdown_time);
			}

		}

	}
	else if ((Time_out_cmd==3) && (Time_out_count==1)){
		xEventGroupSetBits(wireless_event_group, SPI_BLE_SEND_MISS_DATA_BIT);

	}
	else if (((Time_out_cmd==3) && (Time_out_count==2))||((Time_out_cmd==4) && (Time_out_count==2))){

		//Read command -for the current address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_pc_start, transfer_length_flash);
		mem_current_addr=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//Read command -for the erase pointer address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_erase_pointer_iden, transfer_length_flash);
		mem_erase_pointer=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//read the meastype from Meas_info_from_isource
		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];
		if (mem_current_addr==mem_erase_pointer){
			meas_led_low_mem();
			uint64_t countdown_time=30;
			LPTMR_timer_countdown(countdown_time);

		}
		else if (mem_current_addr!=mem_erase_pointer){
			//read the Time_out_meas_length
			//Read command-stored memory location address for the Meas_info_set_length
			transfer_length_flash=6;
			flash_read(Meas_info_set_length, transfer_length_flash);
			Time_out_meas_length=recvbuf_flash[5];
			if (Time_out_meas_length!=0){
				uint64_t countdown_time=10;
				LPTMR_timer_countdown(countdown_time);
			}
			else if (Time_out_meas_length==0){
				Time_out_count=0;
				Time_out_cmd=0;
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
			}

		}
	}

	else if ((Time_out_cmd==4) && (Time_out_count==1))
	{
		xEventGroupSetBits(wireless_event_group, SPI_BLE_DATA_SEND_BATT_LOW_BIT);
	}

}

void S_WIFI_TIME_OUT_BIT_function (void){
	conn_led_wireless_noconn();
	if ((Time_out_cmd==1) && (Time_out_count==1))
	{
		//wait for 1 minute and then try again later
		uint64_t countdown_time=60;
		PIT_WIFI_timer(countdown_time);
	}

	else if ((Time_out_cmd==1) && (Time_out_count==2)){
		//wait for 3 minutes and if nothing is turned on the power is off
		uint64_t countdown_time=180;
		PIT_WIFI_timer(countdown_time);


	}
	else if ((Time_out_cmd!=1)&& (Time_out_cmd!=0)){
		//wait for 1 minute and then try again later
		uint64_t countdown_time=60;
		PIT_WIFI_timer(countdown_time);
	}
}

void S_WIFI_TIME_OUT_PIT_BIT_function (void){
	if ((Time_out_cmd==1) && (Time_out_count==1))
	{
		xEventGroupSetBits(wireless_event_group, SPI_WIFI_DATA_RECEIVE_BIT);
	}

	else if ((Time_out_cmd==1) && (Time_out_count==2)){
		Time_out_count=0;
		Time_out_cmd=0;
		xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
	}

	else if ((Time_out_cmd==2) && (Time_out_count==1)){
		//read the meastype from Meas_info_from_isource

		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];

		//depending on measurement type I do the following
		if ((Time_out_meas_type==0x00)&&(Time_out_meas_type==0x02)&&(Time_out_meas_type==0x04)){

			xEventGroupSetBits(wireless_event_group, SPI_WIFI_SEQ_NUM_REQ_BIT);

		}
		else if (Time_out_meas_type==0x01){
			transfer_length_flash=9;
			flash_read(Meas_info_data_trans_freq_seconds, transfer_length_flash);
			Data_trans_freq_seconds=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);


			RTC_GetDatetime(RTC, &current_time);
			Current_time_seconds=(((((current_time.hour)*60)+current_time.minute)*60)+current_time.second);
			if(Current_time_seconds==Data_trans_freq_seconds){
				Data_trans_freq_seconds=Current_time_seconds+Data_trans_freq;
				Data_trans_reached=21;
				xEventGroupSetBits(wireless_event_group, SPI_WIFI_SEQ_NUM_REQ_BIT);
			}
			else if(Current_time_seconds<=Data_trans_freq_seconds)
			{
				//wait for 10 sec and from LPTMR send a msg to scheduler to perform S_BLE_SEND_BIT
				uint64_t countdown_time=10;
				PIT_BLE_timer(countdown_time);
			}

		}

	}
	else if ((Time_out_cmd==2) && (Time_out_count==2)){
		//read the meastype from Meas_info_from_isource

		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];
		if ((Time_out_meas_type==0x00)&&(Time_out_meas_type==0x02)&&(Time_out_meas_type==0x04)){
			Time_out_count=0;
			Time_out_cmd=0;
			xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
		}
		else if ((Time_out_meas_type==0x01)){
			//Read command-stored memory location address for the Meas_info_set_length
			transfer_length_flash=6;
			flash_read(Meas_info_set_length, transfer_length_flash);
			Time_out_meas_length=recvbuf_flash[5];
			if(Time_out_meas_length==0){
				Time_out_count=0;
				Time_out_cmd=0;
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
			}
			else if (Time_out_meas_length!=0){
				//wait for 10 sec and from LPTMR send a msg to scheduler to perform S_BLE_SEND_BIT
				uint64_t countdown_time=10;
				PIT_BLE_timer(countdown_time);
			}

		}

	}
	else if ((Time_out_cmd==3) && (Time_out_count==1)){
		xEventGroupSetBits(wireless_event_group, SPI_WIFI_SEND_MISS_DATA_BIT);

	}
	else if (((Time_out_cmd==3) && (Time_out_count==2))||((Time_out_cmd==4) && (Time_out_count==2))){

		//Read command -for the current address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_pc_start, transfer_length_flash);
		mem_current_addr=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//Read command -for the erase pointer address(last stored sector start value)
		transfer_length_flash=9;
		flash_read(Meas_erase_pointer_iden, transfer_length_flash);
		mem_erase_pointer=convertFrom8To32(recvbuf_flash[5],recvbuf_flash[6],recvbuf_flash[7],recvbuf_flash[8]);

		//read the meastype from Meas_info_from_isource
		//Read command-stored memory location address for the Meas_info
		transfer_length_flash=17;
		flash_read(Meas_info_from_isource, transfer_length_flash);
		Time_out_meas_type= recvbuf_flash[15];
		if (mem_current_addr==mem_erase_pointer){
			meas_led_low_mem();
			uint64_t countdown_time=30;
			LPTMR_timer_countdown(countdown_time);

		}
		else if (mem_current_addr!=mem_erase_pointer){
			//read the Time_out_meas_length
			//Read command-stored memory location address for the Meas_info_set_length
			transfer_length_flash=6;
			flash_read(Meas_info_set_length, transfer_length_flash);
			Time_out_meas_length=recvbuf_flash[5];
			if (Time_out_meas_length!=0){
				uint64_t countdown_time=10;
				LPTMR_timer_countdown(countdown_time);
			}
			else if (Time_out_meas_length==0){
				Time_out_count=0;
				Time_out_cmd=0;
				xEventGroupSetBits(button_event_group, BUTTON_POWER_OFF_BIT);
			}

		}
	}

	else if ((Time_out_cmd==4) && (Time_out_count==1))
	{
		xEventGroupSetBits(wireless_event_group, SPI_WIFI_DATA_SEND_BATT_LOW_BIT);
	}

}


void S_BLE_SEQ_REQ_SEND_BIT_function (void){

	xEventGroupSetBits(wireless_event_group, SPI_BLE_SEQ_NUM_REQ_BIT);
}

void S_UART_DATA_RECEIVE_BIT_function (void){
	xEventGroupSetBits(uart_event_group, UART_DATA_RECEIVE_BIT);
}

void S_UART_DATA_SEND_BIT_function (void){
	xEventGroupSetBits(uart_event_group, UART_DATA_SEND_BIT);
}


void scheduler_task(void *pvParameters)
{
	if (LLWU_TRIG==100)
	{
		// start the power button task to check for battery value and turn led accordingly
		//read the battery value and time of that recording  from flash (question : rtc how to initialize and maintain time?)
		//read current rtc value and the battery value and get the time difference
		//calculate battery drop and battery left with these info
		// update the flash with the new time and the battery values
		LLWU_TRIG=0;
		//xEventGroupSetBits(wireless_event_group, SPI_BLE_DATA_RECEIVE_BIT);// power on first thing to do : may be BLE receive
		//xEventGroupSetBits(scheduler_event_group, S_DEVICE_START_BIT);
		//xEventGroupSetBits(measurement_event_group, MEASUREMENT_START_BIT);
		//xEventGroupSetBits(uart_event_group, UART_DATA_RECEIVE_BIT);
		xEventGroupSetBits(wireless_event_group, SPI_WIFI_SEQ_NUM_REQ_BIT);
	}
	else if (LLWU_TRIG==200)
	{
		xEventGroupSetBits(scheduler_event_group, S_BUTTON_POW_OFF_BIT); // power off again as the power button accidentally pressed so go back to sleep
	}

	for(;;){
		schedulerEventMask = xEventGroupWaitBits(scheduler_event_group, (S_BATTERY_VAL_BIT |S_WIRELESS_DATA_SEND_BIT|S_WIRELESS_DATA_RECEIVE_BIT|S_BLE_DATA_RECEIVE_REQ_BIT|S_WIFI_DATA_RECEIVE_REQ_BIT|S_BLE_TIME_OUT_BIT|S_BLE_TIME_OUT_PIT_BIT|S_WIFI_TIME_OUT_BIT|S_WIFI_TIME_OUT_PIT_BIT|S_WIRELESS_SEQ_NUM_REQ_BIT|S_DEVICE_START_BIT|S_BLE_SEQ_REQ_SEND_BIT|S_MEASUREMENT_START_BIT|S_MEASUREMENT_STOP_BIT|S_MEASUREMENT_AMB_SCHEDULE_BIT|S_MEASUREMENT_AMB_VAR_STORE_BIT|S_UART_DATA_RECEIVE_BIT|S_UART_DATA_SEND_BIT|S_BUTTON_POW_OFF_BIT|S_BUTTON_CMD_RECV_BIT|S_BUTTON_PAUSE_CMD_BIT|S_BUTTON_PAUSE_LPTMR_BIT), CLEAR, NOWAIT,portMAX_DELAY);

		if ((schedulerEventMask & S_BUTTON_POW_OFF_BIT) != 0) {
			S_BUTTON_POW_OFF_BIT_function ();
		}

		if ((schedulerEventMask & S_BUTTON_CMD_RECV_BIT) != 0) {
			S_BUTTON_CMD_RECV_BIT_function ();
		}

		if ((schedulerEventMask & S_BUTTON_PAUSE_LPTMR_BIT) != 0) {
			S_BUTTON_PAUSE_LPTMR_BIT_function ();
		}

		if ((schedulerEventMask & S_DEVICE_START_BIT) != 0) {
			S_DEVICE_START_BIT_function ();
		}

//		if ((schedulerEventMask & S_DEVICE_START_BIT) != 0) {
//			S_DEVICE_START_BIT_function ();
//		}

		if ((schedulerEventMask & S_MEASUREMENT_AMB_SCHEDULE_BIT) != 0) {
			S_MEASUREMENT_AMB_SCHEDULE_BIT_function ();

		}
		if ((schedulerEventMask & S_MEASUREMENT_AMB_VAR_STORE_BIT) != 0) {
			S_MEASUREMENT_AMB_VAR_STORE_BIT_function ();
		}

		if ((schedulerEventMask & S_MEASUREMENT_START_BIT) != 0) {
			S_MEASUREMENT_START_BIT_function ();
		}

		if ((schedulerEventMask & S_MEASUREMENT_STOP_BIT) != 0) {
			S_MEASUREMENT_STOP_BIT_function ();
		}

		if ((schedulerEventMask & S_WIRELESS_DATA_RECEIVE_BIT) != 0) {
			S_WIRELESS_DATA_RECEIVE_BIT_function();
		}

		if ((schedulerEventMask & S_BLE_DATA_RECEIVE_REQ_BIT) != 0) {
			S_BLE_DATA_RECEIVE_REQ_BIT_function();
		}

		if ((schedulerEventMask & S_WIFI_DATA_RECEIVE_REQ_BIT) != 0) {
			S_WIFI_DATA_RECEIVE_REQ_BIT_function();
		}

		if ((schedulerEventMask & S_WIRELESS_SEQ_NUM_REQ_BIT) != 0) {
			S_WIRELESS_SEQ_NUM_REQ_BIT_function ();
		}

		if ((schedulerEventMask & S_WIRELESS_DATA_SEND_BIT) != 0) {
			S_WIRELESS_DATA_SEND_BIT_function ();
		}

		if ((schedulerEventMask & S_BLE_TIME_OUT_BIT) != 0) {
			S_BLE_TIME_OUT_BIT_function ();
		}

		if ((schedulerEventMask & S_BLE_TIME_OUT_PIT_BIT) != 0) {
			S_BLE_TIME_OUT_PIT_BIT_function ();
		}

		if ((schedulerEventMask & S_WIFI_TIME_OUT_BIT) != 0) {
			S_WIFI_TIME_OUT_BIT_function ();
		}

		if ((schedulerEventMask & S_WIFI_TIME_OUT_PIT_BIT) != 0) {
			S_WIFI_TIME_OUT_PIT_BIT_function ();
		}

		if ((schedulerEventMask & S_BLE_SEQ_REQ_SEND_BIT) != 0) {
			S_BLE_SEQ_REQ_SEND_BIT_function ();
		}

		if ((schedulerEventMask & S_UART_DATA_RECEIVE_BIT) != 0) {
			S_UART_DATA_RECEIVE_BIT_function ();
		}

		if ((schedulerEventMask & S_UART_DATA_SEND_BIT) != 0) {
			S_UART_DATA_SEND_BIT_function ();
		}
	}

}



