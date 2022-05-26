#include "PimManager.h"
#include "TimeHelper.h"
#include "GlobalState.h"

//#define LED_BITMAP_RED			(uint32_t)0xFFFF0000
//#define LED_BITMAP_GREEN		(uint32_t)0x0000FFFF
#define LED_BITMAP_RED			0xFFFF0000
#define LED_BITMAP_GREEN		0x0000FFFF

#define SEQUENCES_NUMBER		32
	
PimManager::PimManager(Ds2408 * const ds2408,
	Buzzer * const buzzer,
	PowerManager * const powerManager,	
	SerialMessageSender &externalMessageSender,
	GPIO_Pin_Status(* getEmergencyStatus)(void),
	/*bool (* const getLockedStatus)(void), GPRS_STATE(* const getGprsState)(void), */
	GPS_STATE(* const getGPSState)(void),
	bool (* const isPanicReset)(void))
	: ds2408(ds2408)
	, buzzer(buzzer)
	, powerManager(powerManager)	
	, externalMessageSender(externalMessageSender)
	, settingsController(&SettingsController::GetInstance())
	, getEmergencyStatus(getEmergencyStatus)
	//, getLockedStatus(getLockedStatus)
	//, getGprsState(getGprsState)
	, getGPSState(getGPSState)
	, isPanicReset(isPanicReset)
{
	emergButtonState = (GPIO_Pin_Status)!getEmergencyStatus();
  	
	// Initialise the ports of the PimManager: all off
	pimData.pim_ports = 0xFF;        // all off
	pimData.pim_ports &= ~PORT_RELAY1_DRV_BITMASK;    // except relay 1
	ds2408->PioDs2408(1, pimData.pim_ports, &pioState);      // the actual state of the ports is inverted

	// Calibrate and initialize the ADC modules
	//Hal_Start_Calibrate();
		
	osSemaphoreDef_t semaphoreDef;	
	semaphore = osSemaphoreCreate(&semaphoreDef, 1); 
	osSemaphoreRelease(semaphore);
	
	pimLeds = 0;
	overrideDelay = 0;
	ledBitmapRed = LED_BITMAP_RED;
	ledBitmapGreen = LED_BITMAP_GREEN;
	
	isCommRegistered = false;
	lockedStatus = false;
}

/*******************************************************************************
 Function         : CheckEmergency
 Description      : Checks if emergency button is pressed
 Parameters       : none
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::CheckEmergency(void)
{
	auto temp_state = (GPIO_Pin_Status)!getEmergencyStatus(); 
	
	if (temp_state != emergButtonState)
	{
		if (temp_state == ON)
		{
			externalMessageSender.DiagPrintf(DIAG_PIM, 1, "Emergency Button Pressed");
		}
		else
		{
			externalMessageSender.DiagPrintf(DIAG_PIM, 1, "Emergency Button Released");
		}

		emergButtonState = temp_state;
	}
}

/*******************************************************************************
 Function         : CheckLedStatus
 Description      : Checks statuses and set blinking pattern
 Parameters       : none
 Return           : none
 Globals Accessed : no
 *******************************************************************************/
void PimManager::CheckLedStatus(void)
{	
	
	lockedStatus = GlobalState::GetInstance().GetLockedStatus();
	
	if (lockedStatus == true || pimSettings.pim_should_blink == false)
	{
		SetLedPattern(NO_RED_NO_GREEN);
	}
	else if (disableLeds == false)
	{
		GPRS_STATE gprsState = GlobalState::GetInstance().GetGprsState();
		if (gprsState == GPRS_CONNECTED)
		{ 
			isCommRegistered = CellState::GetInstance().IsCommRegistered();
			if (isCommRegistered == true)
			{
				if (getGPSState() == GPS_VALID)
				{
					SetLedPattern(NO_RED_SLOW_GREEN);
				}
				else
				{        
					SetLedPattern(SLOW_RED_GREEN_CHASER);
				}
			}
			else
			{
				SetLedPattern(FAST_RED_GREEN_CHASER);
			}
		}
		else if (gprsState == GPRS_CONNECTING)
		{
			SetLedPattern(SLOW_RED_NO_GREEN);
		}
		else if (gprsState == GPRS_FAILED)
		{        
			SetLedPattern(FAST_RED_NO_GREEN);			
		}
	}  
}

/*******************************************************************************
 Function         : CheckLeds
 Description      : Checks if emergency button is pressed
 Parameters       : none
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::CheckLeds(uint8_t cycles_count)
{
	
	uint8_t red_bit, green_bit;
	if (osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
	
//		uint8_t pos = ((sizeof(LED_BITMAP_RED) * 8) - 1) - cycles_count;
//		red_bit = (LED_BITMAP_RED >> pos) & 0x1;
//		green_bit = (LED_BITMAP_GREEN >> pos) & 0x1;
	
		uint32_t pos = ((sizeof(ledBitmapRed) * 8) - 4) - cycles_count;
		red_bit = (ledBitmapRed >> pos) & 0x1;
		green_bit = (ledBitmapGreen >> pos) & 0x1;
		
		osSemaphoreRelease(semaphore);		
	}
	
	if(red_bit)
	{
		// turn off the green led
		pimLeds &= ~PORT_LED_GREEN;

		// turn on the red led
		pimLeds |= PORT_LED_RED;
	}
	else if(green_bit)
	{
		// turn off the red led
		pimLeds &= ~PORT_LED_RED;

		// turn on the GREEN led
		pimLeds |= PORT_LED_GREEN;
	}
	else
	{
		// turn off both leds
		pimLeds &= ~PORT_LED_GREEN;
		pimLeds &= ~PORT_LED_RED;
	}	
}


/*******************************************************************************
 Function         : CheckBuzzer
 Description      : Checks Buzzer
 Parameters       : none
 Return           : none
 Globals Accessed : No
 *******************************************************************************/
void PimManager::CheckBuzzer(void)
{
	// buzzer:
	// still repetition cycles to do?
	if(pimData.buzzer_repeat)
	{
		// buzzer pulses finished?
		if(pimData.buzzer_pulses_count == 0)
		{
			// delay finished?
			if(pimData.buzzer_repeat_delay_count == 0)
			{
				// decrement the repetition counter 
				// reload the pulses counter and delay counter if there are still cycles to do 
				if(--pimData.buzzer_repeat)
				{
					pimData.buzzer_pulses_count = pimData.buzzer_pulses;
					pimData.buzzer_repeat_delay_count = pimData.buzzer_repeat_delay;
				}
			}
			else
			{
				pimData.buzzer_repeat_delay_count--;
			}
		}
	}
}

/*******************************************************************************
 Function         : GetIsBuzzing
 Description      : Checks if it is buzzing
 Parameters       : none
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
bool PimManager::GetIsBuzzing(void)
{
	return (pimData.buzzer_repeat > 0 || pimData.buzzer_pulses_count > 0);
}

/*******************************************************************************
 Function         : CheckIgnition
 Description      : Checks if need to kill/restore ignition
 Parameters       : 
 Return           : none
 Globals Accessed : No
 *******************************************************************************/
void PimManager::CheckIgnition(void)
{
	// kill ignition period?
	if(pimData.kill_ignition_duration)
	{
		if (pimData.kill_ignition_duration-- == 0)
		{
			// period finished, disable KILL_IGNITION 
			pimData.pim_ports |= PORT_IGNITION_DRV_BITMASK;
			pimData.pim_ports |= PORT_RELAY1_DRV_BITMASK;
		}
	}

	// restore ignition period?
	if(pimData.restore_ignition_duration)
	{
		if (pimData.restore_ignition_duration-- == 0)
		{
			if (pimSettings.lock_immunity == false)
			{
				// period finished, enable KILL_IGNITION 
				pimData.pim_ports &= ~PORT_IGNITION_DRV_BITMASK;
				pimData.pim_ports &= ~PORT_RELAY1_DRV_BITMASK;
			}
		}
	}
}

/*******************************************************************************
 Function         : BuzzerPortSwitch
 Description      : Called by interrupt handler TIM2
                    Switch the output for buzzer according to the settings
 Parameters       : none
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::BuzzerPortSwitch(void)
{  
	if (powerManager->GetPowerStatus().connection_type  == CT_PORTABLE)
	{
		if (pimData.charge_indicator)
		{
			buzzer->SetBuzzerOn();
		}
		else
		{
			buzzer->SetBuzzerOff();
		}
	}
	else
	{
		// pulses to do?
		if(pimData.buzzer_pulses_count)           
		{
			// one period elapsed?
			if(pimData.buzzer_period_count-- == 0)  
			{
				// reload the period counter
				pimData.buzzer_period_count = pimData.buzzer_pulsew_high + 
				pimData.buzzer_pulsew_low;
				// decrement pulse counter
				pimData.buzzer_pulses_count--;        
				// output reset to '0'
				buzzer->SetBuzzerOff();
			}
			// otherwise, on-time elapsed?
			else if(pimData.buzzer_period_count == pimData.buzzer_pulsew_high)
			{
				if (!pimSettings.buzzer_immunity)
				{
					// output set to '1'
					buzzer->SetBuzzerOn();
				}
			}
		}
		else
		{
			buzzer->SetBuzzerOff();
		}
	}
}
/*******************************************************************************
 Function         : GetPimState
 Description      : Get the status of the PIM ports and TAG supply . 
                    It is called from other tasks
 Parameters       : none
 Return           : bitfield
                    MASK_PIM_PORTS                    0x00FF
                    MASK_VBAT                         0x0100
                    MASK_VIN                          0x0200
                    
 *******************************************************************************/
uint16_t PimManager::GetPimState(void)
{
	uint16_t return_data = 0;
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{		
		// get the state of the ports
		return_data = pimData.pim_ports;
		// get the state of the supply voltages
//		if(!powerManager->GetVin().connected)
		if(!powerManager->GetVin())
		  return_data |= MASK_VIN_OFF;
//		if (powerManager->GetVbat().average > VBAT_MIN_VOLTAGE)
		if(powerManager->GetVbat() > VBAT_MIN_VOLTAGE)
			return_data |= MASK_VBAT_OK;
		if(powerManager->IsVbatCharging())
			return_data |= MASK_CHARGING;
	
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}
	
	return return_data;
}

/*******************************************************************************
 Function         : SetBuzzer
 Description      : Set the buzer parameters
 Parameters       : parameter to get set: 

 Return           : none 
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::SetBuzzer(uint16_t pulsewHigh, uint16_t pulsewLow, uint16_t pulses, uint16_t pulsesRepeat, uint16_t repeatDelay)
{
	int panic = isPanicReset();

	if (panic || osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{	
		// set parameters
		pimData.buzzer_pulsew_high = pulsewHigh;
		pimData.buzzer_pulsew_low = pulsewLow;
		pimData.buzzer_pulses = pulses;
		pimData.buzzer_repeat = pulsesRepeat + 1;      // because decrement first
		pimData.buzzer_repeat_delay = repeatDelay;
	}
	
	if (!panic)
		osSemaphoreRelease(semaphore);
}

/*******************************************************************************
 Function         : SetIgnitionOff
 Description      : Set the kill ignition parameter
 Parameters       : parameter to get set
          
 Return           : none 
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::SetIgnitionOff(uint16_t kill_duration)
{
	bool lock_immunity;
	bool gotSemaphore = false;
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		gotSemaphore = true;
		// set parameter
		pimData.kill_ignition_duration = kill_duration;
		lock_immunity = pimSettings.lock_immunity;
		osSemaphoreRelease(semaphore);
	}
	
	// kill the ignition by closing the relay
	if(gotSemaphore && lock_immunity == false)
	{
		SetRelay(ON, 1);			
	}
			
}

/*******************************************************************************
 Function         : SetIgnitionOn
 Description      : Set the restore ignition parameter
 Parameters       : parameter to get set

 Return           : none 
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::SetIgnitionOn(uint16_t restore_duration)
{	
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		// set parameter
		pimData.restore_ignition_duration = restore_duration;
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}
		// restore the ignition by opening the relay
		SetRelay(OFF, 1);
}

/*******************************************************************************
 Function         : SetRelay
 Description      : turn on/off relay
 Parameters       : state - ON/OFF , relay - relay num 0,1
          
 Return           : none 
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::SetRelay(GPIO_Pin_Status state, uint8_t relay)
{
	uint8_t bitmask;
	
	if (relay == 0)
		bitmask = PORT_RELAY1_DRV_BITMASK;
	else
		bitmask = PORT_RELAY2_DRV_BITMASK;
	
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		if (state == ON)
			pimData.pim_ports &= ~bitmask;  
		else
			pimData.pim_ports |= bitmask;
		
		if (relay == 1)
		{
			overrideDelay = TimeHelper::GetCurrentTime() + 1000;    
			ds2408->PioDs2408(1, pimData.pim_ports, &pioState);
		}
					
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}
}

/*******************************************************************************
 Function         : IsRelayOn
 Description      : Checks if relay On
 Parameters       : relay - relay num 0,1
          
 Return           : none 
 Globals Accessed : yes
 *******************************************************************************/
bool PimManager::IsRelayOn(uint8_t relay)			
{
	bool ret;
	
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		if (relay == 0)
			ret = (pimData.pim_ports & PORT_RELAY1_DRV_BITMASK) == PORT_RELAY1_DRV_BITMASK;
		else
			ret = (pimData.pim_ports & PORT_RELAY2_DRV_BITMASK) == PORT_RELAY2_DRV_BITMASK;
		
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}
	return !ret;
}
			
/*******************************************************************************
 Function         : PowerNap
 Description      : turn off leds and relay
 Parameters       : 
          
 Return           : none 
 Globals Accessed : yes
 *******************************************************************************/
void PimManager::PowerNap(void)
{
	// wait to get access to the PIM variable
	//if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	//{
		ds2408->PioDs2408(1, 0xff, NULL); 
		// release the access to the PIM variable
	//	osSemaphoreRelease(semaphore);
	//}
}

/*******************************************************************************
 Function         : SetLedPattern
 Description      : Set patterns for LED
 Parameters       : red, green - patterns
          
 Return           : none 
 Globals Accessed : yes
 *******************************************************************************/
void PimManager::SetLedPattern(uint32_t red, uint32_t green)
{
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		ledBitmapRed = red;
		ledBitmapGreen = green;
  
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}
}

/*******************************************************************************
 Function         : EnableLedStatus
 Description      : Enables LED status
 Parameters       : none
          
 Return           : none 
 Globals Accessed : yes
 *******************************************************************************/
void PimManager::EnableLedStatus(void)
{
	disableLeds = false;
}

/*******************************************************************************
 Function         : DisableLedStatus
 Description      : Disables LED status
 Parameters       : none
          
 Return           : none 
 Globals Accessed : yes
 *******************************************************************************/
void PimManager::DisableLedStatus(void)
{
	disableLeds = true;
}
/*******************************************************************************
 Function         : PimDetection
 Description      : check if PIM is connected based on 1Wire device
 Parameters       : none

 Return           : 1: PIM connected, 0: PIM not connected 
 Globals Accessed : yes
 *******************************************************************************/
uint16_t PimManager::PimDetection(uint8_t* sn)
{
	uint8_t temp[8];
	if (sn == NULL)
	{
		sn = temp;
	}
	
	uint16_t returnVal = 0;
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		ds2408->FixDs2408();
		
		//pim_detect = PIM_DETECTING
		if(ds2408->ReadDs2408Sn(sn) == 0)
		  returnVal = 1;              // 1Wire found
		
	// release the access to the PIM variable
	osSemaphoreRelease(semaphore);
	}
	return returnVal;
}
/*******************************************************************************
 Function         : RestorePimSettings
 Description      : read pim settings from flash
 Parameters       : isupdate - false if this is the first call; true if we're updating
 Return           : none 
 *******************************************************************************/
void PimManager::RestorePimSettings(bool isUpdate)
{
	powerManager->RestorePowerSettings(isUpdate);
	
	pimSettings.lock_immunity = settingsController->GetSettingInt(270, 0, 1, 0);
	pimSettings.buzzer_immunity = settingsController->GetSettingInt(271, 0, 1, 0);
	pimSettings.pim_should_blink = settingsController->GetSettingInt(280, 0, 1, 1);
  
	// wait for other tasks to get settings too
	if(!isUpdate)
	  osDelay(500);
}

/*******************************************************************************
 Function         : PimTask
 Description      : Task for managing the PIM
                    Switch ignition and buzzer according to settings 
 Parameters       : restoreSettings -1 do not restore, 
					1 -restore with wait, 0 restore without wait
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::PimTask(int8_t restoreSettings)
{
	static uint8_t cycles_count = 0; 
	static uint8_t pim_ports = 0;
	static unsigned int fix_counter = 0;

	if (restoreSettings == 0)
		RestorePimSettings(false);                    // load the settings of the PIM task
	else if(restoreSettings == 1)
		RestorePimSettings(true);
	
	
	if ((++fix_counter % 40) == 0)
		ds2408->FixDs2408();
        
	CheckEmergency();		
	//DisableLedStatus();
	CheckLedStatus();
	CheckLeds(cycles_count);

	// buzzer, ignition and supply status are updated each second
	if((cycles_count % 4) == 0)
	{
		// wait to get access to the PIM variable
		if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
		{
			CheckBuzzer();
			CheckIgnition();
			//externalMessageSender.DiagPrintf(DIAG_POWER, 1, "Testing");	
			powerManager->CheckPower();
		
			if (powerManager->IsVbatCharging() == true)
			{
				pimData.charge_indicator = !pimData.charge_indicator;
			}
			else
			{
				pimData.charge_indicator = true;
			}

			pim_ports = pimData.pim_ports;             // local copy for releasing access quickly

			// write the PIM with the proper data
			if(overrideDelay <  TimeHelper::GetCurrentTime())
			{
				ds2408->PioDs2408(1, powerManager->IsGoingToNap() ? 0xff : pim_ports & (~pimLeds), &pioState);
			}
			
			BuzzerPortSwitch();
			
			// release the access to the PIM variable
			osSemaphoreRelease(semaphore);
		}
	}
    
	// update the counter for flashing sequences
	if(++cycles_count == SEQUENCES_NUMBER)
	  cycles_count = 0;
		
}

/*******************************************************************************
 Function         : AutoPimTestTask
 Description      : Task for testing the PIM
                     
 Parameters       : 
 Return           : none
 Globals Accessed : Yes
 *******************************************************************************/
void PimManager::AutoPimTestTask(void)
{
	static bool first = true;
	static uint8_t pio_state, cycles_count = 0;  
	static uint8_t pim_ports;  // local copy of the pim ports
	
	if(first)
	{
		first = false;
		// set output and switch off the buzzer
		buzzer->SetBuzzerOff();
    
		// Initialise the ports of the PimManager: all off
		pimData.pim_ports = 0xFF;         // all off
		ds2408->PioDs2408(1, pimData.pim_ports, &pio_state);        // the actual state of the ports is inverted
	}
	
	ds2408->FixDs2408();

	if ((cycles_count % 2) == 0)
	{
		SetBuzzer(20, 25, 10, 1, 1);

		if (pimData.buzzer_repeat)
		{
			// buzzer pulses finished?
			if(pimData.buzzer_pulses_count == 0)
			{
				// delay finished?
				if(pimData.buzzer_repeat_delay_count == 0)
				{
					// decrement the repetition counter 
					// reload the pulses counter and delay counter if there are still cycles to do 
					if(--pimData.buzzer_repeat)
					{
						pimData.buzzer_pulses_count = pimData.buzzer_pulses;
						pimData.buzzer_repeat_delay_count = pimData.buzzer_repeat_delay;
					}
				}
				else
				{
					pimData.buzzer_repeat_delay_count--;
				}
			}
		}
	}
	
	SetRelay(OFF, 0);
	SetRelay(OFF, 1);
	            
	switch (cycles_count % 4)
	{
	case 0: 
		SetRelay(ON, 0);     
		break;
	case 1:
		SetRelay(ON, 1);
		break;      
	}
        
	// wait to get access to the PIM variable
	if(osSemaphoreWait(semaphore, osWaitForever) == osOK)
	{
		pim_ports = pimData.pim_ports;           // local copy for releasing access quickly
      
		// release the access to the PIM variable
		osSemaphoreRelease(semaphore);
	}

	ds2408->PioDs2408(1, pim_ports & (~pimLeds), &pio_state); 

	cycles_count++;
}

/*******************************************************************************
 Function     : SetShippingMode
 Description  : Sets device to shipping mode
 Parameters   : None
 Return       : None
*******************************************************************************/
void PimManager::SetShippingMode(void)
{
	SetBuzzer(10, 15, 3, 2, 1);
	osDelay(2000);
	SetBuzzer(0, 0, 0, 0, 0);
	osDelay(2000);

	powerManager->PowerNap(false, 10000, true, 0);
}

PimManager::~PimManager()
{	
	osSemaphoreDelete(semaphore);
}
