/**
 * Nimh_Charger.c
 *
 * Atmel Studio 6.2 Project
 *
 *
 * Created: 2014.08.17. 15:05:06
 * Author: Kazintsi Zoltan (kazinci~at~gmail~dot~com)
 *
 * AVR (Atmega8) based NiMh charger
 *
 * Version 1.0
 *
 * The entire project, including schematic can be found at:
 *  ... 
 *
 *
 * LCD, UART and I2C library copyright  Peter Fleury 
 * ( pfleury@gmx.ch   http://homepage.hispeed.ch/peterfleury/) 
 *
 *
 *
 *
 *
 *
 * Theory of operation:
 * The charger charges batteries sequentially with C/2 current (fast charge),
 * which means if a 2500mAh battery inserted its charged with 1,25 Ampere (2500/2 = 1250).
 * After the 1st battery is charged then switches to the 2nd one and charges with C/2 current also.
 * If both are charged the charger switches to "Trickle" mode to maintain batteries.
 * Trickle charging 6 hours by default with C/40 current (0,062 Ampere for 2500mAh battery)
 */ 





/************************************************************************/
/*                              INCLUDES                               */
/************************************************************************/
#include <avr/io.h>
#include "NiMh_Charger.h"
#include <stdlib.h> // abs(), itoa()
#include <stdarg.h> // variable number argument function
#include <avr/pgmspace.h> // PROGMEM
#include <avr/eeprom.h> // EEMEM
#include <avr/interrupt.h> // Timer (seconds, minutes)
#include <util/delay.h>
#include <avr/sleep.h> // sleep when Charger stops
#include <avr/wdt.h> // watchdog
#include <util/atomic.h> // interrupt-secure routines
#include "lcd.h"
#include "uart.h" // for debug
#include "i2cmaster.h" // used for temperature reading


/************************************************************************/
/*                        GLOBAL VARIABLES                             */
/************************************************************************/

//Note:
//uint8_t max value is 255
//uint16_t max value is 65535

char		buffer[8]; //UART buffer
// NDV and FDV
uint8_t		neg_db = NDV_SAMPLES; // ndv counter (negative delta voltage)
uint16_t	fdv_db = FDV_SAMPLES; // fdv counter (flat delta V)
uint8_t		peak_db = PEAK_UPDATE_SAMPLES; // If voltage is higher "peak_db"-times = > update "peak_voltage" value
uint16_t	peak_voltage = 0; // if voltage is rising, new voltage value saved here
// ADC oversampling and approximation
uint16_t	voltage_in_ADC = 0; 
uint16_t	current_in_ADC = 0; 
uint8_t		ramp = RAMP_UP_TIME; //timeout to set charging current
// these variables are changed by interrupt-routine, so they are must be declared as volatile
uint8_t volatile	seconds = 0; // 0...59
uint16_t volatile	minutes = 0; // 0...59
uint8_t volatile	timer1_tick = 0; // 0...4
// Hardware components check
uint8_t		isTempOk_1 = 0; // I2C device_1 check flag, O: NOK, 1: OK
uint8_t		isTempOk_2 = 0; // I2C device_2 check flag, O: NOK, 1: OK
//uint8_t		lcd_ok = 0; // 
uint8_t		isDischarged = 0; // to hold discharged status flag (boolean)
// Debug information about charging times
//uint16_t		time_in_precharge_mode = 0;
//uint16_t		time_in_fastcharge_mode = 0;
uint16_t		trickle_end_timer = 0; // Timer for trickle charge, charging halts when this timer ends.
// absorbed mAh's by battery, summed after every mode
uint16_t	mAh = 0;
// MENU items
uint8_t menuItem = 0; 
uint8_t	battery_number = 1; // selected battery
uint8_t		mode = 0; //charger modes, see defines above



uint8_t		mcucsr __attribute__((section(".noinit"))); // Mirror of the MCUCSR register, taken early during startup.
uint8_t		saved_batt_num __attribute__((section(".noinit"))); // batt_num variable back-up if watchdog reset occurs
uint8_t		saved_mode __attribute__((section(".noinit"))); // mode variable back-up if watchdog reset occurs
uint16_t	saved_minute __attribute__((section(".noinit"))); // minute variable back-up if watchdog reset occurs


// Stored variables in EEPROM
uint16_t EEMEM eep_batt_capacity = 2500; // Battery Capacity is 2500 mAh for default, user changeable through "Menu"
uint16_t EEMEM eep_charge_mode = FASTCHARGE_MODE;
uint16_t EEMEM eep_trickle_charge_time = 16*60; // 16 hour by default
uint16_t EEMEM eep_discharge = 0; // Do not discharge by default

// Buttons								left,	select,	down,	right,	up 
const uint16_t buttondata[5] PROGMEM ={	176,	87,		650,	960,	0};
//String constants:
const char string0[] PROGMEM = "DISCHARGE";
const char string1[] PROGMEM = "BATTERY CHARGER";
const char string2[] PROGMEM = "BATT. DETECTION";
const char string3[] PROGMEM = "NO BATTERY";
const char string4[] PROGMEM = "BATT. DETECTED";
const char string5[] PROGMEM = "PRECHARGE";
const char string6[] PROGMEM = "FASTCHARGE";
//const char string7[] PROGMEM = "TOPOFF";
const char string8[] PROGMEM = "TRICKLE";
const char string9[] PROGMEM = "END";
const char string10[] PROGMEM = "PRECHARGE TIME";
const char string11[] PROGMEM = "FASTCHARGE TIME";
const char string12[] PROGMEM = "TOPCHARGE TIME";
const char string13[] PROGMEM = "TRICKLE TIME";
const char string14[] PROGMEM = "TOTAL TIME";
const char string15[] PROGMEM = "mAh:";
const char string16[] PROGMEM = "SAVED";
const char string17[] PROGMEM = "SAVE?";
const char string18[] PROGMEM = "OK-SAVE,UP/DN-NO";
const char string19[] PROGMEM = "TIMER END";
//const char string19[] PROGMEM = "MAINTENANCE_END_TIMER";
//const char string20[] PROGMEM = "PRECHARGE_END_TIMER";
//const char string21[] PROGMEM = "FASTCHARGE_END_TIMER";
//const char string22[] PROGMEM = "TOPOFFCHARGE_END_TIMER";
const char string23[] PROGMEM = "BATT_CAP_NOK";
const char string24[] PROGMEM = "MODE_NOK";
const char string25[] PROGMEM = "TIME_NOK";
const char string26[] PROGMEM = "WDT";
const char string27[] PROGMEM = "YES";
const char string28[] PROGMEM = "NO";
const char string29[] PROGMEM = "MAX VOLTAGE";
 
 
 
 
 
 
 
volatile struct Time
{
	// these variables are changed by interrupt-routine, so they are must be declared as volatile
	uint8_t volatile	seconds; // 0...59
	uint16_t volatile	minute; // 0...59
	uint8_t volatile	timer1_tick; // 0...4
} time;
 
 // TODO: use struct
struct Battery
{ 
	uint16_t volt:12; //0...2560 (2^12=4096)
	uint16_t amp:12; //0...2560
	uint16_t temp:7; //0...127 (2^7 =128)
	uint8_t tempSensorOk:1; //0...1 
	struct Time t;
	// TODO: add mah, time
};




/************************************************************************/
/*                         MAIN FUNCTION START                          */
/************************************************************************/
int main (void) {	
	// LOCAL VARIABLES	
	
	// Charging currents at different phases, calculated from battery capacity stored in EEPROM
	uint16_t precharge_max_current = 0;
	uint16_t fastcharge_max_current = 0;
	uint16_t trickle_max_current = 0;
	 
	
	/* Initializations */
	// Disable MOSFET transistors
	Mosfet_switch(BATTERY_1,OFF);
	Mosfet_switch(BATTERY_2,OFF);
	Mosfet_switch(DISCHARGE,OFF);
	// For safety reasons disable PWM
	Disable_PWM();
	// Initialize UART Interface, used for debugging
	UART_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	// Initialize ADC Interface, used for Buttons, measuring Battery Voltage and Current
	ADC_Init();
	// Initialize LCD display without cursor
	lcd_init(LCD_DISP_ON);
	// Initialize I2C interface for Temperature measuring devices.
	i2c_init();
	// Initialize Timer1 for counting seconds & minutes
	Init_timer();	
	// Enable global interrupts (UART and Timer1 is interrupt driven)
	sei(); 	
	// check if temperature measuring devices are connected
	isTempOk_1 = init_TMP75(1);
	isTempOk_2 = init_TMP75(2);
	
	// If watchdog activated continue with that point from where the reset occurred 
	if ((mcucsr & _BV(WDRF)) == _BV(WDRF))
	{
		// watchdog reset occured:
		//
		uart_send_FLASH_string(string26);
		// Continue with last battery
		battery_number = saved_batt_num;
		// Continue with last mode
		mode = saved_mode;
		time.minute = saved_minute;
	}
	else
	{
		// Normal start-up procedure
		// default mode is "Menu Selection"
		mode = MENU_MODE; 
		// Set battery_number to "0", later it's auto incremented in Battery_Detection routine
		battery_number = 0;
		time.minute = 0;
	}
	
	// "Menu Selection" - to allow users check and change charging parameters.
	if (mode == MENU_MODE)
	{
		ADC_Read_Avg(BUTTON_ADC_CHANNEL,1); // Set ADC Channel for buttons = 2nd channel, 1 sample
		while (mode == MENU_MODE)
		{
			// mode changes if the user pushes "OK" button
			mode = Menu();
			// next mode is INIT_MODE to check whether batteries are inserted or not
		}		
	}
	
	// After "MENU_MODE" read out charging parameters from EEPROM		
	// create new scope {} to destroy "ram_batt_capacity" variable for RAM saving purpose
	// after exiting from this scope "ram_batt_capacity" variable not accessible anymore
	// "ram_batt_capacity" only needed for calculating charging Current in different phases.
	{
		// Battery Capacity
		uint16_t ram_batt_capacity = eeprom_read_word(&eep_batt_capacity);
		if (ram_batt_capacity == 0xFFFF) // 0xFFFF means EEPROM clear
		{
			Terminate(string23);
		}
		
		// Charge mode
		uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
		if (ram_charge_mode == 0xFFFF)
		{
			// If there is no appropriate value found in EEPROM
			Terminate(string24);
		}
		
		// Trickle Charging Time
		uint16_t ram_trickle_charge_time = eeprom_read_word(&eep_trickle_charge_time);
		if (ram_trickle_charge_time == 0xFFFF)
		{
			// If there is no appropriate value found in EEPROM
			Terminate(string25);
		}
		
		// Calculate maximum charging Currents  from battery capacity  for different modes 		
		precharge_max_current = ram_batt_capacity/10; // when the battery deeply depleted trickle charge with C/10 until not achieve 1 Volt
		fastcharge_max_current = ram_batt_capacity/2 ; // Fast charge with C/2
		
		if (ram_charge_mode == FASTCHARGE_MODE)
		{
			// If Fast Charge selected from menu  "Trickle" mode only used for maintenance
			// Therefore charging with C/40 is more than enough
			trickle_max_current = ram_batt_capacity / 40;
			trickle_end_timer = 6*60; // 6 hour in maintenance
		}
		// TODO: check if need double current (batteries in parallel)
		else if (ram_charge_mode == TRICKLE_MODE)
		{
			// If Trickle Charge selected from menu charge with C/10
			trickle_max_current = ram_batt_capacity / 10;
			// Terminate charging after the given time
			trickle_end_timer = eeprom_read_word(&eep_trickle_charge_time);
		}
	}
	
	
		
	/************************************************************************/
	/*                                MAIN LOOP                             */
	/************************************************************************/
	while (1) {		
		
		wdt_enable(WDTO_2S);
		// reset these variables for every battery
		mAh = 0;
		neg_db = NDV_SAMPLES;
		fdv_db = FDV_SAMPLES;
		peak_db = PEAK_UPDATE_SAMPLES;
		ramp = RAMP_UP_TIME;
		peak_voltage = 0;
		voltage_in_ADC = 0;
		current_in_ADC = 0;
		
		// indicate: the program stepped into the main loop
		lcd_clrscr();
		uart_send_FLASH_string(string1);
		LCD_Send_row(0, 1, string1); // with delay
		
		
		
		// Battery detection
		while(mode == INIT_MODE)
		{
			mode = Battery_Detection();		
		}		
		
		// if discharge is necessary
		while(mode == DISCHARGE_MODE){
			mode = Discharge_Battery();		
		}
		
		// If battery found and voltage < 1000mV then PRECHARGE
		while(mode == PRECHARGE_MODE){			
			mode = Charge_Battery(battery_number, precharge_max_current, string5);
		}
		
		// If battery voltage > 1000mV then FASTCHARGE
		while(mode == FASTCHARGE_MODE){
			mode = Charge_Battery(battery_number, fastcharge_max_current, string6);
		}
		
		// After both batteries are successfully charged in FASTCHARGE_MODE switch to TRICKLE_MODE
		while(mode == TRICKLE_MODE){	
			mode = Charge_Battery(battery_number, trickle_max_current, string8);
		}
		
		//Send_charging_info(time_in_precharge_mode,time_in_fastcharge_mode,time_in_topoffcharge_mode,time_in_trickle_mode,mAh);
				
	}
}





uint8_t Menu(void)
{
	uint8_t itemSelected = 1; // flag, indicates "jump out" from actual menu item
	uint8_t save_flag = 0; // flag, prevents asking again for saving data from UP/DOWN menus if it is canceled
	uint8_t	selected_button = 0; //indicate which button is selected
	uint16_t ram_value = 0; // data being increased/decreased
	uint16_t low_limit = 0; // when the user decreasing the actual number it will be the lowest allowed value
	uint16_t high_limit = 0; // highest allowed value
	uint16_t eep_value = 0; // old data, which read out from EEPROM
	
	lcd_clrscr(); // Clear display
	switch (menuItem)
	{
		case 0:
			// message on LCD
			// TODO: clearing one row before sending new string not needed
			LCD_Send_row(0, 0, PSTR ("OK - CHARGE"));
			LCD_Send_row(1, 0, PSTR ("<> - SETUP"));
			break;
				
		case 1:
			LCD_Send_row(0, 0, PSTR ("BATT CAPACITY"));
				
			// Read value from EEPROM
			uint16_t ram_batt_capacity = eeprom_read_word(&eep_batt_capacity );
				
			// Show it on the display (2nd row)
			lcd_send_integer(1,&ram_batt_capacity);
			// TODO: Move to define
			low_limit = 100; // mA
			high_limit = 3000; // mA
			ram_value = ram_batt_capacity;
			eep_value = eep_batt_capacity;
								
			break;
				
		case 2:
			LCD_Send_row(0, 0, PSTR ("CHARGING MODE"));
				
			// Read value from EEPROM
			uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
				
			// Show it on the display (2nd row)
			if (ram_charge_mode == FASTCHARGE_MODE)
			{
				LCD_Send_row(1, 0, string6);
			}
			// TODO: check whether "if" can be removed
			else if (ram_charge_mode == TRICKLE_MODE)
			{
				LCD_Send_row(1, 0, string8);
			}
			low_limit = FASTCHARGE_MODE;
			high_limit = TRICKLE_MODE;
			ram_value = ram_charge_mode;
			eep_value = eep_charge_mode;
				
			break;
				
		case 3:
			LCD_Send_row(0, 0, string13);
				
			// Read value from EEPROM
			uint16_t ram_trickle_charge_time = eeprom_read_word(&eep_trickle_charge_time );
				
			// Show it on the display (2nd row)
			lcd_send_integer(1,&ram_trickle_charge_time);
			// TODO: move to define				
			low_limit = 0;
			high_limit = 1000;
			ram_value = ram_trickle_charge_time;
			eep_value = eep_trickle_charge_time;
				
			break;
				
		case 4: // Enable Discharge?
			LCD_Send_row(0, 0, string0);
				
			// Read value from EEPROM
			uint16_t ram_discharge = eeprom_read_word(&eep_discharge );
				
			// Show it on the display (2nd row)
				
			ram_discharge ?	LCD_Send_row(1, 0, string27) 	:	LCD_Send_row(1, 0, string28);
			// boolean (on-off)
			low_limit = 0; 
			high_limit = 1;
			ram_value = ram_discharge;
			eep_value = eep_discharge;
				
			break;
				
	}
	
	// Loop until menu item doesn't changed
	while (itemSelected) // flag changed if the user selects another item
	{
		selected_button = Read_analog_buttons();
			
		switch (selected_button)
		{
			case LEFT:
				if (menuItem)
				{
					// Decrease actual value until "low_limit"
					Button_Left(low_limit, &ram_value);
					
				}
				break;
					
			case RIGHT:
				if (menuItem)
				{
					// Increase actual value until "high_limit"
					Button_Right(high_limit,&ram_value);
					
				}
				break;
				
			case UP:
				if (menuItem)
				{
					// select previous menu item
					Button_Up_Down( eeprom_read_word(&eep_value ), ram_value, &save_flag, &itemSelected, &menuItem, --menuItem);
				}
				
				break;
					
			case DOWN:	
				if (menuItem)
				{
					// select next menu item
					Button_Up_Down( eeprom_read_word(&eep_value ), ram_value, &save_flag, &itemSelected, &menuItem, ++menuItem);
				}
				else
				{
					
						menuItem++;
						itemSelected = 0; // jump out from this menu item
					
				}
				break;
				
			case SELECT:
				if (!menuItem) // 1st menu item
				{
					//Start charging
					return INIT_MODE;
					// TODO: check if above removable
					//menuItem = max_menuItem+1; // jump out from the MENU
				} 
				else
				{
					// Save new value to EEPROM
					eeprom_write_word(&eep_value, ram_value);
					lcd_clrscr();
						
					// Show "SAVED" with delay
					LCD_Send_row(1, 1, string16);
						
					// Jump out from switch statement
					// to show the new, saved value
					itemSelected = 0;
				}
					
				break;
			default:
				break;
		}
	}
	
	return MENU_MODE;
}



void Button_Up_Down(uint16_t eep_value, uint16_t ram_value, uint8_t *save_flag, uint8_t *isMenuSelected, uint8_t *menuItem, uint8_t next_menu)
{
	if ( (eep_value != ram_value) && *save_flag == 0)
	{
		lcd_clrscr();
		// SAVE?
		LCD_Send_row(0, 0, string17);
		LCD_Send_row(1, 0, string18);
		*save_flag = 1; // prevent asking again for saving, if it's cancelled
	}
	else // if saving cancelled, select next item
	{
		*isMenuSelected = 0; // jump out from current item
		if (*menuItem + 1 <= MENU_MAX) // if we doesn't reached the end of the menu
			*menuItem = next_menu; // next item
	}
}

void Button_Left(uint16_t min, uint16_t *value)
{
	//Decrease if possible
	if (((int)*value - 1) >= (int)min)
	{
		(*value)--;
	}
		
	// Show new value
	lcd_send_integer(1,value);
}

void Button_Right(uint16_t max, uint16_t *value)
{
	// Increase if possible
	if (*value + 1 <= max)
	{
		(*value)++;
	}
	// Show new value
	lcd_send_integer(1,value);
}

void lcd_clean_row (uint8_t row_number)
{
	lcd_gotoxy(0,row_number);
	for (uint8_t i = 0; i < 16; i++)
	{
		lcd_putc(' ');
	}
}

/**
   \brief Send an integer value to the selected row
   row_number 0 == 1st row
   row_number 1 == 2nd row
*/
void lcd_send_integer(uint8_t row_number, uint16_t *integer)
{
	// Clear only the selected row
	lcd_clean_row(row_number);
	// Show value
	itoa( *integer,buffer,10 );
	lcd_gotoxy(0,1);
	lcd_puts(buffer);
}


uint8_t Charge_Battery(uint8_t battery_number, uint16_t max_current, const char *string_for_display)
{
	uint8_t	next_mode = mode;
	uint8_t	previous_sec =	0;
	uint8_t	previous_tick =	0;
	uint16_t time_left_in_this_mode = 0;
	uint8_t pwm = 0;
	uint16_t temperature = Read_temperature(battery_number);
	uint8_t display_flag = 1; // for skipping first 10 seconds 
	uint8_t uart_flag = 0; // uart logging flag, set in every second
	uint16_t voltage = 0;
	uint16_t current = 0;
	ramp = RAMP_UP_TIME; //PWM ramp up timeout to set the desired current
	voltage_in_ADC = 0; // approximation global variable for voltage measuring
	current_in_ADC = 0;
	neg_db = NDV_SAMPLES; // timeout  for NDV
	fdv_db = FDV_SAMPLES; // timeout for FDV
		
	
	if(mode == TRICKLE_MODE)
	{
		// Enable both batteries in Trickle mode
		Mosfet_switch(BATTERY_1, ON); // if not enabled yet
		Mosfet_switch(BATTERY_2, ON); // if not enabled yet
	}
	else
	{
		// Enable only the selected battery
		Mosfet_switch(battery_number, ON);
	}
		// reset PWM
	Disable_PWM();
	Init_PWM(0);
	uart_send_FLASH_string(string_for_display);
	lcd_clrscr();
	lcd_puts_p(string_for_display);
	
	//WDT_Init(); // Enable watchdog
	while(next_mode == mode){
		
		if ( previous_tick != time.timer1_tick){ // Every 200msec, "timer1_tick" changed by interrupt routine
			// measuring voltage and current must happen as frequently as possible
			// because they are calculated from oversampling and approximation.
			// 14.85 mseconds from here ...
			voltage = Measure_U();
			current = Measure_I();
			// ... to there
			
			// uart_flag set after every seconds
			if (uart_flag)
			{
				// Send debug info through UART
				// Because it's generates too many interrupts, activate only when we have more than 50ms idle time
				// 42ms for send out 11 variables through UART
				// We have 200 - 14.85 = 185.15 mseconds idle time after measuring voltage and current
				UART_send_data_block(13, mode, time.minute, time.seconds, voltage, current, temperature, pwm, peak_db, peak_voltage, fdv_db, neg_db, time_left_in_this_mode, battery_number); // Debug
				uart_flag = 0; // reset flag, enabled after 1 seconds
			}
			// TODO: LCD in idle time
			if (previous_sec != time.seconds){// Every secs, "seconds" changed by interrupt routine
				// a check period is ~ 7 useconds 
				
				// State machine
				next_mode = Check_battery_status(battery_number, voltage, current, &temperature, &time_left_in_this_mode); //measure voltage and decide what to do next.
				
				// check if mode was changed by the state-machine
				if (mode != next_mode)
				{
					// Calculate absorbed mAh's in this mode
					mAh += ((uint32_t)current * time.minute ) / 60;
					
					// If only 1 battery charged yet, skip trickle mode,
					// because this mode takes long time
					// switch to the next battery and apply "trickle mode" for both batteries at the end of charging
					if (mode == FASTCHARGE_MODE)
					{
						// Check whether next battery inserted or not, jump to INIT_MODE
						next_mode = INIT_MODE;
						//Send_charging_info(time_in_precharge_mode,time_in_fastcharge_mode,time_in_topoffcharge_mode,time_in_trickle_mode,mAh);
					}
					
				}
				
				if (!(time.seconds % 10) || display_flag) //every 10sec
				{
					display_flag = 0; // it's only for skipping first 10 seconds
					// increase/decrease PWM by 1
					// active only for 10 minutes (ramp-up)
					Modify_PWM_Width(current, max_current, &pwm);
					// Send debug info to LCD
					LCD_Send_Debug(voltage, current, temperature);
				}
				
				// saving total time in this mode
				//if (seconds == 0){ // Every minutes
					//(*time_in_this_mode)++;
				//}
				// Save "seconds" variable for comparing
				previous_sec = time.seconds;
				
				uart_flag = 1; // UART logging in next period
			} // 1 second left...
			
			// Save "timer1_tick" variable for comparing
			previous_tick = time.timer1_tick;
			// reset watchdog
			wdt_reset();			
		} // 200ms left...
	
	} // mode changed, exiting...
	wdt_reset();
	return next_mode;
}


uint8_t Discharge_Battery(void)
{
	uint16_t	voltages[2]; // Battery voltages
	uint8_t		previous_sec = 0;
	
	// Initially measure voltages on both batteries
	//Mosfet_switch(1,0);
	Mosfet_switch(BATTERY_2, OFF); // Disable 2nd
	// 1st
	Mosfet_switch(BATTERY_1, ON);
	voltages[0] = Measure_U();
	Mosfet_switch(BATTERY_1,OFF);
	// 2nd
	Mosfet_switch(BATTERY_2, ON);
	voltages[1] = Measure_U();
	Mosfet_switch(BATTERY_1, ON);
	// Now both batteries are turned on
	
	Mosfet_switch(DISCHARGE,ON);	// Discharge ON
	
	uart_send_FLASH_string(string0);
	lcd_clrscr();
	LCD_Send_row(0, 0, string0);
	
	while(1)
	{
		if (previous_sec != time.seconds && !(time.seconds % 10))// Every second check voltage
		{
			// 1. Check 1st battery
			// prevent discharging below 1000mV
			// also check if battery still present in the slot
			if (voltages[0] < 1000 || voltages[0] > FAST_CHARGE_END_MAX_VOLTAGE) 
			{
				// Stop Discharging this battery
				Mosfet_switch(BATTERY_1,OFF); // disable the 1st battery
			}
			else
			{
				 uint8_t enabled = 0; // boolean flag to check whether the other battery need to be disable and re-enable or already disabled (discharged)
				// disable the 2nd battery if needed to measure voltage only on the 1st
				if (bit_is_set(PINB,PB2)) // check if it's enabled
				{
					Mosfet_switch(BATTERY_2,OFF); // Disable
					enabled = 1; // set flag
				}
				_delay_ms(100); // wait to stabilize voltage
				voltage_in_ADC = 0; // Reset oversampling
				voltages[0] = Measure_U(); // measure
				if (enabled) Mosfet_switch(BATTERY_2, ON); // switch on if it was enabled previously
			}
			
			// 2. Check 2nd battery
			if (voltages[1] < 1000 || voltages[1] > FAST_CHARGE_END_MAX_VOLTAGE) // prevent discharging below 1000mV
			{
				// Stop Discharging this battery
				Mosfet_switch(BATTERY_2,OFF); // disable the 2nd battery
			}
			else 
			{
				uint8_t enabled = 0; // boolean flag to check whether the other battery need to be disable and re-enable or already disabled (discharged)
				// disable the 1st battery to measure voltage only on the 2nd
				if (bit_is_set(PINB,PB1)) //if it's enabled
				{
					Mosfet_switch(BATTERY_1,OFF); // Disable
					enabled = 1; // set flag
				}
				
				_delay_ms(100); // wait to stabilize voltage
				voltage_in_ADC = 0; // Reset oversampling
				voltages[1] = Measure_U(); // measure
				if (enabled) Mosfet_switch(BATTERY_1, ON); // switch on if it was enabled previously
			}
			
			UART_send_data_block(5, DISCHARGE_MODE, time.minute, time.seconds, voltages[0], voltages[1]); // Debug
			//LCD_Send_Data();
			
			// 3. Check if we need to stop discharging
			if (!bit_is_set(PINB,PB2) && !bit_is_set(PINB,PB1)) // if both batteries are discharged
			{
				// Discharge finished
				Mosfet_switch(DISCHARGE,OFF);	// Discharge OFF
				Mosfet_switch(BATTERY_1,OFF);
				Mosfet_switch(BATTERY_2,OFF);
				// Reset variables
				time.seconds = 0;
				time.minute = 0;
				battery_number = 0; // incremented in Battery_Detection routine
				isDischarged = 1; // Discharge Flag (boolean). Prevents entering again in "Discharge mode"
				// Exit from Discharge mode
				return INIT_MODE;
			}
			wdt_reset();
			previous_sec = time.seconds;
		}	
	}
	return DISCHARGE_MODE;
}

// based on voltage-level decide the next step...
uint8_t Battery_Detection(void)
{
	// Increment battery number each time
	battery_number++;
	// Disable batteries
	Mosfet_switch(BATTERY_1,OFF);
	Mosfet_switch(BATTERY_2,OFF);
	Mosfet_switch(DISCHARGE,OFF);
	Disable_PWM();
	
	uint8_t batt_detect_timeout = BATT_DETECT_TIMEOUT; // 1min
	uint8_t previous_sec = time.seconds;
	uint16_t temperature = Read_temperature(battery_number);
	uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode); // Selected charging method (FAST or TRICKLE)
	
	if ( ram_charge_mode == FASTCHARGE_MODE && battery_number > BATTERY_MAX)
	{
		 	// If both batteries are successfully charged in FAST_CHARGE_MODE switch to maintenance charge (trickle mode with C/40 current )
		return TRICKLE_MODE; // don't need to continue Battery_Detection
	}
	
	
// Detect battery...
	uart_send_FLASH_string(string2); // Batt. detection
	LCD_Send_row(0,1,string2); // with delay
	
	 // Enable selected battery
	Mosfet_switch (battery_number, ON);
	// wait 1 minute for battery detection
	while(batt_detect_timeout)
	{
		if (previous_sec != time.seconds)// Every second
		{
			voltage_in_ADC = 0; // disable oversampling, PWM disabled, therefore not needed
			uint16_t voltage = Measure_U();
				
			UART_send_data_block(5, battery_number, time.minute, time.seconds, voltage, temperature); // Debug
			
			if (voltage > BATTERY_DETECTION_VOLTAGE && (voltage < FAST_CHARGE_END_MIN_VOLTAGE+100))
			{
				// Battery detected (voltage within 0,5 and 2 Volts)
				// Switching to PRECHARGE_MODE 
				Mosfet_switch(DISCHARGE,OFF);	// OFF
				uart_send_FLASH_string(string4);
				LCD_Send_row(1, 0, string4);
				// reset variables
				time.seconds = 0;
				time.minute = 0;
				
				// If Discharge enabled in Menu and battery detected
				if (eeprom_read_word(&eep_discharge) && !isDischarged ) 
				{
					return DISCHARGE_MODE;
					// "isDischarged" flag set after DISCHARGE_MODE, prevents discharging again
				}
				
				// If Fast Charge selected in "Menu"
				// and battery number within the allowed value
				if ( ram_charge_mode == FASTCHARGE_MODE /*&& battery_number <= 2*/)
				{
					return PRECHARGE_MODE; // start with trickle charge
				}
				// If Trickle Charge selected in "Menu"
				else if ( ram_charge_mode == TRICKLE_MODE)
				{
					return TRICKLE_MODE;
				}
			}
			
			// No battery detected...
			uart_send_FLASH_string(string3);
			LCD_Send_row(1, 0, string3);
			previous_sec = time.seconds;
			batt_detect_timeout--;
		}
	}

	if (battery_number == 2)
	{
		 	// If there are no batteries in both slots within batt_detect_timeout
		Terminate(string9);
	}
			
	
	// If there is no battery in 1st slot within batt_detect_timeout
	// Switch to next battery (2nd slot)
	return INIT_MODE;
}

	
	


uint8_t init_TMP75 (uint8_t select)
{
	//uint8_t temp_ok = 1;
	/*
	switch (select)
	{
		//i2c_start returns 0 = device accessible, 1= failed to access device
		case 1:
			i2c_start(DEVICE_ADDRESS1+I2C_WRITE) ? (temp_ok = 0) :  (temp_ok = 1);
			break;
		case 2:
			i2c_start(DEVICE_ADDRESS2+I2C_WRITE) ? (temp_ok = 0) : (temp_ok = 1);
			break;
		default:
			temp_ok = 0; // wrong device number
			return 0;
			break;
	}
	*/
	
	 if( i2c_start(select+I2C_WRITE))
	 {
		//if no device found do not continue
		uart_puts_P("Temp NOK");
		uart_send_CR_LF();
		return 0; // Device NOK
		
	} 
	else
	{
		uart_puts_P("Temp OK");
		uart_send_CR_LF();
	}
	// Set 12bit reading mode
	i2c_write(0x01);
	
	i2c_write(0xB0);
	
	i2c_stop();
	
	return 1; // Device OK
}

/***************************************/
uint16_t Read_temperature(uint8_t select)
{
	uint8_t temperatures[2];
	uint16_t digit = 0;
	uint16_t decimal = 0;
	uint8_t address = 0;
	
	switch (select)
	{
		case 1:
			if (!isTempOk_1) return 0; //if no device found do not continue
			address = DEVICE_ADDRESS1;
			break;
		case 2:
			if (!isTempOk_2) return 0; //if no device found do not continue
			address = DEVICE_ADDRESS2;
			break;
		default: 
		// In trickle mode every minute toggle between both devices to check temperatures
			if (time.minute%2)
			{
				 Read_temperature(1);
			} 
			else
			{
				 Read_temperature(2);
			}
			break;
	}
	// After getting the right adress initiate communication
		if( i2c_start(address+I2C_WRITE) ) return 0; // 1= failed to access device
	
	if( i2c_write(0x00) ) return 0; // 1= write failed
	// Start reading temperature
	if( i2c_start(address+I2C_READ) ) return 0; // 1= failed to access device
	// Because the temperature is a 12bit number read sequentially MSB ...
	temperatures[0] = i2c_readAck();
	// ... and LSB
	temperatures[1] = i2c_readAck();
	// Close I2C
	i2c_stop();
	// Calculate temperature...
	digit = (uint16_t)temperatures[0];
	decimal = (uint16_t)((temperatures[1] >> 4)*625)/1000;  // 4 empty place on right side
	// right shift 2nd array by 4: 0000 0000 0000 1111
	// left shift 1st array by 4: 0000 1111 1111 0000
	// add 1st to 2nd: 0000 1111 1111 1111
	// multiply with 0.625 (or *1000, *625, /1000 withot using float)
	
	return (digit*10 + decimal); // 25.2 => 25*10 + 2 => 250 + 2 => 252
	
	//return (digit);
}



/***************************************/
void Init_PWM(uint8_t pwm){
	//max 62500 Hz @ 16MHz with 8-bit Timer. Counter2 used here
	// Resolution is 0...255 (2^8).
	// Buck-Converters running the most efficiently below 50% duty cycle, so using values between 0...127
	DDRB |= 1 << PINB3; //PB3 as output
	TCCR2 |= 1 << COM21; // non-inverting mode
	TCCR2 |= (1 << WGM21) | (1 << WGM20); //Fast PWM 8-bit
	TCCR2 |= 1 << CS20; // start with prescaler = 0
	OCR2 = pwm; // Start with given PWM value
}

/***************************************/
void Disable_PWM(void){
	TCCR2 = 0x00; // disable Timer2
	 	OCR2 = 0x00; //reset OCR2 register
	 	// disable current-flow through Buck-Converter by pulling down PWM output pin
	DDRB |= (1 << PINB3); // output
	PORTB &= ~(1<<PINB3); // pull-down

}


/***************************************/
void Modify_PWM_Width(uint16_t current, uint16_t max_current, uint8_t *pwm)
{
	if (ramp) // 10 minute timeout to increase and stabilize current
	{
		if (((current >= max_current) || (*pwm >= PWM_MAX)) && (*pwm > 0)) // if current is higher than allowed decrease pwm
		{
			 OCR2 = --(*pwm);
		} 
		else if ((current <= max_current) && (*pwm < PWM_MAX)) // if current is lower than allowed increase pwm, but only to PWM_MAX
		{
			 OCR2 = ++(*pwm);
		}
		
		ramp--;
	}
	
}

/***************************************/
void ADC_Init(void){
	//DDRC = 0x00;
	//PORTC = 0x00;
	
	// internal reference voltage as a reference for the ADC:
	//ADMUX = (1 << REFS1) | (1 << REFS0);
	
	// External reference (TL431) used here = 2.5V
	ADMUX &= ~(1 << REFS1);
	ADMUX &= ~(1 << REFS0);
	ADCSRA = (1 << ADPS2) |(1 << ADPS1) | (1 << ADPS0); // Frequency prescaler = 128 (125khz @ 16Mhz)
	//ADCSRA |= 1 << ADIE; //enable ADC interrupt
	ADCSRA |= (1 << ADEN); //Activate  ADC
	
	/* After activating the ADC a "dummy readout" recommended to "warm up" the ADC */
	ADCSRA |= (1 << ADSC); // An ADC conversion
	while (ADCSRA & (1 << ADSC)) {//wait  on completion of the conversion
	}
	/* ADCW must be read once, otherwise it will result the next conversion is not taken. */
	(void) ADCW;
}

/* ADC single measurement */
uint16_t ADC_Read (void){
	//
	_delay_us(10); // Delay to charge ADC input
	ADCSRA |= (1 << ADSC); // a "single conversion"
	while (ADCSRA & (1 << ADSC)) {//wait  on completion of the conversion
	}
	return ADCW; //read and return ADC value
}



/* ADC Multiple measurements with oversampling */
uint16_t ADC_Read_Avg (uint8_t channel, uint8_t nsamples){
	uint16_t sum = 0; // 1023*64=65'472
	
	// Choose channel without affect other bits
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	
	for (uint8_t i = 0; i <nsamples; ++ i) {
		sum += ADC_Read ();
	}
	
	return sum;
}


uint8_t Read_analog_buttons(void){
	
	//uint8_t button = 0;
	uint16_t adc = 0;
	//uint16_t prev_adc = 0;
	//uint8_t button_array[5];
	//uint8_t flag = 0;
	
	// set ADC channel and check if one of the buttons is pressed
	if ( (adc = ADC_Read () ) < 1020) // 2nd ADC channel pulled up to 5V, so it always reads around 1023
	{
		// one of the buttons is pressed
		
		// 1. method
		// Determine by comparing to previous one
		/*
		for (uint8_t i = 0; i< 5; i++)// 5 consecutive reading, if all the same => button detected
		{
			// 50ms * 5 = 250ms to identify one button
			prev_adc = adc; // save old value before overwriting it
			adc = ADC_Read(); //read ADC value
			 // Compare it with previous value
			 // if not the same => quit
				if( abs( prev_adc - adc ) > 40 ) // Allowed error rate is +-40
				{
					return 0; // EXITT
				}
			_delay_ms(50); // button debounce
		}
		*/
		
		// 2. method
		// Determine from average
		adc = 0;
		for (uint8_t i = 0; i <5; ++i)
		{
			adc += ADC_Read ();
			_delay_ms(50); // button debounce
		}
		adc /= 5; 
	
		// We have now 5 equal readings
		// Find which button was pressed from button array
		for(uint8_t j = 0; j < 5; j++) // 5 possible buttons
		{
			if( abs(pgm_read_word (&buttondata[j]) - adc ) < 40 ) // Allowed error rate is +-40
				return j+1; // button found, return it's index (buttons are indexed from 1)
		}
			 
	}
	// button not found
	return 0;
}

/***************************************/
uint16_t Measure_U(void){
	uint16_t ADC0_value_unfilt = 0;
	//uint16_t voltage0 = 0; // Voltage on ADC0
	
	ADC0_value_unfilt = ADC_Read_Avg(0, ADC_SAMPLES); // Sum of 64 ADC values from ADC0 channel
	 	// Approximation...
	if(!(voltage_in_ADC)){
		voltage_in_ADC = ADC0_value_unfilt;
		} else {
		if(voltage_in_ADC > ADC0_value_unfilt) {
			voltage_in_ADC -= (voltage_in_ADC - ADC0_value_unfilt) >> FILTER_BITS;
			} else {
			voltage_in_ADC += (ADC0_value_unfilt - voltage_in_ADC) >> FILTER_BITS;
		}
	}
	// not using resistor divider to achive the best resolution
	// calculate voltage on this pin
	return (((uint32_t) voltage_in_ADC * ADC_REF) / ADC_OVERSAMPLE_MAX );
	//voltage0 = ((uint32_t)(R1 + R2) *n voltage0) / R2 ; // calculate voltage1 before resistor divider
	//voltage = voltage0; // for Monitoring
	//return voltage0;
}


/***************************************/
uint16_t Measure_I(void){
	uint16_t ADC1_value_unfilt = 0;
	uint16_t voltage1 = 0; // Voltage on ADC1
	
	ADC1_value_unfilt = ADC_Read_Avg(1, ADC_SAMPLES); // Sum of 64 ADC values from ADC1 channel


	// Approximation...
	if(!current_in_ADC)
	{ // used first time
		current_in_ADC = ADC1_value_unfilt; // global variable to backup previous value
		}
		 else
		  {
		  		if(current_in_ADC > ADC1_value_unfilt)
		  		{
			current_in_ADC -= (current_in_ADC - ADC1_value_unfilt) >> FILTER_BITS;
			} 
			else 
			{
			current_in_ADC += (ADC1_value_unfilt - current_in_ADC) >> FILTER_BITS;
		}
	}
	// convert oversampled ADC value to volts
	voltage1 = (((uint32_t)current_in_ADC * ADC_REF) / ADC_OVERSAMPLE_MAX );	// Voltage on ADC-input pin
	/*
	voltage1 = voltage1*5; // Voltage before resistor divider, ratio is 1/5
	voltage1 = voltage1*10; // convert to mV
	voltage1 = voltage1/90; // Gain of Current Amplifier. Actually in datasheet it's 100, but after measuring with precise voltmeter turned out that it's only 90
	return voltage1/RSHUNT;	// calculate milliAmps from voltage drop on SHUNT resistor: I = U/R 
	*/
	
	return ((uint32_t)voltage1 * 500) / (9*RSHUNT); // see description above
	
}

/***************************************/
void Init_timer(void){
	// Using Timer1 to count seconds, minutes, hours.
	// Interrupt driven
	TCCR1B |= (1 << WGM12); // CTC mode	(Hardware controlled)
	TIMSK |= (1 << OCIE1A); // Enable interrupt
	//OCR1A = 15624; // interrupt on every 1 sec.	16.000.000 / 1024 = 15625. And -1, because starting from 0.
	OCR1A = 3124; // interrupt on every 1/5 = 0.2 sec.	 15625/5 = 3125. And -1, because starting from 0.
	// TODO: modify to 1562
	TCCR1B |= ((1 << CS12) | (1 << CS10)); // start with 1024 pre-scaler
}

/*******/
/*
void Battery_Enable(uint8_t batt_select, uint8_t on_off)
{
	if(on_off) // ON
	{
		switch(batt_select)
		{
			case 1:
				DDRB |= (1 << PB1); // output
				PORTB |= (1 << PB1); // high
				break;
			case 2:
				DDRB |= (1 << PB2); // output
				PORTB |= (1 << PB2); // high
				break;
			default:
				break;
		}
	}
	else // OFF
	{
		switch(batt_select)
		{
			case 1:
				DDRB |= (1 << PB1); // output
				PORTB &= ~(1 << PB1); // low
				break;
			case  2:
				DDRB |= (1 << PB2); // output
				PORTB &= ~(1 << PB2); // low
				break;
			default:
				break;
		}
	}
}
*/
/*******
void Battery_Disable (uint8_t batt_select)
{
	switch(batt_select)
	{
		case 1:
			DDRB |= (1 << PB1); // output
			PORTB &= ~(1 << PB1); // low
			break;
		case  2:
			DDRB |= (1 << PB2); // output
			PORTB &= ~(1 << PB2); // low
			break;
		default:
			break;
	}
}
*******/
void Mosfet_switch (uint8_t pin, uint8_t on_off)
{
	// pins are set as output at initialization
	// TODO: set them
			if(on_off)
				PORTB |= (1 << pin); // pull high
				else
				 PORTB &= ~(1 << pin); // pull low
		
	
}

/*
void Discharge_Enable (uint8_t on_off)
{
	switch(on_off)
	{
		case 0: // off
			DDRB |= (1 << PB0); // output
			PORTB &= ~(1 << PB0); // low
			break;
		case 1: // on
			DDRB |= (1 << PB0); // output
			PORTB |= (1 << PB0); // high
			break;
		default:
		break;
	}
}
*/

void Terminate(const char *str)
{
	// EXIT
	Disable_PWM();
	// Disable MOSFETs
	Mosfet_switch(DISCHARGE,OFF);
	Mosfet_switch(BATTERY_1,OFF);
	Mosfet_switch(BATTERY_2,OFF);
	// Disable watchdog in order to prevent MCU from waking up
	wdt_disable();
// Show last message	
	uart_send_FLASH_string(str);
	LCD_Send_row(0,1,str);// delay added to assure everything is sent
	cli(); // disable interrupts
		// Sleep MCU ...
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_cpu();
}


void uart_send_integer (int i)
{
	itoa(i,buffer,10);
	uart_puts(buffer);
	uart_send_CR_LF();
}


/**************************************
void uart_send_RAM_string(const char *str){
	//
	uart_puts(str);
	uart_send_CR_LF();
}
*/
/***************************************/
void uart_send_CR_LF(void){
	// Send Carriage return and Line feed
	uart_putc('\r');
	uart_putc('\n');
}

/***************************************/
void uart_send_FLASH_string(const char *str){
	//
	uart_puts_p(str);
	uart_send_CR_LF();
}

/***************************************/
void UART_send_data_block(uint8_t n_args, ...){
	
	register uint8_t i;
	va_list ap;
	
	va_start(ap, n_args);
	
	for(i = 1; i <= n_args; i++) {
		
		itoa(va_arg(ap, int),buffer,10);
		uart_puts(buffer);
		
		uart_putc(';');
		uart_putc(' ');
		//uart_puts("; ");
	}
	
	va_end(ap);
	/*
	itoa(voltage,buffer,10);
	uart_puts(buffer);
	uart_puts(" mV; ");
	
	itoa(current,buffer,10);
	uart_puts(buffer);
	uart_puts(" mA; ");
	
	itoa(temperature,buffer,10);
	uart_puts(buffer);
	uart_puts(" C; ");
	
	itoa(minutes,buffer,10);
	uart_puts(buffer);
	uart_puts(" min; ");
	
	itoa(seconds,buffer,10);
	uart_puts(buffer);
	uart_puts(" sec; ");
	
	itoa(pwm,buffer,10);
	uart_puts(buffer);
	uart_puts(" PWM; ");
	
	uart_puts("mode: ");
	itoa(mode,buffer,10);
	uart_puts(buffer);
	uart_puts(";");
	
	uart_puts("peak_db: ");
	itoa(peak_db,buffer,10);
	uart_puts(buffer);
	uart_puts(";");
	
	uart_puts("peak_voltage: ");
	itoa(peak_voltage,buffer,10);
	uart_puts(buffer);
	uart_puts(";");
	
	uart_puts("dv_zero_db: ");
	itoa(dv_zero_db,buffer,10);
	uart_puts(buffer);
	uart_puts(";");
	
	uart_puts("dv_neg_db: ");
	itoa(dv_neg_db,buffer,10);
	uart_puts(buffer);
	uart_puts(";");
	*/
	uart_send_CR_LF();
	
}


/*************************************************************************
Send a row to LCD
Input:    row_number 0: 1st, 1: 2nd
Returns:  none
*************************************************************************/
void LCD_Send_row(uint8_t row_number, uint8_t isDelayEnabled, const char *str)
{
	//if(lcd_status){
		//1. clear only the 1st row
		lcd_clrscr();
		/*
		lcd_gotoxy(0,row_number);
		for(uint8_t i = 0; i< 16; i++)
		{
			lcd_putc(' ');
		}
		*/
		//2. show string on LCD from program memory in the given row.
		lcd_gotoxy(0,row_number);
		lcd_puts_p(str);
		if (isDelayEnabled)
		{
			// additional delay if enabled
			_delay_ms(1000);
		}
		
	//}
}

// fill lcd array
// TODO: Check if numbers could be sent without converting to string
void lcd_fill_array (uint8_t min, uint8_t max, uint16_t number, char *lcd)
{
	
 for (uint8_t x = max; x >= min; x--)
	{
		if(x==min+1 && x != 13) // 2nd position is a "," symbol, except for temperature (12...13)
		{
			lcd[x] = ',';
			continue; // skip below parts to jump to the next position
		}
		 		number %= 10; // calculate digit
		    lcd[x] = number + 48; // ASCII "0" = 48
		 	number = number/10; // drop last number
	}
	
	
}
/***************************************/
void LCD_Send_Debug(uint16_t volts, uint16_t current, uint16_t temperature)
{
	//if(!lcd_status) return; //if LCD not found do not continue
	
	 	lcd_gotoxy(0,1); // 2nd row
	
	// array for LCD's 2nd row
	char lcd[16];
	 	
	 	// TODO: delete, use function parameters
	uint16_t lcd_temp, lcd_volt, lcd_cur; // temp variables
	//char c[8]; // string buffer
	//lcd_clean_row(1); // clear only the 2nd row
	// TODO: cut above function

	// Show Voltage, format: [1,25V] 0...4

	//lcd_gotoxy(4,1); //second row, 4th position
	//lcd_puts("V");
	
	lcd_volt = volts; // temp

	 		lcd_volt = lcd_volt/10; // drop last number
	 		
	 		 	lcd_fill_array(0,3, lcd_volt, lcd  );
 //x = 3; // starting from 3-rd position (last digit)
/*
 for (uint8_t x = 3; x >= 0; x--)
	{
	
		if(x==1) // 2nd position is a "," symbol
		{
			lcd[x] = ',';
			//lcd_gotoxy(x,1);
			//lcd_puts(",");
			//lcd_putc(44); // ASCII ","
			//x--;
			continue;
		}
		
		 		number = lcd_volt%10; // calculate digit
		 		 		//itoa(number,c,10); // convert to string
		    lcd[x] = number + 48;
		
		//lcd_gotoxy(x,1); // put digit in this place
		//lcd_puts(c);
		//lcd_putc(number + 48) // in ASCII table 0 is 48
		//x--; //decreasing position
		
		 	lcd_volt = lcd_volt/10; // drop last number
	}
	*/
	 	lcd[4] = 'V';
	lcd[5] = ' '; // space between each section
	// Show current
	// [6...10] (1.56A)
	//lcd_gotoxy(10,1); //second row, 4th position
	//lcd_puts("A");

	lcd_cur = current;
	lcd_cur /= 10;
	lcd_fill_array(6, 9, lcd_cur, lcd  );
	/*
	x = 9; // starting from 3-rd position (last digit)
	while (x>5)
	{
		lcd_cur = lcd_cur/10; // drop last number, because we don't need all 4 digits
		//only 3
		 number = lcd_cur%10; // calculate last digit
		//itoa(number,c,10); // convert to string
		if(x==7) // 7th position is a "," symbol
		{
			lcd[x] = ',';
			//lcd_gotoxy(x,1);
			//lcd_puts(",");
			x--;
		}
		//lcd_gotoxy(x,1); // put digit in this place
		//lcd_puts(c);
		 lcd[x] = number + 48;
		x--; //decreasing position
	}
	*/
	 	lcd[10] = 'A';
	 	lcd[11] = ' ';
	 
	//lcd_gotoxy(14,1); //second row, 4th position
	//lcd_puts("C");
	
	lcd_temp = temperature;
	lcd_fill_array(12, 13, lcd_temp, lcd);
	/*
	x = 13; // starting from 3-rd (last) digit
	while(x>11)
	{
		lcd_temp = lcd_temp/10; // drop last number
		uint8_t number = (uint8_t)lcd_temp%10; // calculate digit
		itoa(number,c,10); // convert to string
		
		lcd_gotoxy(x,1); // put digit in this place
		lcd_puts(c);
		x--; //decreasing position
	}
	*/
	 	lcd[14] = 'C';
	 	lcd[15] = ' ';
	 	
	 // Show the array on LCD
	 	lcd_puts(lcd);
}

/*
 void Send_16bit_integer_to_LCD_and_UART(uint16_t integer, const char *str)
{
	itoa(integer,buffer,10);
	
	uart_send_FLASH_string(str);
	uart_send_RAM_string(buffer);
	
	lcd_clrscr();
	lcd_home();
	lcd_puts_p(str);
	lcd_gotoxy(0,1); // 2nd row
	lcd_puts(buffer);
	_delay_ms(1000);
}
*/
//Send information about charging times
/*
void Send_charging_info(uint16_t time_in_precharge_mode, uint16_t time_in_fastcharge_mode, uint16_t time_in_topoffcharge_mode, uint16_t time_in_trickle_mode, uint16_t mAh)
{
	Disable_PWM();
	Battery_Enable(1,0);
	Battery_Enable(2,0);
	
	Send_16bit_integer_to_LCD_and_UART(time_in_precharge_mode, string10);
	
	Send_16bit_integer_to_LCD_and_UART(time_in_fastcharge_mode, string11);
	
	Send_16bit_integer_to_LCD_and_UART(time_in_topoffcharge_mode, string12);
	
	Send_16bit_integer_to_LCD_and_UART(time_in_trickle_mode, string13);
	
	uint16_t time_total = time_in_precharge_mode + time_in_fastcharge_mode + time_in_topoffcharge_mode + time_in_trickle_mode;
	Send_16bit_integer_to_LCD_and_UART(time_total, string14);
	
	Send_16bit_integer_to_LCD_and_UART(mAh, string15);
}
*/

/***************************************/
uint8_t Check_battery_status(uint8_t battery_number, uint16_t voltage, uint16_t current, uint16_t *temperature, uint16_t *time_left_in_this_mode)
{

	// Temperature check START
	if (time.seconds == 0){ // Every minute
	 	// backup old temperature value
	uint16_t old_temperature = *temperature;
		// Measure current temperature to compare with previous value
		uint16_t temp = Read_temperature(battery_number);
		
		if (temp) // if read successful >0
		{
			*temperature = temp;
			// Continue only after NDV_PREVENTIVE_DELAY
			// Active in all modes, except in TRICKLE
			// TODO: enable in all mode
			if (time.minute >=  NDV_PREVENTIVE_DELAY && mode != TRICKLE_MODE)
			{
				// If current temperature 1 Celsius bigger 
				// or 15 Celsius bigger than the starting temperature
				// or bigger then 40 Celsius
				// Stop charging
				if ( (*temperature > old_temperature) && ( ((*temperature - old_temperature) > 10) || (*temperature > 400) )){
					// 315 - 305 = 10. current=>31.5 C | previous=>30.5 C | delta => 1 C
					
					uart_puts_P("Temp!");
					uart_send_CR_LF();
					// old temp visible from Debug block
					// uart_send_integer(old_temperature);
					uart_send_integer(*temperature);
					Disable_PWM();
					// Disable battery
					Mosfet_switch(battery_number,OFF);
					
					// select next battery
					return INIT_MODE; 
					// trickle charge is done after temperature termination, but only if all batteries are already Fast Charged
				}
			}
		}
	}
	//Temperature check END

	
	// Voltage check START
	switch (mode)
	{
		case PRECHARGE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE || voltage >  FAST_CHARGE_END_MAX_VOLTAGE)
			{ 
			// battery removed
				uart_send_FLASH_string(string3);
				LCD_Send_row(0,1,string3);
				// switch to next battery
				return INIT_MODE;
			}
			// If the battery charged to a safe level continue with FastCharge Mode
			if (voltage >= PRECHARGE_TERMINATION_VOLTAGE)
			{
				return  FASTCHARGE_MODE;
			}
			// Debug: battery has to be precharged in 10 minutes
			*time_left_in_this_mode = PRECHARGE_END_TIMER - time.minute;
			
			if (	*time_left_in_this_mode)
			{
				// if precharge not successful
				uart_send_FLASH_string(string19);
				LCD_Send_row(0,1,string19);
				// switch to next battery
				return INIT_MODE;
			}
	
			 
			break;
			
		case FASTCHARGE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE || voltage >  FAST_CHARGE_END_MAX_VOLTAGE) // battery removed
			{
				uart_send_FLASH_string(string3);
				LCD_Send_row(0,1,string3);
				return INIT_MODE;
			}
			
			 			//Debug:
			*time_left_in_this_mode = FASTCHARGE_END_TIMER - time.minute;
			// If FDV, NDV or temperature termination doesn't activated stop charging after FASTCHARGE_END_TIMER seconds
			if (	*time_left_in_this_mode ) // max time reached
			{
				uart_send_FLASH_string(string19);
				LCD_Send_row(0,1,string19);
				// switch to next battery
				return INIT_MODE;
			}

			
			break;
		
		
		case TRICKLE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE || voltage >  FAST_CHARGE_END_MAX_VOLTAGE)
			{
				 // battery removed
				Terminate(string3); // exit
			}
			
			/*uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
			if ( ram_charge_mode == TRICKLE_MODE && minutes > trickle_end_timer)
			{
				Terminate(string19);
				
			} 
			
			else */
			// Trickle-timeout depends witch mode is selected from menu (Fast or Trickle)
			// If Fast Charging selected: Trickle mode only used to maintain batteries for 6 hours
			// If Trickle Charging selected: timeout given by the user (stored in EEPROM)
			
			//Debug: send through UART how many seconds left in this mode before it stops.
			*time_left_in_this_mode = trickle_end_timer - time.seconds;
			
			 			if(*time_left_in_this_mode)
			{
				// Trickle timeout activated
				// Stop charger
				Terminate(string19);
			}
			
			break;
		
		default:
			break;
	}
	
	
	// NDV and FDV detection START
	// Wait 15 minutes in FASTCHARGE_MODE before checking NDV or FDV (stabilizing battery, timeout defined in NDV_PREVENTIVE_DELAY)
	//if  voltage >= 1475 mV the battery is close to the end of full charge, defined in FAST_CHARGE_END_MIN_VOLTAGE

	
		if ((mode == FASTCHARGE_MODE) && (time.minute >=  NDV_PREVENTIVE_DELAY) && (voltage >= FAST_CHARGE_END_MIN_VOLTAGE))
		{
			// Update voltage peak if necessary
			if(voltage > peak_voltage)// voltage is rising
			{
				// actual voltage higher than the last one
				// wait 5 sec before updating "peak_voltage"
				// timeout defined in "peak_db"
				// peak_db => Number in a row charging voltage higher that the last peak voltage
				if(peak_db) // if > 0
				{
					peak_db--;
					// wait 5sec before updating "peak_voltage"
				}
				else // if = 0
				{
					// If after 5sec battery voltage still higher store it in "peak_voltage"
					peak_voltage += (voltage - peak_voltage) / 2 + 1;
					// restore initial values (reset)
					peak_db = PEAK_UPDATE_SAMPLES;// 5sec
					fdv_db = FDV_SAMPLES; //15min
				}
			}
			else
			{	// when voltage <= peak_voltage, voltage is decreasing or constant
			// reset peak_db to trigger FDV condition
				peak_db = PEAK_UPDATE_SAMPLES;
			}

			//check for flat delta V condition
			if(fdv_db)
			{
				// wait 15min before triggering FDV
				fdv_db--;
				//Debug("Voltage is flat!");
			}
			else
			{
				uart_puts_P("FDV!");
				uart_send_CR_LF();
				// stop charging actual battery
				// switch to next one
				// in INIT_MODE we are checking if next battery inserted
				return INIT_MODE;
			}
			
			//check for negative delta v condition
			if(voltage <  (peak_voltage - NDV_DROP))
			{ // 1500 < (1506 - 5) last peak is 1506
				if(neg_db)
				{
					// wait 5sec before triggering NDV
					neg_db--;
				}
				else
				{
					// if aftef 5 sec NDV condition still true
					uart_puts_P("NDV!");
					uart_send_CR_LF();
					// stop charging actual battery
					// switch to the next one
					return INIT_MODE;
				}
			}
			else
			{
				// if within 5sec voltage increased reset neg_db
				neg_db = NDV_SAMPLES;
			}
			
		}// NDV and FDV detection END
	
	return mode; // return same "mode" to stay in actual charging phase
}


  // 
void handle_mcucsr(void)
{
	//After a watchdog reset restore the value of these variables to continue charging where it was interrupted.
  mcucsr = MCUCSR; // in this register stored the reset source. At start-up check if it was occured from watchdog.
  MCUCSR = 0; // reset
  saved_batt_num = battery_number; // continue with last battery
  saved_mode = mode; // continue with last mode
  saved_minute = time.minute;
}

// Timer1 interrupt	for counting sec & min
ISR(TIMER1_COMPA_vect){
	// TODO: change to 1/10 seconds
	//configured to interrupt after every 1/5 seconds
	// We need it because Battery Voltage and Current calculated through approximation in order to prevent reading incorrect values ( PWM generates a lot of spikes, which could accidently trigger one of the charge termination methods). Therefore it must called as fast as it can be.
	
	time.timer1_tick++;
	if (time.timer1_tick == 5) 
	{	
		time.timer1_tick = 0; // reset every second
		time.seconds++;	
	
		if (time.seconds == 60){
			   time.seconds = 0; // reset every minute
			   time.minute++;
		}
		
	}		
}