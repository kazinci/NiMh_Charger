/*
 * Amp_Meter.c
 *
 * Created: 2014.08.17. 15:05:06
 *  Author: Kazo
 */ 

/************************************************************************/
/*                               DEFINES                                */
/************************************************************************/
#define F_CPU 16000000UL // CPU frequency
#define UART_BAUD_RATE      9600 // UART baud-rate

#define DEVICE_ADDRESS1  0x90      // I2C 1st device address
#define DEVICE_ADDRESS2  0x9E      // I2C 2nd device address

//#define DISPLAY_STRING(STRING) {lcd_clrscr();lcd_puts_p(STRING);uart_puts_p(STRING);uart_putc('\r');uart_putc('\n');_delay_ms(1000);}

#define BATTERY_CAPACITY 2500 // battery max mAh

#define RSHUNT 55 //Shunt-resistance in milli-Ohms
//#define R1 12000 //Resistor divider in Ohms (top resistor)
//#define R2 3000 //Resistor divider in Ohms (lower resistor)
//#define GAIN 100

#define NDV_DROP							5 // mV
//#define NDV_DROP_IN_ADC						((unsigned long) NDV_DROP * ADC_OVERSAMPLE_MAX / ADC_REF)// voltage drop for detecting negative delta V in ADC counts

#define BATTERY_DETECTION_VOLTAGE			500 // mV
//#define BATTERY_DETECTION_VOLTAGE_IN_ADC	((unsigned long) BATTERY_DETECTION_VOLTAGE * ADC_OVERSAMPLE_MAX / ADC_REF) 

#define PRECHARGE_TERMINATION_VOLTAGE		1000 //mV
#define PRECHARGE_TERMINATION_VOLTAGE_IN_ADC	((unsigned long) PRECHARGE_TERMINATION_VOLTAGE * ADC_OVERSAMPLE_MAX / ADC_REF) 

#define FAST_CHARGE_END_MIN_VOLTAGE			1400//1475 // mV
#define FAST_CHARGE_END_MIN_VOLTAGE_IN_ADC	((unsigned long) FAST_CHARGE_END_MIN_VOLTAGE * ADC_OVERSAMPLE_MAX / ADC_REF) 

#define FAST_CHARGE_END_MAX_VOLTAGE			2000 // mV
#define FAST_CHARGE_END_MAX_VOLTAGE_IN_ADC	((unsigned long) FAST_CHARGE_END_MAX_VOLTAGE * ADC_OVERSAMPLE_MAX / ADC_REF) 

#define NDV_SAMPLES							10 // number of samples needed for NDV
#define FDV_SAMPLES							(uint16_t)15*60	// 10 minutes required for flat delta V condition 
#define PEAK_UPDATE_SAMPLES					5 // Number in a row charging-voltage higher that the last peak voltage needed to update

#define NDV_PREVENTIVE_DELAY				15 //minutes, used at the beginning of FASTCHARGE
#define PRECHARGE_END_TIMER					10 // minutes, PRECHARGE termination timer
#define FASTCHARGE_END_TIMER				210 // in minutes, FASTCHARGE termination timer
#define TOPOFFCHARGE_END_TIMER				60 // minutes, TOPOFFCHARGE termination timer
#define MAINTENANCE_END_TIMER				(uint16_t)6*60 // minutes, MAINTENANCE termination timer
#define BATTERY_COOLDOWN_TIMER				10 // minutes, used after temperature termination
#define RAMP_UP_TIME						30 // * 10 sec = 300 sec (5 minutes), PWM ramp up time
#define BATT_DETECT_TIMEOUT					60 // seconds, time for detecting a battery
//		*** Voltage reading filter definitions ***
//		Normally the values shouldn't be changed unless you have an ADC
//		with a different number of bits or you plan to use a different number
//		of read samples.

#define ADC_REF				2500 // mV, measured with multimeter at running ADC
#define ADC_SAMPLES			64  // samples for averaging
//#define ADC_RESULT_SHIFT 	6
#define	ADC_OVERSAMPLE_MAX	65535	// ADC max value for oversampling 64*1024
//#define	ADC_MAX				1023	// ADC max value, 10bit 0-1023
#define	FILTER_BITS			6 //used for oversampling
//#define	FILTER_MAX			1023
//#define	FILTER_SAMPLES		1

// Charger modes used in state machine
#define MENU_MODE			0
#define INIT_MODE			1
#define DISCHARGE_MODE		2
#define PRECHARGE_MODE		3
#define FASTCHARGE_MODE		4
#define TOPOFFCHARGE_MODE	5
#define TRICKLE_MODE	6
#define END_MODE			7

// EEPROM CHARGE MODES
#define EEP_FAST_CHARGE 1
#define EEP_TRICKLE_CHARGE 2
// Time for trickle charge
#define EEP_TRICKLE_CHARGE_TIME (uint16_t)16*60 // 16 hour charging time = 960 minutes
// Buttons
#define LEFT	1
#define SELECT	2
#define DOWN	3
#define RIGHT	4
#define UP		5

// PWM defines
// the maximum value 255 (8 bit, 2^8 = 256, -1 because starting from 0)
//#define PWM_PRECHARGE		255/10 // C/10, 2500/10=250 mA
//#define PWM_FASTCHARGE		255/2 // max 50% duty cycle allowed, => 1200mA
//#define PWM_TOPOFFCHARGE	255/10 //
//#define PWM_MAINTENANCE		255/20 // C/40, 2500/40=62.5 mA
#define PWM_MAX				255/2 // max 50% duty cycle with Buck-converter
/*
#define PRECHARGE_MODE_MAX_CURRENT		BATTERY_CAPACITY/10 //mA
#define FASTCHARGE_MODE_MAX_CURRENT		BATTERY_CAPACITY/2 //mA
#define TOPOFFCHARGE_MODE_MAX_CURRENT	BATTERY_CAPACITY/10 //mA
#define MAINTENANCE_MODE_MAX_CURRENT	BATTERY_CAPACITY/40 //mA
*/


/************************************************************************/
/*                              INCLUDES                               */
/************************************************************************/

 
#include <avr/io.h>
#include <stdlib.h> // abs(), itoa();
#include <stdarg.h> // variable argument number
#include <avr/pgmspace.h> // PROGMEM
#include <avr/eeprom.h> // EEMEM
#include <avr/interrupt.h> 
#include <util/delay.h>
#include <avr/sleep.h> // sleep when END_MODE reached
#include <avr/wdt.h>
#include "lcd.h"
#include "uart.h" // Debug
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
uint8_t		peak_db = PEAK_UPDATE_SAMPLES; // If voltage not rising "peak_db"-times = > update "peak_voltage" value
uint16_t	peak_voltage = 0; // if voltage is rising, new voltage value saved here
// ADC oversampling
uint16_t	voltage_in_ADC = 0; 
uint16_t	current_in_ADC = 0; 
uint8_t		ramp = RAMP_UP_TIME; //seconds for pwm
//changed by interrupt-routine, must be volatile
uint8_t volatile	seconds = 0; // 0...59
uint16_t volatile	minutes = 0; // 0...59
uint8_t volatile	timer1_tick = 0; // 0...4
// Hardware components check
uint8_t		temp_ok = 0; // assuming I2C devices are OK ( =>1 ), later we will check if it is really OK or not
uint8_t		lcd_ok = 0; // assuming LCD device are OK ( =>1 ), later we will check if it is really OK or not
uint8_t		isDischarged = 0; // to hold discharged status flag (boolean)



uint16_t EEMEM eep_batt_capacity = BATTERY_CAPACITY;
uint16_t EEMEM eep_charge_mode = EEP_FAST_CHARGE;
uint16_t EEMEM eep_trickle_charge_time = EEP_TRICKLE_CHARGE_TIME;


// Buttons								left,	select,	down,	right,	up 
const uint16_t buttondata[5] PROGMEM ={	176,	87,		650,	960,	0};
//String constants:
const char string0[] PROGMEM = "DISCHARGING";
const char string1[] PROGMEM = "BATTERY CHARGER";
const char string2[] PROGMEM = "BATT. DETECTION";
const char string3[] PROGMEM = "NO BATTERY";
const char string4[] PROGMEM = "BATT. DETECTED";
const char string5[] PROGMEM = "PRECHARGE";
const char string6[] PROGMEM = "FASTCHARGE";
const char string7[] PROGMEM = "TOPOFF";
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
const char string19[] PROGMEM = "MAINTENANCE_END_TIMER";
const char string20[] PROGMEM = "PRECHARGE_END_TIMER";
const char string21[] PROGMEM = "FASTCHARGE_END_TIMER";
const char string22[] PROGMEM = "TOPOFFCHARGE_END_TIMER";
const char string23[] PROGMEM = "BATT_CAP_NOK";
const char string24[] PROGMEM = "MODE_NOK";
const char string25[] PROGMEM = "TIME_NOK";
 
 /************************************************************************/
 /*                           FUNCTION PROTOTYPES                        */
 /************************************************************************/
  
static void Init_timer(void);
 
static uint8_t init_TMP75 (uint8_t select);
static uint16_t Read_temperature(uint8_t temp_ok, uint8_t select);

static void Init_PWM(uint8_t pwm);
static void Disable_PWM(void);
static void Modify_PWM_Width(uint16_t current, uint16_t max_current, uint8_t *pwm);

static void ADC_Init(void);
static uint16_t ADC_Read (void);
static uint16_t ADC_Read_Avg (uint8_t channel, uint8_t nsamples);
static uint16_t ADC_Read_channel (uint8_t channel);

static uint16_t Measure_U(void);
static uint16_t Measure_I(void);

static void Battery_Enable (uint8_t batt_select, uint8_t on_off);
//static void Battery_Disable (uint8_t batt_select);
static void Discharge_Enable (uint8_t on_off);
static void Terminate(uint8_t *mode, uint8_t *batt_num, const char *str);
static uint8_t Discharge_Battery (void);
static uint8_t Charge_Battery(uint8_t this_mode, uint8_t battery_number, uint16_t max_current, uint16_t* time_in_this_mode, uint16_t *mah, const char *string_for_display);
static uint8_t Battery_Detection(uint8_t *batt_num);
static uint8_t Check_battery_status(uint8_t battery_number, uint16_t voltage, uint16_t current, uint16_t *temperature, uint8_t this_mode);

static void uart_send_integer (int i);
static void uart_send_RAM_string(const char *str);
static void uart_send_FLASH_string(const char *str);
static void uart_send_CR_LF(void);
static void UART_send_data_block(uint8_t n_args, ...);

static uint8_t Init_LCD (void);
static void lcd_send_integer(uint8_t row_number, uint16_t *integer);

/**
 @brief    Send one row to LCD
 
 @param    row_number 0: 1st, 1: 2nd
 @return   none
*/
static void LCD_Send_row(uint8_t row_number, uint8_t isDelayEnabled, const char *str);
static void lcd_clean_row (uint8_t row_number);
static void LCD_Send_Debug(uint16_t volts, uint16_t current, uint16_t temperature);


static void Send_16bit_integer_to_LCD_and_UART(uint16_t integer, const char *str);

static uint8_t Read_analog_buttons(void);
static void Button_Left(uint16_t *value);
static void Button_Right(uint16_t *value);
static void Button_Up_Down(uint16_t eep_value, uint16_t ram_value, uint8_t *save_flag, uint8_t *isMenuSelected, uint8_t *menuItem, uint8_t next_menu);

static uint8_t Menu (void);
//initialize watchdog
static void WDT_Init(void);
static void WDT_Off(void);



/************************************************************************/
/*                         MAIN FUNCTION START                          */
/************************************************************************/
int main (void) {	
	// LOCAL VARIABLES	
	
	
	uint16_t	mah = 0; // absorbed mAh's by battery, summed after every mode 
	
	uint8_t		battery_number = 1;// selected battery
	uint8_t		mode = 0; //charger modes, see defines above
	
	// These values are calculated from EEPROM values
	uint16_t precharge_max_current = 0;
	uint16_t fastcharge_max_current = 0;
	uint16_t topoffcharge_max_current = 0;
	uint16_t trickle_max_current = 0;
		
	uint16_t		time_in_precharge_mode = 0;
	uint16_t		time_in_fastcharge_mode = 0;
	uint16_t		time_in_topoffcharge_mode = 0;
	uint16_t		time_in_trickle_mode = 0;
	
	
	mode = MENU_MODE; // default mode for start-up
	
	/* Initializations */
	Battery_Enable(1,0);
	Battery_Enable(2,0);
	Discharge_Enable(0);
	Disable_PWM();
	UART_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); // For Debug	
	ADC_Init();
	lcd_ok = Init_LCD();	// LCD initialization
	i2c_init();    // init I2C interface
	Init_timer();	// Timer for counting seconds + minutes
	
	sei(); // Enable global interrupts 		
	
	Read_analog_buttons(); // dummy reading, needed for stabilizing ADC;
	while (mode == MENU_MODE)
	{
		// mode depends from user selection
		mode = Menu();
	}
	
	// After "Settings" read out new values from EEPROM		
		
	// create new scope {} to destroy "ram_batt_capacity" variable for RAM saving purpose
	// after exiting from this scope "ram_batt_capacity" variable not accessible anymore
	{
		// "ram_batt_capacity" only needed for calculating other values
		uint16_t ram_batt_capacity = eeprom_read_word(&eep_batt_capacity);
		
		if (ram_batt_capacity > 0x0000 && ram_batt_capacity < 0xFFFF)
		{
			precharge_max_current = ram_batt_capacity / 10;
			fastcharge_max_current = ram_batt_capacity / 2;
			topoffcharge_max_current = ram_batt_capacity / 10;
			trickle_max_current = ram_batt_capacity / 40;
		}
		else // If there is no appropriate value found in EEPROM
		{
			Terminate(&mode,&battery_number,string23);
		}
		
		// Charge mode
		uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
		if (ram_charge_mode == 0x0000 && ram_charge_mode == 0xFFFF)
		{
			// If there is no appropriate value found in EEPROM
			Terminate(&mode,&battery_number,string24);
		}
		
		// Trickle Charging Time
		uint16_t ram_trickle_charge_time = eeprom_read_word(&eep_trickle_charge_time);
		if (ram_trickle_charge_time == 0x0000 && ram_trickle_charge_time == 0xFFFF)
		{
			// If there is no appropriate value found in EEPROM
			Terminate(&mode,&battery_number,string25);
		}
	}
	
	
		
	/************************************************************************/
	/*                                MAIN LOOP                             */
	/************************************************************************/
	while (1) {		
		// clean variables
		time_in_precharge_mode = 0;
		time_in_fastcharge_mode = 0;
		time_in_topoffcharge_mode = 0;
		time_in_trickle_mode = 0;
		
		mah = 0;
		neg_db = NDV_SAMPLES;
		fdv_db = FDV_SAMPLES;
		peak_db = PEAK_UPDATE_SAMPLES;
		ramp = RAMP_UP_TIME;
		peak_voltage = 0;
		voltage_in_ADC = 0;
		current_in_ADC = 0;
		
		lcd_clrscr();
		lcd_home();
		uart_send_FLASH_string(string1);// used to indicate: the program stepped into the main loop
		LCD_Send_row(0, 1, string1); // with delay
		/*
		switch (mode)
		{
			case INIT_MODE: // Battery detection
				mode = Battery_Detection(&battery_number);	
				break;
			case DISCHARGE_MODE: // if voltage > 1000 mV
				mode = Discharge_Battery();
				break;
			case PRECHARGE_MODE: 	// If battery found and voltage < 1000mV 
				mode = Charge_Battery(PRECHARGE_MODE, battery_number, precharge_max_current, &time_in_precharge_mode, &mah, string5);
				break;
			case FASTCHARGE_MODE: 	// If battery voltage > 1000mV
				mode = Charge_Battery(FASTCHARGE_MODE, battery_number, fastcharge_max_current, &time_in_fastcharge_mode, &mah, string6);
				break;
			case TOPOFFCHARGE_MODE: // If charging ended with temperature termination 
				mode = Charge_Battery(TOPOFFCHARGE_MODE, battery_number, topoffcharge_max_current, &time_in_topoffcharge_mode, &mah, string7);
				break;
			case TRICKLE_MODE: // If charging ended with NDV or FDV termination
				mode = Charge_Battery(TRICKLE_MODE, battery_number, trickle_max_current, &time_in_trickle_mode, &mah, string8);
				break;
			case END_MODE: // End of charging...
				uart_send_FLASH_string(string9);
				Disable_PWM();
				Battery_Enable(1,0);
				Battery_Enable(2,0);
				
				//Send information about charging times
				
				Send_16bit_integer_to_LCD_and_UART(time_in_precharge_mode, string10);
				
				Send_16bit_integer_to_LCD_and_UART(time_in_fastcharge_mode, string11);
				
				Send_16bit_integer_to_LCD_and_UART(time_in_topoffcharge_mode, string12);
				
				Send_16bit_integer_to_LCD_and_UART(time_in_trickle_mode, string13);
				
				uint16_t time_total = time_in_precharge_mode + time_in_fastcharge_mode + time_in_topoffcharge_mode + time_in_trickle_mode;
				Send_16bit_integer_to_LCD_and_UART(time_total, string14);
								
				Send_16bit_integer_to_LCD_and_UART(mah, string15);
				
				// If only the 1st battery was charged yet
				if (battery_number == 1)
				{
					// switch to the 2nd battery
					battery_number = 2;
					
					mode = INIT_MODE;
				}
				
				while(mode == END_MODE)
				{
					lcd_clrscr();
					lcd_home();
					LCD_Send_row(0, 1, string9); // with delay
					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
					sleep_enable();
					sleep_cpu();
				}
				break;
		}
		*/
		// Battery detection
		if(mode == INIT_MODE)
		{
			mode = Battery_Detection(&battery_number);		
		}		
		
		// if voltage > 1000 mV
		if(mode == DISCHARGE_MODE){
			mode = Discharge_Battery();		
		}
		
		// If battery found and voltage < 1000mV then => PRECHARGE
		if(mode == PRECHARGE_MODE){			
			mode = Charge_Battery(PRECHARGE_MODE, battery_number, precharge_max_current, &time_in_precharge_mode, &mah, string5);
		}
		
		// If battery voltage > 1000mV => FASTCHARGE
		if(mode == FASTCHARGE_MODE){
			mode = Charge_Battery(FASTCHARGE_MODE, battery_number, fastcharge_max_current, &time_in_fastcharge_mode, &mah, string6);
		}
		
		// If charging ended with temperature termination TOPOFF charging required
		if(mode == TOPOFFCHARGE_MODE){
			mode = Charge_Battery(TOPOFFCHARGE_MODE, battery_number, topoffcharge_max_current, &time_in_topoffcharge_mode, &mah, string7);
		}
		
		// If charging ended with NDV or FDV termination TRICKLE charging required
		if(mode == TRICKLE_MODE){	
			mode = Charge_Battery(TRICKLE_MODE, battery_number, trickle_max_current, &time_in_trickle_mode, &mah, string8);
		}
		
		// End of charging...
		if(mode == END_MODE){
			uart_send_FLASH_string(string9);	
			Disable_PWM();	
			Battery_Enable(1,0);	
			Battery_Enable(2,0);	
			
			//Send information about charging times
			Send_16bit_integer_to_LCD_and_UART(time_in_precharge_mode, string10);
			
			Send_16bit_integer_to_LCD_and_UART(time_in_fastcharge_mode, string11);
			
			Send_16bit_integer_to_LCD_and_UART(time_in_topoffcharge_mode, string12);
			
			Send_16bit_integer_to_LCD_and_UART(time_in_trickle_mode, string13);
			
			uint16_t time_total = time_in_precharge_mode + time_in_fastcharge_mode + time_in_topoffcharge_mode + time_in_trickle_mode;
			Send_16bit_integer_to_LCD_and_UART(time_total, string14);
			
			Send_16bit_integer_to_LCD_and_UART(mah, string15);
			
			//itoa(time_in_precharge_mode,buffer,10);
			//uart_send_FLASH_string(string10);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string10); 
			//LCD_Send_row(1, 1, buffer); // with delay
			//
			//itoa(time_in_fastcharge_mode,buffer,10);
			//uart_send_FLASH_string(string11);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string11);
			//LCD_Send_row(1, 1, buffer); // with delay
			//
			//itoa(time_in_topoffcharge_mode,buffer,10);
			//uart_send_FLASH_string(string12);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string12);
			//LCD_Send_row(1, 1, buffer); // with delay
			//
			//itoa(time_in_trickle_mode,buffer,10);
			//uart_send_FLASH_string(string13);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string13);
			//LCD_Send_row(1, 1, buffer); // with delay
			//
			//uint16_t time_total = time_in_precharge_mode + time_in_fastcharge_mode + time_in_topoffcharge_mode + time_in_trickle_mode;
			//itoa(time_total,buffer,10);
			//uart_send_FLASH_string(string14);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string14);
			//LCD_Send_row(1, 1, buffer); // with delay
			//
			//itoa(mah,buffer,10);
			//uart_send_FLASH_string(string15);
			//uart_send_RAM_string(buffer);
			//LCD_Send_row(0, 0, string15);
			//LCD_Send_row(1, 1, buffer); // with delay
			
			// If only the 1st battery was charged yet
			if (battery_number == 1)
			{
				// switch to the 2nd battery
				battery_number = 2;
				mode = INIT_MODE; 
			}
		
			while(mode == END_MODE)
			{
				lcd_clrscr();
				lcd_home();
				LCD_Send_row(0, 1, string9); // with delay
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				sleep_enable();
				sleep_cpu();
			}
		}
		
	}
}


/************************************************************************/


/************************************************************************/
/*              Beginning Other Function Definitions                    */
/************************************************************************/


/************************************************************************/
/*                                  MENU                                */
/************************************************************************/
uint8_t Menu(void)
{
	uint8_t isMenuSelected = 0; // flag, indicates "jump out" from menu item
	uint8_t menuItem = 0; // MENU items
	uint8_t save_flag = 0; // flag, prevent entering again in IF cycles from UP/DOWN menus if saving data canceled
	uint8_t	selected_button = 0; //indicate which button selected
	
	if (menuItem == 0)
	{
		lcd_clrscr(); // Clear display
		// Initial message on LCD
		LCD_Send_row(0, 0, PSTR ("OK - CHARGE"));
		LCD_Send_row(1, 0, PSTR ("<> - SETUP"));
		
		while (menuItem == 0)
		{
			selected_button = Read_analog_buttons();
			
			switch (selected_button)
			{
				case 1:
				case 3:
				case 4:
				case 5:
					//SETUP selected
					// 1st menu item
					menuItem = 1;
					break;
				case 2:
					//Start charging
					return INIT_MODE;
					menuItem = 6; // jump out from the MENU
					break;
				default:
					break;
			}
		}
	}
	
	// 2. BATT CAPACITY
	if (menuItem == 1)
	{
		lcd_clrscr(); // Clear display
		LCD_Send_row(0, 0, PSTR ("BATT CAPACITY"));
		
		// Read value from EEPROM
		uint16_t ram_batt_capacity = eeprom_read_word(&eep_batt_capacity );
		
		// Show it on the display (2nd row)
		lcd_send_integer(1,&ram_batt_capacity);
		
		save_flag = 0; // reset
		isMenuSelected = 1; //reset
		
		while (isMenuSelected)
		{
			selected_button = Read_analog_buttons();
			
			switch (selected_button)
			{
				case 1:
				// <=
				Button_Left(&ram_batt_capacity);
				break;
				case 3:
				// DOWN
				Button_Up_Down( eeprom_read_word(&eep_batt_capacity ), ram_batt_capacity, &save_flag, &isMenuSelected, &menuItem, 2);
				break;
				case 4:
				// =>
				Button_Right(&ram_batt_capacity);
				break;
				case 5:
				// UP
				Button_Up_Down( eeprom_read_word(&eep_batt_capacity ), ram_batt_capacity, &save_flag, &isMenuSelected, &menuItem, 0);
				break;
				case 2:
				// SELECT
				// Save new value to EEPROM
				eeprom_write_word(&eep_batt_capacity, ram_batt_capacity);
				lcd_clrscr();
				
				// Show "SAVED" with delay
				LCD_Send_row(1, 1, string16);
				
				// Jump out from switch statement
				// to show the new, saved value
				isMenuSelected = 0;
				
				break;
				
				default:
				break;
			}
		}
	}
	
	// 3. Next item: CHARGING MODE
	if (menuItem == 2)
	{
		lcd_clrscr(); // Clear display
		LCD_Send_row(0, 0, PSTR ("CHARGING MODE"));
		
		// Read value from EEPROM
		uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
		
		// Show it on the display (2nd row)
		if (ram_charge_mode == EEP_FAST_CHARGE)
		{
			LCD_Send_row(1, 0, string6);
		}
		else if (ram_charge_mode == EEP_TRICKLE_CHARGE)
		{
			LCD_Send_row(1, 0, string8);
		}
		else // If there is no appropriate value in EEPROM
		{
			// Provide default value
			ram_charge_mode = EEP_FAST_CHARGE;
			LCD_Send_row(1, 0, string6);
		}
		
		save_flag = 0; // reset
		isMenuSelected = 1; //reset
		
		while (isMenuSelected)
		{
			selected_button = Read_analog_buttons();
			
			switch (selected_button)
			{
				case LEFT:
				// <=
				if (ram_charge_mode == EEP_FAST_CHARGE)
				{
					ram_charge_mode = EEP_TRICKLE_CHARGE;
					LCD_Send_row(1, 0, string8);
				}
				else if (ram_charge_mode == EEP_TRICKLE_CHARGE)
				{
					ram_charge_mode = EEP_FAST_CHARGE;
					LCD_Send_row(1, 0, string6);
				}
				break;
				case DOWN:
				// DOWN
				Button_Up_Down( eeprom_read_word(&eep_charge_mode), ram_charge_mode, &save_flag, &isMenuSelected, &menuItem, 3);
				break;
				case RIGHT:
				// =>
				if (ram_charge_mode == EEP_FAST_CHARGE)
				{
					ram_charge_mode = EEP_TRICKLE_CHARGE;
					LCD_Send_row(1, 0, string8);
				}
				else if (ram_charge_mode == EEP_TRICKLE_CHARGE)
				{
					ram_charge_mode = EEP_FAST_CHARGE;
					LCD_Send_row(1, 0, string6);
				}
				break;
				case UP:
				// UP
				Button_Up_Down( eeprom_read_word(&eep_charge_mode), ram_charge_mode, &save_flag, &isMenuSelected, &menuItem, 1);
				break;
				case SELECT:
				// SELECT
				// Save new value to EEPROM
				eeprom_write_word(&eep_charge_mode, ram_charge_mode);
				lcd_clrscr();
				
				// Show "SAVED" with delay
				LCD_Send_row(1, 1, string16);
				
				// Jump out from switch statement
				// to show the new, saved value
				isMenuSelected = 0;
				
				break;
				
				default:
				break;
			}
		}
	}
	
	
	// 4. Next item: TRICKLE TIME
	if (menuItem == 3)
	{
		lcd_clrscr(); // Clear display
		LCD_Send_row(0, 0, string13);
		
		// Read value from EEPROM
		uint16_t ram_trickle_charge_time = eeprom_read_word(&eep_trickle_charge_time );
		
		// Show it on the display (2nd row)
		lcd_send_integer(1,&ram_trickle_charge_time);
		
		save_flag = 0; // reset
		isMenuSelected = 1; //reset
		
		while (isMenuSelected)
		{
			selected_button = Read_analog_buttons();
			
			switch (selected_button)
			{
				case 1:
				// <=
				Button_Left(&ram_trickle_charge_time);
				break;
				case 3:
				// DOWN
				Button_Up_Down( eeprom_read_word(&eep_trickle_charge_time ), ram_trickle_charge_time, &save_flag, &isMenuSelected, &menuItem, 0);
				break;
				case 4:
				// =>
				Button_Right(&ram_trickle_charge_time);
				break;
				case 5:
				// UP
				Button_Up_Down( eeprom_read_word(&eep_trickle_charge_time ), ram_trickle_charge_time, &save_flag, &isMenuSelected, &menuItem, 2);
				break;
				case 2:
				// SELECT
				// Save new value to EEPROM
				eeprom_write_word(&eep_trickle_charge_time, ram_trickle_charge_time);
				lcd_clrscr();
				
				// Show "SAVED" with delay
				LCD_Send_row(1, 1, string16);
				
				// Jump out from switch statement
				// to show the new, saved value
				isMenuSelected = 0;
				break;
				
				default:
				break;
			}
		}
	}
	return MENU_MODE;
}


//initialize watchdog
void WDT_Init(void)
{
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	//set up WDT interrupt
	WDTCR |= (1<<WDCE)|(1<<WDE);
	//Start watchdog timer with 4s prescaller
	WDTCR |= (1<<WDP0)|(1<<WDP1)|(1<<WDP2);
	//Enable global interrupts
	sei();
}

static void WDT_Off(void)
{
	//disable interrupts
	cli();
	wdt_reset();
	WDTCR |= (1<<WDCE)|(1<<WDE);
	WDTCR = 0x00;
	//Enable global interrupts
	sei();
}


void Button_Up_Down(uint16_t eep_value, uint16_t ram_value, uint8_t *save_flag, uint8_t *isMenuSelected, uint8_t *menuItem, uint8_t next_menu)
{
	if ( (eep_value != ram_value) && *save_flag == 0)
	{
		lcd_clrscr();
		lcd_home();
		// SAVE?
		LCD_Send_row(0, 0, string17);
		LCD_Send_row(1, 0, string18);
		*save_flag = 1; // prevent entering again in this IF cycle
	}
	else
	{
		*isMenuSelected = 0;
		*menuItem = next_menu; // next item
	}
}

void Button_Left(uint16_t *value)
{
	//Decrease
	*value = *value - 1;
		
	// Show new value
	lcd_send_integer(1,value);
}

void Button_Right(uint16_t *value)
{
	// Increase
	*value = *value + 1;
	
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


uint8_t Charge_Battery(uint8_t this_mode, uint8_t battery_number, uint16_t max_current, uint16_t* time_in_this_mode, uint16_t *mah, const char *string_for_display)
{
	uint8_t	mode = this_mode;
	uint8_t	previous_sec =	0;
	uint8_t	previous_tick =	0;
	uint8_t pwm = 0;
	uint16_t temperature = 0;
	
	// initializations
	ramp = RAMP_UP_TIME;
	voltage_in_ADC = 0;
	current_in_ADC = 0;
	neg_db = NDV_SAMPLES;
	fdv_db = FDV_SAMPLES;
	if(mode == TRICKLE_MODE)
	{
		// Enable both batteries
		Battery_Enable(1, 1); // if not enabled yet
		Battery_Enable(2, 1); // if not enabled yet
	}
	else
	{
		// Enable only the selected
		Battery_Enable(battery_number, 1);
	}	
	Disable_PWM();
	Init_PWM(0);
	uart_send_FLASH_string(string_for_display);
	lcd_clrscr();
	lcd_home();
	lcd_puts_p(string_for_display);
	
	while(mode == this_mode){
		if ( previous_tick != timer1_tick){
			WDT_Init();
			uint16_t voltage = Measure_U();
			uint16_t current = Measure_I();
			if (previous_sec != seconds){// Every secs
				//1. Debug
				UART_send_data_block(11, mode, minutes, seconds, voltage, current, temperature, pwm, peak_db, peak_voltage, fdv_db, neg_db); // Debug
				LCD_Send_Debug(voltage, current, temperature);
				//LCD_Send_Data(voltage, current, temperature);
				//2. State machine
				mode = Check_battery_status(battery_number, voltage, current, &temperature, this_mode); //measure voltage and decide what to do next.
				//Check_timer(battery_number, current);
				//3.every 10sec increase/decrease PWM by 1 (ramp-up)
				Modify_PWM_Width(current, max_current, &pwm);
				//4. saving total time for EXIT mode
				if (seconds == 0){ // Every minutes
					(*time_in_this_mode)++;
				}
				//5. check if mode was changed
				if (this_mode != mode)
				{
					// Calculate absorbed mAh's
					(*mah) += ((uint32_t)current * minutes ) / 60;
					// If only 1 battery charged yet, skip other modes and switch to the next battery
					if (this_mode == FASTCHARGE_MODE && battery_number == 1)
					{
						mode = END_MODE;
					}
				}
				previous_sec = seconds;
			}
			previous_tick = timer1_tick;
			WDT_Off();
		}
	} 
	return mode;
}


uint8_t Discharge_Battery(void)
{
	uint16_t	voltages[2] = {0, 0}; // Discharging voltages per battery
	uint8_t		previous_sec = 0;
	uint8_t mode = DISCHARGE_MODE;
	
	// Measure voltages on both battery, one by one
	Battery_Enable(1,0);
	Battery_Enable(2,0);
	
	Battery_Enable(1, 1);
	voltages[0] = Measure_U();
	Battery_Enable(1,0);
	
	Battery_Enable(2, 1);
	voltages[1] = Measure_U();
	Battery_Enable(1, 1);
	// Now both batteries are turned on
	
	Discharge_Enable(1);	// Discharge ON
	
	uart_send_FLASH_string(string0);
	LCD_Send_row(0, 0, string0);
	
	while(mode == DISCHARGE_MODE)
	{
		if (previous_sec != seconds)// Every second
		{
			// 1. Check 1st battery
			if (voltages[0] < 1000) //discharge to 1000mV
			{
				// Stop Discharging this battery
				Battery_Enable(1,0); // disable the 1st battery
			}
			else
			{
				uint8_t enabled = 0;
				// disable the 2nd battery if needed to measure voltage only on the 1st
				if (bit_is_set(PINB,PB2)) //if it's enabled
				{
					Battery_Enable(2,0); // Disable
					enabled = 1; // set flag
				}
				_delay_ms(100); // wait to stabilize voltage
				voltage_in_ADC = 0; // Reset oversampling
				voltages[0] = Measure_U(); // measure
				if (enabled) Battery_Enable(2, 1); // switch on if it was enabled previously
			}
			
			// 2. Check 2nd battery
			if (voltages[1] < 1000)//discharge to 1000mV
			{
				// Stop Discharging this battery
				Battery_Enable(2,0); // disable the 2nd battery
			}
			else
			{
				uint8_t enabled = 0;
				// disable the 1st battery to measure voltage only on the 2nd
				if (bit_is_set(PINB,PB1)) //if it's enabled
				{
					Battery_Enable(1,0); // Disable
					enabled = 1; // set flag
				}
				
				_delay_ms(100); // wait to stabilize voltage
				voltage_in_ADC = 0; // Reset oversampling
				voltages[1] = Measure_U(); // measure
				if (enabled) Battery_Enable(1, 1); // switch on if it was enabled previously
			}
			
			UART_send_data_block(5, mode, minutes, seconds, voltages[0], voltages[1]); // Debug
			//LCD_Send_Data();
			
			// 3. Check if we need to stop "Discharge"
			if (( voltages[0] < 1000) && (voltages[1] < 1000))
			{
				// Discharge finished
				Discharge_Enable(0);	// Discharge OFF
				Battery_Enable(1,0);
				Battery_Enable(2,0);
				// Reset variables
				seconds = 0;
				minutes = 0;
				isDischarged = 1; // Discharge Flag (boolean). Prevents entering again in "Discharge mode"
				// Exit from Discharge mode
				return INIT_MODE;
			}
			previous_sec = seconds;
		}	
	}
	return DISCHARGE_MODE;
}

// based on voltage-level decide the next step...
uint8_t Battery_Detection(uint8_t *batt_num)
{
	uart_send_FLASH_string(string2); // Batt. detection
	LCD_Send_row(0,1,string2); // with delay
	// Disable batteries
	Battery_Enable(1,0);
	Battery_Enable(2,0);
	Discharge_Enable(0);
	Disable_PWM();
		
	Battery_Enable(*batt_num, 1);
	temp_ok = init_TMP75(*batt_num); // initialize I2C temperature measuring device
	
	uint8_t batt_detect_timeout = BATT_DETECT_TIMEOUT;
	uint8_t previous_sec = seconds;
	
	while(batt_detect_timeout)
	{
		if (previous_sec != seconds)// Every second
		{
			voltage_in_ADC = 0; // disable oversampling;
			uint16_t voltage = Measure_U();
			uint16_t temperature = Read_temperature(temp_ok, *batt_num);
				
			UART_send_data_block(5, *batt_num, minutes, seconds, voltage, temperature); // Debug
				
			if ((voltage > PRECHARGE_TERMINATION_VOLTAGE) && (voltage < FAST_CHARGE_END_MIN_VOLTAGE+100) && (isDischarged == 0))
			{
				// Battery found and not discharged yet
				return DISCHARGE_MODE;
			}
			else if (voltage > BATTERY_DETECTION_VOLTAGE && (voltage < FAST_CHARGE_END_MIN_VOLTAGE+100))
			{
				// Discharge finished or battery doesn't need to be discharged
				// Switching to charging
				Discharge_Enable(0);	// OFF
				uart_send_FLASH_string(string4);
				LCD_Send_row(1, 0, string4);
				// before going to another mode reset variables
				seconds = 0;
				minutes = 0;
				
				
				uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode); // Selected charging method (FAST / TRICKLE)
				// If Fast Charge selected in "Menu"
				if ( ram_charge_mode == EEP_FAST_CHARGE)
				{
					return PRECHARGE_MODE;
				}
				// If Trickle Charge selected in "Menu"
				else if ( ram_charge_mode == EEP_TRICKLE_CHARGE)
				{
					return TRICKLE_MODE;
				}
				else
				{
					// Terminate
					*batt_num = 2;
					return END_MODE;
				}
			}
			else
			{
				// No battery detected...
				// wait 1 minute for battery detection
				uart_send_FLASH_string(string3);
				LCD_Send_row(1, 0, string3);
				batt_detect_timeout--;
				//return INIT_MODE;
			}
			previous_sec = seconds;
		}
	}
	// If no battery detected within "batt_detect_timeout"
	return END_MODE;
}

	
	


uint8_t init_TMP75 (uint8_t select)
{
	uint8_t temp_ok = 0;
	switch (select)
	{
		case 1:
		i2c_start(DEVICE_ADDRESS1+I2C_WRITE) ? (temp_ok = 0) : (temp_ok = 1);
		break;
		case 2:
		i2c_start(DEVICE_ADDRESS2+I2C_WRITE) ? (temp_ok = 0) : (temp_ok = 1);
		break;
	}
	
	//i2c_start(address+I2C_WRITE);
	
	if (!temp_ok)
	{
		//if no device found do not continue
		uart_puts_P("Temp NOK");
		uart_send_CR_LF();
		return temp_ok;
		
	} 
	else
	{
		uart_puts_P("Temp OK");
		uart_send_CR_LF();
	}
	
	i2c_write(0x01);
	
	i2c_write(0xB0);
	
	i2c_stop();
	
	return temp_ok;
}

/***************************************/
uint16_t Read_temperature(uint8_t temp_ok, uint8_t select){
	
	init_TMP75(select);
	if (!temp_ok) return 0; //if no device found do not continue
		
	uint8_t temperatures[2];
	uint16_t digit = 0;
	uint16_t decimal = 0;
	uint8_t address = 0;
	
	switch (select)
	{
		case 1:
		address = DEVICE_ADDRESS1;
		break;
		case 2:
		address = DEVICE_ADDRESS2;
		break;
	}
	if( i2c_start(address+I2C_WRITE) ) return 0; // 1= failed to access device
	
	if( i2c_write(0x00) ) return 0; // 1= write failed
	
	if( i2c_start(address+I2C_READ) ) return 0; // 1= failed to access device
	
	temperatures[0] = i2c_readAck();
	
	temperatures[1] = i2c_readAck();
	
	i2c_stop();
	
	digit = (uint16_t)temperatures[0];
	decimal = (uint16_t)((temperatures[1] >> 4)*625)/1000;
	
	digit = digit*10 + decimal; // 25.2 => 25*10 + 2 => 250 + 2 => 252
	
	return (digit);
}



uint8_t Init_LCD (void)
{
	lcd_init(LCD_DISP_ON);
	
	uint8_t lcd_ok = lcd_test(); // test if LCD connected or not
	
	if(lcd_ok) //if LCD found
	{
		uart_puts_P("LCD OK");
		uart_send_CR_LF();
		
		lcd_command(0x28);      /* function set: display lines  */
		lcd_command(0x08);      /* display off                  */
		lcd_clrscr();                           /* display clear                */ 
		lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
		lcd_command(LCD_DISP_ON);                  /* display/cursor control       */
		lcd_home();
	}
	else
	{
		uart_puts_P("LCD NOK");
		uart_send_CR_LF();
	}
	
	return lcd_ok;
}

/***************************************/
void Init_PWM(uint8_t pwm){
	//max 62500 Hz @ 16MHz
	// Counter2 used here (8-bit)
	DDRB |= 1 << PINB3; //PB3 as output
	TCCR2 |= 1 << COM21; // non-inverting mode
	TCCR2 |= (1 << WGM21) | (1 << WGM20); //Fast PWM 8-bit
	TCCR2 |= 1 << CS20; // start with prescaler = 0
	OCR2 = pwm;
}

/***************************************/
void Disable_PWM(void){
	TCCR2 = 0x00; // disable timer
	DDRB |= (1 << PINB3); // output
	PORTB &= ~(1<<PINB3); // pull-down
	OCR2 = 0;
}

/**************************************
void Set_PWM(uint8_t pwm_to_set)
{
	unsigned char sreg;
	
	sreg = SREG;
	cli();
	OCR2 = pwm_to_set;
	SREG = sreg;
	sei();
}*/

/***************************************/
void Modify_PWM_Width(uint16_t current, uint16_t max_current, uint8_t *pwm)
{
	
	uint8_t previous_pwm = *pwm;
		
	if (ramp && (seconds%10 == 0))
	{
		if (((current >= max_current) || (*pwm >= PWM_MAX)) && (*pwm > 0))
		{
			(*pwm)--;
		} 
		else if ((current <= max_current) && (*pwm < PWM_MAX))
		{
			(*pwm)++;
		}
		
		
		// if pwm changed, write new value
		if (previous_pwm != *pwm)
		{
			OCR2 = *pwm;
			//Set_PWM(pwm);
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
	
	// External reference
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

/* ADC single measurement */
uint16_t ADC_Read_channel (uint8_t channel){
	
	// Choose channel without affect other bits
	ADMUX = (ADMUX & ~ (0x1F)) | (channel & 0x1F);
	_delay_us(10); // Delay to charge ADC input
	ADCSRA |= (1 << ADSC); // a "single conversion"
	while (ADCSRA & (1 << ADSC)) {//wait  on completion of the conversion
	}
	return ADCW; //read and return ADC value
}

/* ADC Multiple measurements with oversampling */
uint16_t ADC_Read_Avg (uint8_t channel, uint8_t nsamples){
	uint32_t sum = 0;
	
	// Choose channel without affect other bits
	ADMUX = (ADMUX & ~ (0x1F)) | (channel & 0x1F);
	
	for (uint8_t i = 0; i <nsamples; ++ i) {
		sum += ADC_Read ();
	}
	
	return sum;
}


uint8_t Read_analog_buttons(void){
	
	uint8_t button = 0;
	uint8_t temp_button = 0;
	//uint8_t button_array[5];
	uint8_t flag = 0;
	
	for (uint8_t i = 0; i< 5; i++)// 5 consecutive reading
	{
		_delay_ms(50); // 50ms * 5 = 250ms
		uint16_t adc = ADC_Read_channel(2);  //read analog value
		
		if (adc < 1023)
		{
			uart_send_integer(adc);
		}
		
		temp_button = button; // save old value before overwriting
		
		// Find which button was pressed
		for(uint8_t j = 0; j < 5; j++)
		{
			if( abs(pgm_read_word (&buttondata[j]) - adc ) < 40 ) // Allowed error is +-40
				button = j+1;
		}
		 // Compare to previous value
		 // if not the same => quit
		if ( i > 0 && button > 0)
		{
			if (temp_button == button )
			{
				flag = 1;
			}
			else
			{
				return 0;
				//flag = 0;
			}
		}
		
	}
	// We have now 5 equal readings
		
	if (flag)
	{
		uart_send_integer(button);
		return button;
	}
	
	return 0;
}

/***************************************/
uint16_t Measure_U(void){
	uint16_t ADC0_value_unfilt = 0;
	uint16_t voltage0 = 0; // Voltage on ADC0
	
	ADC0_value_unfilt = ADC_Read_Avg(0, ADC_SAMPLES);
	
	if(!(voltage_in_ADC)){
		voltage_in_ADC = ADC0_value_unfilt;
		} else {
		if(voltage_in_ADC > ADC0_value_unfilt) {
			voltage_in_ADC -= (voltage_in_ADC - ADC0_value_unfilt) >> FILTER_BITS;
			} else {
			voltage_in_ADC += (ADC0_value_unfilt - voltage_in_ADC) >> FILTER_BITS;
		}
	}
	
	voltage0 = (((uint32_t) voltage_in_ADC * ADC_REF) / ADC_OVERSAMPLE_MAX );
	//voltage0 = ((uint32_t)(R1 + R2) * voltage0) / R2 ; // calculate voltage1 before resistor divider
	//voltage = voltage0; // for Monitoring
	return voltage0;
}


/***************************************/
uint16_t Measure_I(void){
	uint16_t ADC1_value_unfilt = 0;
	uint16_t voltage1 = 0; // Voltage on ADC1
	
	ADC1_value_unfilt = ADC_Read_Avg(1, ADC_SAMPLES); // Voltage on ADC1 channel in ADC values
	
	//ADC1_value_unfilt = ADC1_value_unfilt/64; // 1023 max
	
	
	//voltage1 = ((uint32_t)ADC1_value_unfilt*ADC_REF)/1023; // 2477 max, voltage on ADC input pin
	//voltage1 = voltage1*5; // Voltage before resistor divider
	//voltage1 = voltage1*100; // convert to mV
	//voltage1 = voltage1/9; // Gain of Op-amp
	//voltage1 = voltage1/RSHUNT;	// calculate Amp from voltage drop
	
	//voltage1 = ((uint32_t)voltage1*1000/GAIN)/RSHUNT; // 3216 max
	//voltage1 = ((uint32_t)(R1 + R2) * voltage1) / ((uint32_t)R2*GAIN) ; // calculate voltage1 before resistor divider
	//voltage1 = voltage1/RSHUNT; // 45036 max, if not divided by 14
	//current = voltage1;
	
	// Filtering...
	if(!current_in_ADC){ // used first time
		current_in_ADC = ADC1_value_unfilt;
		} else {
		if(current_in_ADC > ADC1_value_unfilt){
			current_in_ADC -= (current_in_ADC - ADC1_value_unfilt) >> FILTER_BITS;
			} else {
			current_in_ADC += (ADC1_value_unfilt - current_in_ADC) >> FILTER_BITS;
		}
	}
	
	voltage1 = (((uint32_t)current_in_ADC * ADC_REF) / ADC_OVERSAMPLE_MAX );	// Voltage on ADC-input pin
	/*
	voltage1 = voltage1*5; // Voltage before resistor divider
	voltage1 = voltage1*100; // convert to mV
	voltage1 = voltage1/9; // Gain of Op-amp
	voltage1 = voltage1/RSHUNT;	// calculate Amp from voltage drop
	//current = voltage1;
	*/
	voltage1 = ((uint32_t)voltage1 * 500) / (9*RSHUNT);
	return voltage1;
	
	////voltage1 *= 1000; // shunt is 0.055R, a float number, better to use mV, mA and mOhm in formulas
	////voltage1 /= 14; // 18 is Gain of Op-Amp => calculating voltage-drop on Shunt-Resistor, before the OpAmp
	////current = voltage1/RSHUNT; // I = U / R
	//////
	//voltage1 = ((uint32_t)voltage1*1000/GAIN)/RSHUNT; // 3216 max
	
}

/***************************************/
void Init_timer(void){
	// Using Timer1 to count seconds, minutes, hours.
	// Interrupt driven
	TCCR1B |= (1 << WGM12); // CTC mode	(Hardware controlled)
	TIMSK |= (1 << OCIE1A); // Enable CTC interrupt
	//OCR1A = 15624; // interrupt on every 1 sec.	16.000.000 / 1024 = 15625. And -1, because starting from 0.
	OCR1A = 3124; // interrupt on every 1/5 = 0.2 sec.	5 * 3125 = 15625. And -1, because starting from 0.
	TCCR1B |= ((1 << CS12) | (1 << CS10)); // start with 1024 pre-scaler
}

/*******/
void Battery_Enable(uint8_t batt_select, uint8_t on_off)
{
	if(on_off)
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
	else
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
	}
}


void Terminate(uint8_t *mode, uint8_t *batt_num, const char *str)
{
	uart_send_FLASH_string(str);
	LCD_Send_row(0,1,str);
	// EXIT
	*batt_num = 2; 
	*mode = END_MODE;
}


void uart_send_integer (int i)
{
	itoa(i,buffer,10);
	uart_puts(buffer);
	uart_send_CR_LF();
}


/***************************************/
void uart_send_RAM_string(const char *str){
	//
	uart_puts(str);
	uart_send_CR_LF();
}

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
		uart_puts("; ");
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
		lcd_gotoxy(0,row_number);
		for(uint8_t i = 0; i< 16; i++)
		{
			lcd_putc(' ');
		}
		//2. show string on LCD from program memory
		lcd_gotoxy(0,row_number);
		lcd_puts_p(str);
		if (isDelayEnabled)
		{
			_delay_ms(1000);
		}
		
	//}
}


/***************************************/
void LCD_Send_Debug(uint16_t volts, uint16_t current, uint16_t temperature)
{
	//if(!lcd_status) return; //if LCD not found do not continue
	
	uint16_t lcd_temp, lcd_volt, lcd_cur;
	char c[7]; // buffer for the digits
	
	lcd_gotoxy(4,1); //second row, 4th position
	lcd_puts("V");
	
	lcd_volt = volts;
	int x = 3; // starting from 3-rd position (last digit)
	while(lcd_volt>0)
	{
		lcd_volt = lcd_volt/10; // drop last number
		uint8_t number = (uint8_t)lcd_volt%10; // calculate digit
		itoa(number,c,10); // convert to string
		if(x==1) // 2nd position is a "," symbol
		{
			lcd_gotoxy(x,1);
			lcd_puts(",");
			x--;
		}
		lcd_gotoxy(x,1); // put digit in this place
		lcd_puts(c);
		x--; //decreasing position
	}
	
	
	lcd_gotoxy(10,1); //second row, 4th position
	lcd_puts("A");
	
	lcd_cur = current;
	x = 9; // starting from 3-rd position (last digit)
	while((lcd_cur>0) && (x>5))
	{
		lcd_cur = lcd_cur/10; // drop last number
		uint8_t number = (uint8_t)lcd_cur%10; // calculate digit
		itoa(number,c,10); // convert to string
		if(x==7) // 7th position is a "," symbol
		{
			lcd_gotoxy(x,1);
			lcd_puts(",");
			x--;
		}
		lcd_gotoxy(x,1); // put digit in this place
		lcd_puts(c);
		x--; //decreasing position
	}
	
	lcd_gotoxy(14,1); //second row, 4th position
	lcd_puts("C");
	
	lcd_temp = temperature;
	x = 13; // starting from 3-rd position (last digit)
	while((lcd_temp>0) && (x>11))
	{
		lcd_temp = lcd_temp/10; // drop last number
		uint8_t number = (uint8_t)lcd_temp%10; // calculate digit
		itoa(number,c,10); // convert to string
		
		lcd_gotoxy(x,1); // put digit in this place
		lcd_puts(c);
		x--; //decreasing position
	}
	
}


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



/***************************************/
uint8_t Check_battery_status(uint8_t battery_number, uint16_t voltage, uint16_t current, uint16_t *temperature, uint8_t this_mode)
{
	// Called every seconds
	//uint8_t old_mode = this_mode;
	uint16_t old_temperature = *temperature;

	// Temperature check START
	if (seconds == 0){ // Every minute
		// Measure current temperature to compare previous values
		
		*temperature = Read_temperature(temp_ok, battery_number);
		if ( *temperature ) // if read successful
		{
			// Continue only after NDV_PREVENTIVE_DELAY
			// Active in all modes, except in TRICKLE
			if (minutes >=  NDV_PREVENTIVE_DELAY && this_mode != TRICKLE_MODE)
			{
				// If current temp 1 Celsius bigger now than in the previous minute
				// or 15 Celsius bigger than the starting temperature
				// or bigger then 40 Celsius
				if ( (*temperature > old_temperature) && ( ((*temperature - old_temperature) > 10) || (*temperature > 400) )){
					// 315 - 305 = 10. current=>31.5 C | previous=>30.5 C | delta => 1 C
					// 350 - 200 = 150,
					uart_puts_P("Temperature termination!");
					uart_send_integer(old_temperature);
					uart_send_integer(*temperature);
					Disable_PWM();
					Battery_Enable(battery_number,0);
					
					uint8_t battery_delay = BATTERY_COOLDOWN_TIMER; // 10 minutes to cool down
					uint16_t old_minutes = minutes;
					
					while (battery_delay)
					{
						if (old_minutes != minutes){// Every minute, changed by interrupt
							*temperature = Read_temperature(temp_ok, battery_number);
							uart_send_integer(*temperature);
							battery_delay--; // until 0 => if 0 exit from loop
							old_minutes = minutes; // saving new "minutes" value
						}
					}
					return TOPOFFCHARGE_MODE; // trickle charge after temperature termination
				}
			}
		}
	}
	//Temperature check END
	
	
	



	// Voltage check START
	switch (this_mode)
	{
		case PRECHARGE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE)
			{ // no battery found
				//Debug("Voltage <200, END_MODE");
				return END_MODE; // exit
			}
			
			if (voltage >= PRECHARGE_TERMINATION_VOLTAGE)
			{
				return  FASTCHARGE_MODE;
			}
			
			if (minutes > PRECHARGE_END_TIMER)
			{
				uart_send_FLASH_string(string20);
				LCD_Send_row(0,1,string20);
				return END_MODE;
			}
			break;
			
		case FASTCHARGE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE)
			{ // no battery found
				//Debug("Voltage <200, END_MODE");
				return END_MODE; // exit
			}
			
			if (voltage >= FAST_CHARGE_END_MAX_VOLTAGE)
			{
				return  TRICKLE_MODE;
			}
			
			if (minutes > FASTCHARGE_END_TIMER)
			{
				uart_send_FLASH_string(string21);
				LCD_Send_row(0,1,string21);
				return TRICKLE_MODE;
			}
			break;
		
		case TOPOFFCHARGE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE)
			{ // no battery found
				//Debug("Voltage <200, END_MODE");
				return END_MODE; // exit
			}
			
			if (minutes > TOPOFFCHARGE_END_TIMER)
			{
				uart_send_FLASH_string(string22);
				LCD_Send_row(0,1,string22);
				return TRICKLE_MODE;
			}
			break;
		
		case TRICKLE_MODE:
			if (voltage < BATTERY_DETECTION_VOLTAGE)
			{ // no battery found
				//Debug("Voltage <200, END_MODE");
				return END_MODE; // exit
			}
			
			uint16_t ram_charge_mode = eeprom_read_word(&eep_charge_mode);
			uint16_t ram_trickle_charge_time = eeprom_read_word(&eep_trickle_charge_time);
			if ( ram_charge_mode == EEP_TRICKLE_CHARGE && minutes > ram_trickle_charge_time)
			{
				uart_send_FLASH_string(string19);
				LCD_Send_row(0,1,string19);
				return END_MODE;
				
			} 
			else if (minutes > MAINTENANCE_END_TIMER)
			{
				uart_send_FLASH_string(string19);
				LCD_Send_row(0,1,string19);
				return END_MODE;
			}
			
			break;
		
		default:
			break;
	}
	
	/*
	else if (voltage >= BATTERY_DETECTION_VOLTAGE && voltage < PRECHARGE_TERMINATION_VOLTAGE )
	{
		//Debug("Voltage between 500 and 1000, PRECHARGE_MODE");
		return  PRECHARGE_MODE; // pre-charge
	}
	*/
	//else if (voltage >= PRECHARGE_TERMINATION_VOLTAGE && voltage <= FAST_CHARGE_END_MAX_VOLTAGE && mode != MAINTENANCE_MODE && mode != TOPOFFCHARGE_MODE)
	//{
		//Debug("Voltage between 1000 and 1700, FASTCHARGE_MODE");
		//return  FASTCHARGE_MODE; // fast charge
	//}
	//else if (voltage >= FAST_CHARGE_END_MAX_VOLTAGE && mode == FASTCHARGE_MODE)
	//{ //end of Fast-charging, changing to Maintenance-mode
		//Debug("Voltage >1700, MAINTENANCE_MODE");
		//return  MAINTENANCE_MODE; // trickle charge
	//}
	// Voltage check END
	
	
	
	// NDV and FDV detection START
	// Wait 15 minutes in FASTCHARGE_MODE to stabilize the battery (NDV_PREVENTIVE_DELAY)
	// voltage >= 1475 mV, close to the end of charge
	if ((this_mode == FASTCHARGE_MODE) && (minutes >=  NDV_PREVENTIVE_DELAY) && (voltage >= FAST_CHARGE_END_MIN_VOLTAGE))
	{		

		// Update voltage peak if necessary
		if(voltage > peak_voltage)
		{ // voltage rising
		// peak_db => Number in a row charging voltage higher that the last peak voltage needed to update
			if(peak_db)
			{ // if > 0
				peak_db--; // decrease by 1
				//Debug("Voltage is flat!");
				//itoa(peak_db,buffer,10);
				//uart_puts("peak_db: ");
				//uart_puts(buffer);
				//uart_puts("\r");
				//uart_puts("\n");
			} 
			else 
			{ // if = 0
				//Debug("Voltage rising!");
				peak_voltage += (voltage - peak_voltage) / 2 + 1;
				//itoa(peak_voltage,buffer,10);
				//uart_puts("peak_voltage: ");
				//uart_puts(buffer);
				//uart_puts("\r");
				//uart_puts("\n");
				// restore initial values (reset)
				peak_db = PEAK_UPDATE_SAMPLES;
				fdv_db = FDV_SAMPLES;
			}
			
		} 
		else 
		{	// when voltage <= peak_voltage, voltage is decreasing or constant
			peak_db = PEAK_UPDATE_SAMPLES;
		}

		//check for flat delta V condition
		if(fdv_db)
		{
			fdv_db--;
			//Debug("Voltage is flat!");
			//itoa(dv_zero_db,buffer,10);
			//uart_puts("dv_zero_db: ");
			//uart_puts(buffer);
			//uart_puts("\r");
			//uart_puts("\n");
		} 
		else
		{
			uart_puts_P("FDV detected!");
			uart_send_CR_LF();
			return TRICKLE_MODE;
		}
		
		//check for negative delta v condition
		if(voltage <  (peak_voltage - NDV_DROP))
		{
			if(neg_db)
			{
				neg_db--;
				//Debug("Voltage is negative!");
				//itoa(dv_neg_db,buffer,10);
				//uart_puts("dv_neg_db: ");
				//uart_puts(buffer);
				//uart_puts("\r");
				//uart_puts("\n");
			} 
			else
			{
				uart_puts_P("NDV detected!");
				uart_send_CR_LF();
				return TRICKLE_MODE;
			}
			
		} 
		else 
		{
			neg_db = NDV_SAMPLES;
		}
	}
	// NDV and FDV detection END
	
	return this_mode;
}

/**************************************
uint8_t Check_timer(uint8_t battery_number, uint16_t current, uint8_t this_mode){
		// If in ... minutes ...charge was not successful => change mode
	switch(this_mode)
	{
		case PRECHARGE_MODE:
			if (minutes > PRECHARGE_END_TIMER))
			{
				uart_puts_p("PRECHARGE_END_TIMER");
				uart_send_CR_LF();
				return END_MODE;
			}
			break;
			
		case FASTCHARGE_MODE:
			if (minutes > FASTCHARGE_END_TIMER))
			{
				uart_puts_p("FASTCHARGE_END_TIMER!");
				uart_send_CR_LF();
				return MAINTENANCE_MODE;
			}
			break;
		
		case TOPOFFCHARGE_MODE:
			if (minutes > TOPOFFCHARGE_END_TIMER))
			{
				uart_puts_p("TOPOFFCHARGE_END_TIMER!");
				uart_send_CR_LF();
				return MAINTENANCE_MODE;
			}
			break;
		
		case MAINTENANCE_MODE:
			if (minutes > TOPOFFCHARGE_END_TIMER))
			{
				uart_puts_p("MAINTENANCE_END_TIMER!");
				uart_send_CR_LF();
				return END_MODE;
			}
			break;
			
		default:
			break;
	}
	
	
	if (mode == PRECHARGE_MODE && minutes > PRECHARGE_END_TIMER)
	{
		// If in 10 minutes precharge was not successful => EXIT
		uart_send_string("PRECHARGE_END_TIMER!");
		mode = END_MODE;
		
	} 
	if (mode == FASTCHARGE_MODE && minutes > FASTCHARGE_END_TIMER)
	{
		//
		uart_send_string("FASTCHARGE_END_TIMER!");
		mode = MAINTENANCE_MODE;
		
	} 
	if (mode == TOPOFFCHARGE_MODE && minutes > TOPOFFCHARGE_END_TIMER)
	{
		//
		uart_send_string("TOPOFFCHARGE_END_TIMER!");
		mode = MAINTENANCE_MODE;
		
	}
	if (mode == MAINTENANCE_MODE && minutes > MAINTENANCE_END_TIMER)
	{
		//
		uart_send_string("MAINTENANCE_END_TIMER!");
		mode = END_MODE;
	}
	
}
*/		

// Timer interrupt	
ISR(TIMER1_COMPA_vect){
	//configured to interrupt every 1/5 seconds
	
	timer1_tick++;
	if (timer1_tick == 5){	
		timer1_tick = 0; // reset
		seconds++;	
	
		if (seconds == 60){
			   seconds = 0; // reset
			   minutes++;
		}
	}		
}

