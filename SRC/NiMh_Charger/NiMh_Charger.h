#ifndef NIMH_CHARGER_H_
#define NIMH_CHARGER_H_


/************************************************************************/
/*                               DEFINES                                */
/************************************************************************/
#define F_CPU								16000000UL // CPU frequency
#define UART_BAUD_RATE						9600 // UART baud-rate

// TMP75 Temperature sensing devices addresses
// Separate device for both battery
#define DEVICE_ADDRESS1						0x90      // I2C 1st device address
#define DEVICE_ADDRESS2						0x9E      // I2C 2nd device address

#define RSHUNT								55 //Shunt-resistance in milli-Ohms

#define NDV_DROP							5 // mV

#define BATTERY_DETECTION_VOLTAGE			500 // mV

#define PRECHARGE_TERMINATION_VOLTAGE		1000 //mV

#define FAST_CHARGE_END_MIN_VOLTAGE			1400// was 1475 , mV

#define FAST_CHARGE_END_MAX_VOLTAGE			2400 // mV

#define NDV_SAMPLES							10 // number of samples needed for NDV
#define FDV_SAMPLES							(uint16_t)15*60	// 10 minutes required for flat delta V condition 
#define PEAK_UPDATE_SAMPLES					5 // Number in a row charging-voltage higher that the last peak voltage needed to update

#define NDV_PREVENTIVE_DELAY				15 //minutes, used at the beginning of FASTCHARGE
#define PRECHARGE_END_TIMER					10 // minutes, PRECHARGE termination timer
#define FASTCHARGE_END_TIMER				210 // in minutes, FASTCHARGE termination timer (3.5 hour)
#define RAMP_UP_TIME						30 // * 10 sec = 300 sec (5 minutes), PWM ramp up time
#define BATT_DETECT_TIMEOUT					60 // seconds, time for detecting a battery

//		*** Voltage reading filter definitions ***
#define ADC_REF								2500 // mV, TL431 precise shunt
#define ADC_SAMPLES							64  // samples for oversampling
#define	ADC_OVERSAMPLE_MAX					65535	// ADC max value for oversampling 64*1024
#define	FILTER_BITS							6 //used for oversampling

// Charger modes used in state machine
#define MENU_MODE							0
#define INIT_MODE							1
#define DISCHARGE_MODE						2
#define PRECHARGE_MODE						3
#define FASTCHARGE_MODE						4
#define TRICKLE_MODE						5

// Buttons
#define LEFT								1
#define SELECT								2
#define DOWN								3
#define RIGHT								4
#define UP									5

// PWM defines
// the maximum value of PWM is 255 (8 bit, 2^8 = 256, and -1 because starting from 0)
#define PWM_MAX								255/2 // max 50% duty cycle for Buck-converter



#define DISCHARGE							PB0
#define BATTERY_1 							PB1
#define BATTERY_2							PB2
#define ON									1
#define OFF									0
#define BATTERY_MAX 						2
#define MENU_MAX							2
#define BUTTON_ADC_CHANNEL					PC2



 /************************************************************************/
 /*                           FUNCTION PROTOTYPES                        */
 /************************************************************************/
 
  /**
 @brief    Initialize Timer1 to count seconds & minutes
 
 @param    none
 @return   none
*/
static void Init_timer(void);
 
/**
 @brief    Initialize TMP75 Temperature sensing device to 12bit measuring mode. This routine also checks whether devices are connected or no. If not sets "isTempOk" global flag to 0 to prevent using unconnected devices.
 
 @param    select: 1 (1st device), 2 (2nd device)
 @return   0 (not connected), 1 (connected)
*/
static uint8_t init_TMP75 (uint8_t select);
/**
 @brief    Read temperature from selected device. Active only if "isTempOk" global flag equals 1.
 
 @param    select: 1 (1st device), 2 (2nd device)
 @return   temparature in the following format: 30.8 Celsius = 308 (to prevent using float numbers)
*/
static uint16_t Read_temperature(uint8_t select);
/**
 @brief    Initialize Timer2 to PWM mode
 
 @param    pwm: OCR2 value
 @return   none
*/
static void Init_PWM(uint8_t pwm);
/**
 @brief    Disable Timer2 and PWM
 
 @param    none
 @return   none
*/
static void Disable_PWM(void);
/**
 @brief    Increase / decrease pwm value by 1 to achieve the desired current level (ramp up)
 
 @param    current: actual current level measured on the selected battery
 @param    max_current: current level to be achieved
 @param    pwm: actual pwm value
 @return   none
*/
static void Modify_PWM_Width(uint16_t current, uint16_t max_current, uint8_t *pwm);
/**
 @brief    Initialize ADC
 
 @param    none
 @return   none
*/
static void ADC_Init(void);
/**
 @brief    Single ADC measurement
 
 @param    none
 @return   ADC value: 0-1023
*/
static uint16_t ADC_Read (void);
/**
 @brief    Multiple ADC measurement
 
 @param    channel: selected ADC channel 0-7
 @param    nsamples: number of samples (max value 64, because 64*1023=65472 and uint16_t max value is 65536
 @return   Sum of ADC samples on selected channel
*/
static uint16_t ADC_Read_Avg (uint8_t channel, uint8_t nsamples);

/**
 @brief    Measure battery voltage
 
 @param    none
 @return   Measured voltage in mV
*/
static uint16_t Measure_U(void);
/**
 @brief    Measure battery current
 
 @param    none
 @return   Measured current in mA
*/
static uint16_t Measure_I(void);
/**
 @brief    Enable or disable selected battery
  
 @param    batt_select: 1-2
 @return   on_off: 1(on) - 0(off)
*/
//static void Battery_Enable (uint8_t batt_select, uint8_t on_off);
/**
 @brief    Enable or disable Discharge MOSFET
 
 @param    on_off: 1(on) - 0(off)
 @return   none
*/
//static void Discharge_Enable (uint8_t on_off);

/**
 @brief    Enable or disable  MOSFETs
 
 @param    pin: declared pins for MOSFET transistors
 @param    on_off: 1(on) - 0(off)
 @return   none
*/
static void Mosfet_switch (uint8_t pin, uint8_t on_off);
/**
 @brief    Terminate charging with the given message. Disables batteries & PWM & watchdog + puts CPU to sleep mode.
 
 @param    str: message to display
 @return   none
*/
static void Terminate(const char *str);
/**
 @brief    Discharge all batteries to 1V
 
 @param    none
 @return   next mode after discharging
*/
static uint8_t Discharge_Battery (void);
/**
 @brief    Charge selected battery with the given current. Also displays a message on LCD and through UART.
 
 @param    battery_number: 1-2
 @param    max_current: charging current
 @param    string_for_display: message for LCD and UART
 @return   next mode
*/
static uint8_t Charge_Battery(uint8_t battery_number, uint16_t max_current, const char *string_for_display);
/**
 @brief    Loops through the batteries to check whether they are inserted or not. Timeout/battery is 60 sec.
 
 @param    none
 @return   next mode
*/
static uint8_t Battery_Detection(void);
/**
 @brief    The main state machine used to determine next mode or end of charging
 
 @param    battery_number: 1-2
 @param    voltage: voltage in mV
 @param    current: current in mA
 @param    temperature: temp in integer format
 @param    time_left_in_this_mode: indicate how many seconds left in this mode before it ends
 @return   next mode
*/
static uint8_t Check_battery_status(uint8_t battery_number, uint16_t voltage, uint16_t current, uint16_t *temperature, uint16_t *time_left_in_this_mode);
/**
 @brief    Sends an integer number to UART with new line & carrier return characters
 
 @param    i: number to send
 @return   none
*/
static void uart_send_integer (int i);
/**
 @brief    Sends a string to UART stored in FLASH memory
 
 @param    str: String to send
 @return   none
*/
static void uart_send_FLASH_string(const char *str);
/**
 @brief    Sends a new line & carrier return character
 
 @param    none
 @return   none
*/
static void uart_send_CR_LF(void);
/**
 @brief    Sends variable quantity of integers to UART. Used for Debugging
 
 @param    n_args: number of arguments
 @return   none
*/
static void UART_send_data_block(uint8_t n_args, ...);
/**
 @brief    Show an integer number on LCD in the selected row.
 
 @param    row_number: 0(1st row), 1(2nd row)
 @param    integer: the number to display
 @return   none
*/
static void lcd_send_integer(uint8_t row_number, uint16_t *integer);

/**
 @brief    Show a string on LCD in the selected row with 1 second delay if necessary
 
 @param    row_number: 0(1st), 1( 2nd)
 @param    isDelayEnabled: 0(disable), 1(enable)
 @param    str: the string to display
 @return   none
*/
static void LCD_Send_row(uint8_t row_number, uint8_t isDelayEnabled, const char *str);
/**
 @brief    Clean only the selected row by sending 16 space chars
 
 @param    row_number: 0(1st), 1(2nd)
 @return   none
*/
static void lcd_clean_row (uint8_t row_number);
/**
 @brief    Show battery voltage, current and  temperature on LCD in the 2nd row.
 
 @param    volts: voltage in mV
 @param    current: current in mA
 @param    temperature: temp in integer format
 @return   next mode
*/
static void LCD_Send_Debug(uint16_t volts, uint16_t current, uint16_t temperature);

/**
 @brief    Fill array with numbers used to temporarly hold data before sending to LCD
 
 @param    min: position on LCD
 @param    max: position on LCD
 @param    number: shown value
 @param    lcd: filled up array which is returned
 @return   none
*/
static void lcd_fill_array (uint8_t min, uint8_t max, uint16_t number, char *lcd);
/**
 @brief    Detect which button was pressed.
 
 @param    none
 @return   button number
*/
static uint8_t Read_analog_buttons(void);
/**
 @brief    Decrease the given value by 1 but don't  less than the minimum.
 
 @param    min: minimum value
 @param    value: value to decrease
 @return   none
*/
static void Button_Left(uint16_t min, uint16_t *value);
/**
 @brief    Increase the given value by 1 until the maximum limit is reached
 
 @param    max: maximum allowed value
 @param    value: value to increase
 @return   none
*/
static void Button_Right(uint16_t max, uint16_t *value);
/**
 @brief    Select next/previous menu item. If value changed in current menu item but not saved yet than offers to save it. After saving procedure remains in the same menu item to confirm whether value really saved or not.
 
 @param    eep_value: old value (stored in EEPROM)
 @param    ram_value: new value (stored in SRAM)
 @param    save_flag: prevents entering again in "Save?" dialog if it's cancelled.
 @param    isMenuSelected: if set, instead of jumping to the next menu item, remains in the current one.
 @param    menuItem: current menu item, changed to the next or previous menu item if isMenuSelected 0
 @param    next_menu: shows that menu point where to jump when saving cancelled or value doesn't changed.
 @return   none
*/
static void Button_Up_Down(uint16_t eep_value, uint16_t ram_value, uint8_t *save_flag, uint8_t *isMenuSelected, uint8_t *menuItem, uint8_t next_menu);
/**
 @brief    User menu to start charging or to change different charging parameters
 
 @param    none
 @return   next mode (usually INIT_MODE)
*/
static uint8_t Menu (void);
/**
 @brief    Initialize Watchdog Timer to the maximum allowed value (~2 seconds). If it isn't disabled within this time a Watchdog reset occures. Charging will continue from that charging phase where the reset occured.
 
 @param    none
 @return   none
*/
// TODO: delete, use wdt.h
//static void WDT_Init(void);
/**
 @brief    Disable Watchdog Timer
 
 @param    none
 @return   none
*/
//static void WDT_Off(void);

 /**
 @brief    Read out and reset MCUCSR early during startup. Used to restore charging procedure after Watchdog Reset.
 
 @param    none
 @return   none
*/
void handle_mcucsr(void)
  __attribute__((section(".init3")))
  __attribute__((naked));



#endif /* NIMH_CHARGER_H_ */