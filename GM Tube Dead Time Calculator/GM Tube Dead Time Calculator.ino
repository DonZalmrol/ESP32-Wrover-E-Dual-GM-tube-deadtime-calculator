// Include libraries
#include <driver/pcnt.h>
#include <esp_adc_cal.h>

// Helper functions declarations
float displayTubeVoltage(float);
static void Init_PulseCounter_01(void);
static void Init_PulseCounter_02(void);

// Variables
int countingTimeTotal = 60;
int increaseSecCount = 1, increaseSixtySecCount = 0, countingTime = 60;
float tubeVoltage = 0.0, deadtime = 0.0;

int16_t cps_1 = 0, cps_2 = 0, cpm_temp = 0;
unsigned long currentMillis = 0, previousMillis_1 = 0, previousMillis_2 = 0;

// ESP32 Pulse Counter
static pcnt_config_t pcnt_config_01 = {};
static pcnt_config_t pcnt_config_02 = {};

#define PCNT_UNIT_01 PCNT_UNIT_0   // Select the Pulse Counter Unit 0
#define PCNT_UNIT_02 PCNT_UNIT_1   // Select the Pulse Counter Unit 1

#define PCNT_H_LIM_VAL 100          // Set the high limit count to trigger the interrupt
#define PCNT_L_LIM_VAL 0            // Set the low limit count to trigger the interrupt
#define PCNT_FILTER_VAL 0         // Set the filter value from 0-1023

#define PCNT_INPUT_SIG_IO_01 13     // Pulse Input selected as GPIO14
#define PCNT_INPUT_SIG_IO_02 14     // Pulse Input selected as GPIO12

volatile bool eventTriggerd = false;
pcnt_isr_handle_t user_isr_handle = NULL; // User ISR handler for Interrupt

//--------------------//
//--- START SETUP ---//
//------------------//
void setup()
{
  // Initialize UART on 115200 baud rate for ethernet shield
  Serial.begin(115200);

  Serial.println(F("Initializing Serial."));
  while (!Serial)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println(F("Serial initialized."));

  // Initialize Arduino pins
  pinMode(13, INPUT_PULLUP);  // Set pin13 (GPIO14) input for capturing GM Tube 01 events (pulses)
  pinMode(14, INPUT_PULLUP);  // Set pin14 (GPIO12) input for  capturing GM Tube 02 events (pulses)

  // Initialize Pulse Counter (PCNT)
  Init_PulseCounter_01();
  Init_PulseCounter_02();

  Serial.println(F("--- Start program ---"));
  Serial.println(F("\n"));
}
//------------------//
//--- END SETUP ---//
//----------------//

//-------------------//
//--- START LOOP ---//
//-----------------//
void loop()
{
  // If reached 01 seconds with a total of countingTime seconds
  if (((millis() - previousMillis_1) >= 1000UL) && (increaseSecCount <= countingTimeTotal))
  {
    Serial.println("| eventTriggerd value = " + String(eventTriggerd));
    if(eventTriggerd)
    {
      Serial.println(F("--- ISR ---"));
      Serial.println("| Counter of tube exceeded the value of " + String(PCNT_H_LIM_VAL));
      Serial.println(F("--- ISR ---"));

      eventTriggerd = false;
    }

    pcnt_get_counter_value(PCNT_UNIT_01, &cps_1);
    pcnt_get_counter_value(PCNT_UNIT_02, &cps_2);

    Serial.println(F("--- 01 sec ---"));
    Serial.println("| Tube 1 " + String(increaseSecCount) + " sec current count (cps_1) = " + String(cps_1));
    Serial.println("| Tube 2 " + String(increaseSecCount) + " sec current count (cps_2) = " + String(cps_2));
    Serial.println("| Tube voltage = " + String(displayTubeVoltage(), 3) + " V");
    Serial.println(F("--- 01 sec ---"));

    // Display 1 second tick for a total of 60
    Serial.println(String(increaseSecCount) + " of " + String(countingTimeTotal));

    increaseSecCount++;

    previousMillis_1 = millis();
    previousMillis_2 = millis();
  }
  // If countingTime seconds have been reached do simple check of values
  if ( ( (millis() - previousMillis_2) >= (countingTimeTotal * 1000UL) + 1000UL ) && (increaseSecCount >= countingTimeTotal))
  {
    // Raw CPM
    //cpm_temp = cps_1 + cps_2;

    // Print new line
    Serial.println(F("\n"));

    /*
      // Uncomment below for calculating the tube dead time, only use when you have a high enough test source!
      // Otherwise your ESP32 will
      // Values from Excel
      float c3 = countingTimeTotal;
      float c5 = cps_1;
      float c6 = cps_2;
      floatc 7 = 1; //cpm_temp OR 1

      float rc = 1.0;
      float deadtime = ( (C5/C3) + (C6/C3) - (C7/C3) ) / ( 2 * (C5/C3) * (C6/C3) );
    */

    Serial.println("--- " + String(increaseSecCount) + " sec ---");
    Serial.println("| Tube 1 C.P.M = " + String(cps_1));
    Serial.println("| Tube 2 C.P.M = " + String(cps_2));
    Serial.println("| Tube voltage = " + String(displayTubeVoltage(), 3) + " V");
    Serial.println("--- " + String(increaseSecCount) + " sec ---");

    Serial.println(String(increaseSecCount) + " of " + String(countingTimeTotal));

    // Print new line
    Serial.println(F("\n"));

    // Reset variables
    cps_1 = 0, cps_2 = 0, cpm_temp = 0, deadtime = 0.0;
    increaseSecCount = 1;
    
    Clean_Counters();

    previousMillis_1 = millis();
    previousMillis_2 = millis();
  }
}
//-----------------//
//--- END LOOP ---//
//---------------//

//------------------------//
//--- START functions ---//
//----------------------//

// Measure Tube voltage through A0
float displayTubeVoltage(void)
{
  float adcInput = 0.0, lowVoltage = 0.0, voltage_offset = 0.0, highVoltage = 0.0;

  adc1_config_width(ADC_WIDTH_12Bit);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);
  adcInput = adc1_get_raw(ADC1_CHANNEL_5);

  // ESP32 pin 33 measures from 0-3.3V and has a width of 0-4096
  // Use 3.4 for as correcting value
  lowVoltage = ((adcInput * 3.4 ) / 4096.0);
  
  /*
    If you want to monitor high voltage on tubes with microcontroller ADC you can use A0 module output.
    ~190-195 conversion factor according to Alex - RH Electronics 2021-03-15
    ~184.097 calcultated on 2022-09-20 for ESP32 Wrover-E after research, see below

    Calculate your value with the "ADC Linearity calculator", yellow is your measured ADC value.
    Measure the voltage between your tubes and multiple x2, measure voltage of pin A0
    
    Measured:
     - ADC = 2863
     - Voltage of tube = 2x 218.8V = 437.6V
     - Voltage of A0 = 2.37V

    Calculated:
     - Voltage should be 2377mV
     - Conversion factor calculation = 437.6V / 2.377 = 184.097
     - Confirm with calculated high voltage table     
  */
  voltage_offset = 184.097;
  highVoltage = (lowVoltage * voltage_offset);

  // Uncomment for testing
  //Serial.println("| adcInput = " + String(adcInput, 3));
  //Serial.println("| A0 LV = " + String(lowVoltage, 3));
  //Serial.println("| Tubes HV = " + String(highVoltage, 3));

  return highVoltage;
}

// ISR Function for counter 1
static void IRAM_ATTR Counter_ISR(void *arg)
{
  eventTriggerd = true;

  esp_intr_disable(user_isr_handle);
  user_isr_handle = NULL;
}

static void Init_PulseCounter_01(void)
{
  pcnt_config_01.pulse_gpio_num = PCNT_INPUT_SIG_IO_01;   // Set pulse input GPIO member
  pcnt_config_01.ctrl_gpio_num = PCNT_PIN_NOT_USED;       // No GPIO for control

  // What to do on the positive / negative edge of pulse input?
  pcnt_config_01.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
  pcnt_config_01.neg_mode = PCNT_COUNT_DIS;   // Count down disable

  // What to do when control input is low or high?
  pcnt_config_01.lctrl_mode = PCNT_MODE_KEEP; // Keep the primary counter mode if low
  pcnt_config_01.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode 

  // Set the maximum and minimum limit values to watch
  pcnt_config_01.counter_h_lim  = PCNT_H_LIM_VAL;
  pcnt_config_01.counter_l_lim  = PCNT_L_LIM_VAL;

  pcnt_config_01.unit = PCNT_UNIT_01;                           // Select pulse unit
  pcnt_config_01.channel = PCNT_CHANNEL_0;                      // Select PCNT channel 0
  pcnt_unit_config(&pcnt_config_01);                            // Configure PCNT

  pcnt_counter_pause(PCNT_UNIT_01);                             // Pause PCNT counter
  pcnt_counter_clear(PCNT_UNIT_01);                             // Clear PCNT counter

  pcnt_set_filter_value(PCNT_UNIT_01, PCNT_FILTER_VAL);         // Maximum filter_val should be limited to 1023.
  pcnt_filter_enable(PCNT_UNIT_01);                             // Enable filter

  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_H_LIM);              // Enable event for when PCNT watch point event: Maximum counter value
  pcnt_event_enable(PCNT_UNIT_01, PCNT_EVT_L_LIM);

  pcnt_isr_register(Counter_ISR, NULL, 0, &user_isr_handle);    // Set call back function for the Event
  pcnt_intr_enable(PCNT_UNIT_01);                               // Enable Pulse Counter (PCNT)

  pcnt_counter_resume(PCNT_UNIT_01);
  
  Serial.println(F("PCNT_01 Init Completed"));
}

static void Init_PulseCounter_02(void)
{
  pcnt_config_02.pulse_gpio_num = PCNT_INPUT_SIG_IO_02;   // Set pulse input GPIO member
  pcnt_config_02.ctrl_gpio_num = PCNT_PIN_NOT_USED;       // No GPIO for control 

  // What to do on the positive / negative edge of pulse input?
  pcnt_config_02.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
  pcnt_config_02.neg_mode = PCNT_COUNT_DIS;   // Count down disable

  // What to do when control input is low or high?
  pcnt_config_02.lctrl_mode = PCNT_MODE_KEEP;     // Keep the primary counter mode if low
  pcnt_config_02.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode

  // Set the maximum and minimum limit values to watch
  pcnt_config_02.counter_h_lim  = PCNT_H_LIM_VAL;
  pcnt_config_02.counter_l_lim  = PCNT_L_LIM_VAL;

  pcnt_config_02.unit = PCNT_UNIT_02;                           // Select pulse unit
  pcnt_config_02.channel = PCNT_CHANNEL_1;                      // Select PCNT channel 1
  pcnt_unit_config(&pcnt_config_02);                            // Configure PCNT

  pcnt_counter_pause(PCNT_UNIT_02);                             // Pause PCNT counter
  pcnt_counter_clear(PCNT_UNIT_02);                             // Clear PCNT counter

  pcnt_set_filter_value(PCNT_UNIT_02, PCNT_FILTER_VAL);         // Set filter value
  pcnt_filter_enable(PCNT_UNIT_02);                             // Enable filter

  pcnt_event_enable(PCNT_UNIT_02, PCNT_EVT_H_LIM);              // Enable event for when PCNT watch point event: Maximum counter value
  pcnt_event_enable(PCNT_UNIT_02, PCNT_EVT_L_LIM);

  pcnt_isr_register(Counter_ISR, NULL, 0, &user_isr_handle);    // Set call back function for the Event
  pcnt_intr_enable(PCNT_UNIT_02);                               // Enable Pulse Counter (PCNT)

  pcnt_counter_resume(PCNT_UNIT_02);
  
  Serial.println(F("PCNT_02 Init Completed"));
}

// Function to clean the Counter and its variables
static void Clean_Counters()                                       
{
  pcnt_counter_pause(PCNT_UNIT_01);    // Pause Pulse counters such that we can set event
  pcnt_counter_pause(PCNT_UNIT_02);

  pcnt_counter_clear(PCNT_UNIT_01);    // Clean Pulse Counters
  pcnt_counter_clear(PCNT_UNIT_02);

  pcnt_counter_resume(PCNT_UNIT_01);   // Resume Pulse Counters
  pcnt_counter_resume(PCNT_UNIT_02);
}