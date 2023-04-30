/* Trailer for RC engine sound & LED controller for Arduino ESP32. Written by TheDIYGuy999
use it in combination with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32
*/

char codeVersion[] = "1.0-beta.5"; // Software revision.

//
// =======================================================================================================
// ERROR CODES (INDICATOR LIGHTS)
// =======================================================================================================
//
/*
   Indicators:
   - Constantly on = no SBUS signal (check "sbusInverted" true / false in "2_Remote.h")
   - Number of blinks = this channel signal is not between 1400 and 1600 microseconds and can't be auto calibrated (check channel trim settings)
   - 2 fast blinks = battery error during powering up (see below)
   - 3 fast blinks = no valid receiver bus signal detected / wrong remote configuration

   Beeps (only, if "#define BATTERY_PROTECTION" in "3_ESC.h"):
   - Number of beeps = number of detected battery cells in series
   - 10 fast beeps = battery error, disconnect it!
*/

//
// =======================================================================================================
// ! ! I M P O R T A N T ! !   ALL USER SETTINGS ARE DONE IN THE FOLLOWING TABS, WHICH ARE DISPLAYED ABOVE
// (ADJUST THEM BEFORE CODE UPLOAD), DO NOT CHANGE ANYTHING IN THIS TAB EXCEPT THE DEBUG OPTIONS
// =======================================================================================================
//

// All the required user settings are done in the following .h files:
#include "0_generalSettings.h" // <<------- general settings
#include "3_Battery.h"         // <<------- Battery related adjustments
#include "7_Servos.h"          // <<------- Servo output related adjustments

//
// =======================================================================================================
// LIRBARIES & HEADER FILES, REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder.
// No manual library download is required in Visual Studio Code IDE (see platformio.ini)
#include <statusLED.h>       // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control
#include <ESP32AnalogRead.h> // https://github.com/madhephaestus/ESP32AnalogRead <<------- required for battery voltage measurement

// No need to install these, they come with the ESP32 board definition
#include "driver/mcpwm.h" // for servo PWM output
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Esp.h>         // for displaying memory information
#include "rom/rtc.h"     // for displaying reset reason
#include "soc/rtc_wdt.h" // for watchdog timer
#include "EEPROM.h"

// The following tasks are not required for Visual Studio Code IDE! ----
// Install ESP32 board according to: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
// Warning: do not use Espressif ESP32 board definition v1.05, its causing crash & reboot loops! Use v1.04 instead.
// Adjust board settings according to: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/pictures/settings.png

// Make sure to remove -master from your sketch folder name

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES (Do not play around here)
// =======================================================================================================
//
// Pin assignment and wiring instructions ******************************************************

// ------------------------------------------------------------------------------------
// Use a 330Ohm resistor in series with all I/O pins! allows to drive LED directly and
// provides short circuit protection. Also works on the serial Rx pin "VP" (36)
// ------------------------------------------------------------------------------------

#define TAILLIGHT_PIN 15       // Red tail- & brake-lights (combined)
#define INDICATOR_LEFT_PIN 2   // Orange left indicator (turn signal) light
#define INDICATOR_RIGHT_PIN 4  // Orange right indicator (turn signal) light
#define REVERSING_LIGHT_PIN 17 // (TX2) White reversing light
#define SIDELIGHT_PIN 18       // Side lights

#define SERVO_1_PIN 13 // Servo CH1 legs
#define SERVO_2_PIN 12 // Servo CH2 ramps
#define SERVO_3_PIN 14 // Servo CH3 beacon control
#define SERVO_4_PIN 27 // Servo CH4 spare servo channel

#define FIFTH_WHEEL_DETECTION_PIN 32 // This NO switch is closed, if the trailer is coupled to the 5th wheel. Between Pin 32 and GND

#define BATTERY_DETECT_PIN 39 // Voltage divider resistors connected to pin "VN & GND"

// Objects *************************************************************************************
// Status LED objects -----
statusLED tailLight(false); // "false" = output not inversed
statusLED indicatorL(false);
statusLED indicatorR(false);
statusLED reversingLight(false);
statusLED sideLight(false);

// Battery voltage
ESP32AnalogRead battery;

// Global variables ****************************************************************************

esp_now_peer_info_t peerInfo; // This MUST be global!! Transmission is not working otherwise!

typedef struct struct_message
{ // This is the data packet
  uint8_t tailLight;
  uint8_t sideLight;
  uint8_t reversingLight;
  uint8_t indicatorL;
  uint8_t indicatorR;
  bool legsUp;
  bool legsDown;
  bool rampsUp;
  bool rampsDown;
  bool beaconsOn;
} struct_message;

// Create a struct_message called trailerData
struct_message trailerData;

bool trailerCoupled;

String ssid;
String password;

// MAC address for communication with tractor
uint8_t customMACAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Webserver on port 80
WiFiServer server(80);

// HTTP request memory
String header;

// For HTTP GET value
String valueString = "";
int pos1 = 0;
int pos2 = 0;

// Battery
float batteryCutoffvoltage;
float batteryVoltage;
uint8_t numberOfCells;
bool batteryProtection = false;

// These are used to print the reset reason on startup
const char *RESET_REASONS[] = {"POWERON_RESET", "NO_REASON", "SW_RESET", "OWDT_RESET", "DEEPSLEEP_RESET", "SDIO_RESET", "TG0WDT_SYS_RESET", "TG1WDT_SYS_RESET", "RTCWDT_SYS_RESET", "INTRUSION_RESET", "TGWDT_CPU_RESET", "SW_CPU_RESET", "RTCWDT_CPU_RESET", "EXT_CPU_RESET", "RTCWDT_BROWN_OUT_RESET", "RTCWDT_RTC_RESET"};

// The following variables are buffered in the eeprom an can be modified, using the web interface -----
// 5th wheel switch enable / disable
bool fifthWhweelDetectionActive = true;
bool useCustomMac = true;

// Light brightness percentages
uint8_t tailLightBrightness = 100;
uint8_t sideLightBrightness = 100;
uint8_t reversingLightBrightness = 100;
uint8_t indicatorLightBrightness = 100;

// Eeprom size and storage addresses -----------------------------------------------------------------
#define EEPROM_SIZE 256 // 256 Bytes (512 is maximum)

#define adr_eprom_init 0 // Eeprom initialized or not?
#define adr_eprom_useCustomMac 4
#define adr_eprom_Mac0 8
#define adr_eprom_Mac1 12
#define adr_eprom_Mac2 16
#define adr_eprom_Mac3 20
#define adr_eprom_Mac4 24
#define adr_eprom_Mac5 28
#define adr_eprom_fifthWhweelDetectionActive 32
#define adr_eprom_tailLightBrightness 36
#define adr_eprom_sideLightBrightness 40
#define adr_eprom_reversingLightBrightness 44
#define adr_eprom_indicatorLightBrightness 48
#define adr_eprom_ssid 64      // 64
#define adr_eprom_password 128 // 128

//
// =======================================================================================================
// ESP NOW TRAILER DATA RECEIVED CALLBACK
// =======================================================================================================
//

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&trailerData, incomingData, sizeof(trailerData));

  // led(); // This is now handled in loop

  Serial.print("Tailllight: ");
  Serial.println(trailerData.tailLight * tailLightBrightness / 100);
  Serial.print("Sidelight: ");
  Serial.println(trailerData.sideLight * sideLightBrightness / 100);
  Serial.print("Reversing light: ");
  Serial.println(trailerData.reversingLight * reversingLightBrightness / 100);
  Serial.print("Indicator L: ");
  Serial.println(trailerData.indicatorL * indicatorLightBrightness / 100);
  Serial.print("Indicator R: ");
  Serial.println(trailerData.indicatorR * indicatorLightBrightness / 100);
  Serial.print("Legs up: ");
  Serial.println(trailerData.legsUp);
  Serial.print("Legs down: ");
  Serial.println(trailerData.legsDown);
  Serial.print("Ramps up: ");
  Serial.println(trailerData.rampsUp);
  Serial.print("Ramps down: ");
  Serial.println(trailerData.rampsDown);
  Serial.print("Beacons on: ");
  Serial.println(trailerData.beaconsOn);

  Serial.println();
}

//
// =======================================================================================================
// mcpwm SETUP (1x during startup)
// =======================================================================================================
//
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#configure

void setupMcpwm()
{
  // 1. set our servo output pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_1_PIN); // Set legs as PWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_2_PIN); // Set ramps as PWM0B
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, SERVO_4_PIN); // Set beacon as PWM1A
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, SERVO_3_PIN); // Set spare servo as PWM1B

  // 2. configure MCPWM parameters
  mcpwm_config_t pwm_config;
  pwm_config.frequency = SERVO_FREQUENCY; // frequency usually = 50Hz, some servos may run smoother @ 100Hz
  pwm_config.cmpr_a = 0;                  // duty cycle of PWMxa = 0
  pwm_config.cmpr_b = 0;                  // duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // 0 = not inverted, 1 = inverted

  // 3. configure channels with settings above
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config); // Configure PWM1A & PWM1B
}

//
// =======================================================================================================
// ESP NOW SETUP FOR WIRELESS TRAILER CONTROL & WIFI SETUP FOR WEB CONFIGURATION
// =======================================================================================================
//

void setupEspNow()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set IP address
  IPAddress IP = WiFi.softAPIP();

  // Set custom MAC address
  if (useCustomMac)
  {
    esp_wifi_set_mac(WIFI_IF_STA, &customMACAddress[0]);
    // esp_wifi_set_mac(ESP_IF_WIFI_STA, &customMACAddress[0]); // Board before 1.0.5
  }

  // Print MAC address (this is the required MAC address in the transmitter)
  Serial.printf("\nInformations for communication with tractor ******************************************************\n");
  Serial.print("Currently used MAC address (add it in '10_Trailer.h' in the sound controller): ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Custom MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", customMACAddress[0], customMACAddress[1], customMACAddress[2], customMACAddress[3], customMACAddress[4], customMACAddress[5]);

  Serial.printf("\nInformations for web configuration via your cell phone or computer *******************************\n");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("IP address: ");
  Serial.println(IP);

  WiFi.disconnect();

  WiFi.softAP(ssid.c_str(), password.c_str());

  Serial.printf("\nWiFi Tx Power Level: %u", WiFi.getTxPower());
  WiFi.setTxPower(cpType); // WiFi and ESP-Now power according to "0_generalSettings.h"
  Serial.printf("\nWiFi Tx Power Level changed to: %u\n\n", WiFi.getTxPower());

  server.begin(); // Start Webserver

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

//
// =======================================================================================================
// BATTERY SETUP
// =======================================================================================================
//

void setupBattery()
{
#if defined BATTERY_PROTECTION

  Serial.printf("Battery voltage: %.2f V\n", batteryVolts());
  Serial.printf("Cutoff voltage per cell: %.2f V\n", CUTOFF_VOLTAGE);
  Serial.printf("Fully charged voltage per cell: %.2f V\n", FULLY_CHARGED_VOLTAGE);

#define CELL_SETPOINT (CUTOFF_VOLTAGE - ((FULLY_CHARGED_VOLTAGE - CUTOFF_VOLTAGE) / 2))

  if (batteryVolts() <= CELL_SETPOINT * 2)
    numberOfCells = 1;
  if (batteryVolts() > CELL_SETPOINT * 2)
    numberOfCells = 2;
  if (batteryVolts() > CELL_SETPOINT * 3)
    numberOfCells = 3;
  if (batteryVolts() > FULLY_CHARGED_VOLTAGE * 3)
    numberOfCells = 4;
  batteryCutoffvoltage = CUTOFF_VOLTAGE * numberOfCells; // Calculate cutoff voltage for battery protection
  if (numberOfCells > 1 && numberOfCells < 4)
  { // Only 2S & 3S batteries are supported!
    Serial.printf("Number of cells: %i (%iS battery detected) Based on setpoint: %.2f V\n", numberOfCells, numberOfCells, (CELL_SETPOINT * numberOfCells));
    Serial.printf("Battery cutoff voltage: %.2f V (%i * %.2f V) \n", batteryCutoffvoltage, numberOfCells, CUTOFF_VOLTAGE);
    for (uint8_t beeps = 0; beeps < numberOfCells; beeps++)
    {                    // Number of beeps = number of cells in series
                         // tone(26, 3000, 4, 0);
      tone(26, 3000, 4); // For platform = espressif32@4.3.0
      delay(200);
    }
  }
  else
  {
    Serial.printf("Error, no valid battery detected! Only 2S & 3S batteries are supported!\n");
    Serial.printf("REMOVE BATTERY, CONTROLLER IS LOCKED = 2 FAST FLASHES!\n");
    bool locked = true;
    while (locked)
    {
      // wait here forever!
      indicatorL.flash(70, 75, 500, 2); // Show 2 fast flashes on indicators!
      indicatorR.flash(70, 75, 500, 2);
      rtc_wdt_feed(); // Feed watchdog timer
    }
  }
#else
  Serial.printf("Warning, BATTERY_PROTECTION disabled!\n");
#endif
  Serial.printf("-------------------------------------\n");
}

//
// =======================================================================================================
// EEPROM SETUP
// =======================================================================================================
//

void setupEeprom()
{
  EEPROM.begin(EEPROM_SIZE);
#if defined ERASE_EEPROM_ON_BOOT
  eepromErase(); // uncomment this option, if you want to erase all stored settings!
#endif
  eepromInit(); // Init new board with default values
  eepromRead(); // Read settings from Eeprom
  Serial.print("current eeprom_id: ");
  Serial.println(EEPROM.read(adr_eprom_init));
  Serial.println("change it for default value upload!\n");
  eepromDebugRead(); // Shows content of entire eeprom, except of empty areas
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup()
{

#if defined LOW_POWER_MODE
  setCpuFrequencyMhz(80); // Lower CPU clock frequency to 80MHz to save energy
#endif

  // Serial setup
  Serial.begin(115200); // USB serial (for DEBUG)

  // ADC setup
  battery.attach(BATTERY_DETECT_PIN);

  // Print some system and software info to serial monitor
  delay(1000); // Give serial port/connection some time to get ready
  Serial.printf("\n**************************************************************************************************\n");
  Serial.printf("Wireless trailer for ESP32 software version %s\n", codeVersion);
  Serial.printf("https://github.com/TheDIYGuy999\n");
  Serial.printf("Use it in combination with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("XTAL Frequency: %i MHz, CPU Clock: %i MHz, APB Bus Clock: %i Hz\n", getXtalFrequencyMhz(), getCpuFrequencyMhz(), getApbFrequency());
  Serial.printf("Internal RAM size: %i Byte, Free: %i Byte\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("WiFi MAC address: %s\n", WiFi.macAddress().c_str());
  for (uint8_t coreNum = 0; coreNum < 2; coreNum++)
  {
    uint8_t resetReason = rtc_get_reset_reason(coreNum);
    if (resetReason <= (sizeof(RESET_REASONS) / sizeof(RESET_REASONS[0])))
    {
      Serial.printf("Core %i reset reason: %i: %s\n", coreNum, rtc_get_reset_reason(coreNum), RESET_REASONS[resetReason - 1]);
    }
  }

#if defined BATTERY_PROTECTION
  Serial.printf("Battery protection calibration data:\n");
  Serial.printf("RESISTOR_TO_BATTTERY_PLUS: %i Ω\n", RESISTOR_TO_BATTTERY_PLUS);
  Serial.printf("RESISTOR_TO_GND: %i Ω\n", RESISTOR_TO_GND);
  Serial.printf("DIODE_DROP: %.2f V\n", DIODE_DROP);
#endif

  Serial.printf("**************************************************************************************************\n\n");

  // Eeprom setup
  setupEeprom();

  // Pin setup
  pinMode(FIFTH_WHEEL_DETECTION_PIN, INPUT_PULLUP);

  delay(1000);

  // LED setup (note, that we only have timers from 0 - 15)
  tailLight.begin(TAILLIGHT_PIN, 2, 20000);            // Timer 2, 20kHz
  sideLight.begin(SIDELIGHT_PIN, 3, 20000);            // Timer 3, 20kHz
  reversingLight.begin(REVERSING_LIGHT_PIN, 4, 20000); // Timer 4, 20kHz
  indicatorL.begin(INDICATOR_LEFT_PIN, 5, 20000);      // Timer 5, 20kHz
  indicatorR.begin(INDICATOR_RIGHT_PIN, 6, 20000);     // Timer 6, 20kHz

  // Battery setup
  setupBattery();

  // ESP NOW setup
  setupEspNow();

  tailLight.pwm(255 * tailLightBrightness / 100);
  sideLight.pwm(255 * sideLightBrightness / 100);
  reversingLight.pwm(255 * reversingLightBrightness / 100);
  indicatorL.pwm(255 * indicatorLightBrightness / 100);
  indicatorR.pwm(255 * indicatorLightBrightness / 100);

  delay(500); // LED test during 0.5s (all on)

  tailLight.pwm(0);
  sideLight.pwm(0);
  reversingLight.pwm(0);
  indicatorL.pwm(0);
  indicatorR.pwm(0);

  delay(2000); // Display serial console

  setupMcpwm(); // mcpwm servo output setup
}

//
// =======================================================================================================
// 5TH WHEEL DETECTION SWITCH
// =======================================================================================================
//

void switchDetect()
{
  static unsigned long switchMillis;

  if (!digitalRead(FIFTH_WHEEL_DETECTION_PIN) || !fifthWhweelDetectionActive)
  {                          // only read switch, if enabled
    switchMillis = millis(); // if coupled
    // Serial.println("coupled");
  }

  trailerCoupled = (millis() - switchMillis <= 1000); // 1s delay, if not coupled
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

void led()
{

  if (trailerCoupled && !batteryProtection)
  {
    tailLight.pwm(trailerData.tailLight * tailLightBrightness / 100);
    sideLight.pwm(trailerData.sideLight * sideLightBrightness / 100);
    reversingLight.pwm(trailerData.reversingLight * reversingLightBrightness / 100);
    indicatorL.pwm(trailerData.indicatorL * indicatorLightBrightness / 100);
    indicatorR.pwm(trailerData.indicatorR * indicatorLightBrightness / 100);
  }
  else
  {
    tailLight.pwm(0);
    sideLight.pwm(0);
    reversingLight.pwm(0);
    if (!batteryProtection)
    {
      indicatorL.pwm(0);
      indicatorR.pwm(0);
    }
    else
    {
      indicatorL.flash(70, 75, 500, 2); // Show 2 fast flashes on indicators!
      indicatorR.flash(70, 75, 500, 2);
    }
  }
}

//
// =======================================================================================================
// ROTATING BEACON CONTROL (disconnect USB & battery after upload for correct beacon function)
// =======================================================================================================
//

bool beaconControl(uint8_t pulses)
{

  /* Beacons: "RC DIY LED Rotating Beacon Light Flash For 1/10 Truck Crawler Toy"
      from: https://www.ebay.ch/itm/303979210629
      States (every servo signal change from 1000 to 2000us will switch to the next state):
      0 rotating beacon slow
      1 Rotating beacon slow
      2 4x flash
      3 endless flash
      4 off

  */

  static unsigned long pulseMillis;
  static unsigned long pulseWidth = CH3L;
  static uint8_t i;

  if (millis() - pulseMillis > 40)
  { // Every 40ms (this is the required minimum)
    pulseMillis = millis();
    if (pulseWidth == CH3L)
    {
      pulseWidth = CH3R;
    }
    else
    {
      pulseWidth = CH3L;
      i++;
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, pulseWidth);
  }

  if (i >= pulses)
  {
    i = 0;
    return true;
  }
  else
    return false;
}

//
// =======================================================================================================
// MCPWM SERVO RC SIGNAL OUTPUT (BUS communication mode only)
// =======================================================================================================
//
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#configure

void mcpwmOutput()
{

  // Legs servo CH1 (active, if 5th wheel is unlocked, use horn pot) **********************

#ifdef LEGS_ESC_MODE // ESC mode
  static uint16_t legsServoMicros = CH1L;
  if (trailerData.legsDown)
    legsServoMicros = CH1L; // down
  else if (trailerData.legsUp)
    legsServoMicros = CH1R; // up
  else
    legsServoMicros = CH1C; // off

#else // Servo mode
  static uint16_t legsServoMicrosTarget = CH1L;
  static uint16_t legsServoMicros = CH1L;
  static unsigned long legsDelayMicros;
  if (micros() - legsDelayMicros > LEGS_RAMP_TIME)
  {
    legsDelayMicros = micros();
    if (trailerData.legsDown)
      legsServoMicrosTarget = CH1L; // down
    else if (trailerData.legsUp)
      legsServoMicrosTarget = CH1R; // up
    else
      legsServoMicrosTarget = legsServoMicros; // stop
    if (legsServoMicros < legsServoMicrosTarget)
      legsServoMicros++;
    if (legsServoMicros > legsServoMicrosTarget)
      legsServoMicros--;
  }
#endif
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, legsServoMicros);

  // Ramps servo CH2 (active, if hazards are on, use horn pot) *****************************
#ifdef RAMPS_ESC_MODE // ESC mode
  static uint16_t rampsServoMicros = CH2L;
  if (trailerData.rampsDown)
    rampsServoMicros = CH2L; // down
  else if (trailerData.rampsUp)
    rampsServoMicros = CH2R; // up
  else
    rampsServoMicros = CH2C; // off

#else // Servo mode
  static uint16_t rampsServoMicrosTarget = CH2R;
  static uint16_t rampsServoMicros = CH2R;
  static unsigned long rampsDelayMicros;
  if (micros() - rampsDelayMicros > RAMPS_RAMP_TIME)
  {
    rampsDelayMicros = micros();
    if (trailerData.rampsDown)
      rampsServoMicrosTarget = CH2L; // down
    else if (trailerData.rampsUp)
      rampsServoMicrosTarget = CH2R; // up
    else
      rampsServoMicrosTarget = rampsServoMicros; // stop
    if (rampsServoMicros < rampsServoMicrosTarget)
      rampsServoMicros++;
    if (rampsServoMicros > rampsServoMicrosTarget)
      rampsServoMicros--;
  }
#endif
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, rampsServoMicros);

  // Beacon control CH3 (use horn / blue light pot)******************************************
  // Init (5 pulses are required to shut beacons off after power on)
  static bool blueLightInit;
  if (!blueLightInit)
  {
    if (beaconControl(5))
      blueLightInit = true;
  }

  // Switching modes
  static uint16_t beaconServoMicros;
  static bool lockRotating, lockOff;
  if (blueLightInit)
  {
    if (trailerData.beaconsOn && !lockRotating)
    { // Rotating mode on (1 pulse)
      if (beaconControl(1))
      {
        lockRotating = true;
        lockOff = false;
      }
    }
    if (!trailerData.beaconsOn && !lockOff && lockRotating)
    { // Off (4 pulses)
      if (beaconControl(4))
      {
        lockOff = true;
        lockRotating = false;
      }
    }
  }
}

//
// =======================================================================================================
// EEPROM
// =======================================================================================================
//

// Write string to EEPROM ------
// https://roboticsbackend.com/arduino-write-string-in-eeprom/#Write_the_String

int writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
  return addrOffset + 1 + len;
}

// Read string from EEPROM ------
int readStringFromEEPROM(int addrOffset, String *strToRead)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';
  *strToRead = String(data);
  return addrOffset + 1 + newStrLen;
}

// Erase EEPROM ------
void eepromErase()
{
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("EEPROM erased!");
  delay(500);
  eepromDebugRead();
}

// Init new board with the default values you want ------
void eepromInit()
{
  if (EEPROM.readInt(adr_eprom_init) != eeprom_id)
  {
    EEPROM.writeInt(adr_eprom_init, eeprom_id);
    EEPROM.writeInt(adr_eprom_useCustomMac, DefaultUseCustomMac);
    EEPROM.writeInt(adr_eprom_Mac0, defaultCustomMACAddress[0]); // Should always be 0xFE!
    EEPROM.writeInt(adr_eprom_Mac1, defaultCustomMACAddress[1]); // Country number
    EEPROM.writeInt(adr_eprom_Mac2, defaultCustomMACAddress[2]); // Region number
    EEPROM.writeInt(adr_eprom_Mac3, defaultCustomMACAddress[3]); // User Number 1
    EEPROM.writeInt(adr_eprom_Mac4, defaultCustomMACAddress[4]); // User Number 2
    EEPROM.writeInt(adr_eprom_Mac5, defaultCustomMACAddress[5]); // 0x01 = trailer #1
    EEPROM.writeInt(adr_eprom_fifthWhweelDetectionActive, defaulFifthWhweelDetectionActive);
    EEPROM.writeInt(adr_eprom_tailLightBrightness, defaultLightsBrightness);
    EEPROM.writeInt(adr_eprom_sideLightBrightness, defaultLightsBrightness);
    EEPROM.writeInt(adr_eprom_reversingLightBrightness, defaultLightsBrightness);
    EEPROM.writeInt(adr_eprom_indicatorLightBrightness, defaultLightsBrightness);
    writeStringToEEPROM(adr_eprom_ssid, default_ssid);
    writeStringToEEPROM(adr_eprom_password, default_password);
    EEPROM.commit();
    Serial.println("EEPROM initialized.");
  }
}

// Write new values to EEPROM ------
void eepromWrite()
{
  EEPROM.writeInt(adr_eprom_useCustomMac, useCustomMac);
  EEPROM.writeInt(adr_eprom_Mac0, customMACAddress[0]);
  EEPROM.writeInt(adr_eprom_Mac1, customMACAddress[1]);
  EEPROM.writeInt(adr_eprom_Mac2, customMACAddress[2]);
  EEPROM.writeInt(adr_eprom_Mac3, customMACAddress[3]);
  EEPROM.writeInt(adr_eprom_Mac4, customMACAddress[4]);
  EEPROM.writeInt(adr_eprom_Mac5, customMACAddress[5]);
  EEPROM.writeInt(adr_eprom_fifthWhweelDetectionActive, fifthWhweelDetectionActive);
  EEPROM.writeInt(adr_eprom_tailLightBrightness, tailLightBrightness);
  EEPROM.writeInt(adr_eprom_sideLightBrightness, sideLightBrightness);
  EEPROM.writeInt(adr_eprom_reversingLightBrightness, reversingLightBrightness);
  EEPROM.writeInt(adr_eprom_indicatorLightBrightness, indicatorLightBrightness);
  writeStringToEEPROM(adr_eprom_ssid, ssid);
  writeStringToEEPROM(adr_eprom_password, password);
  EEPROM.commit();
  Serial.println("EEPROM written.");
  eepromDebugRead();
}

// Read values from EEPROM ------
void eepromRead()
{
  useCustomMac = EEPROM.readInt(adr_eprom_useCustomMac);
  customMACAddress[0] = EEPROM.readInt(adr_eprom_Mac0);
  customMACAddress[1] = EEPROM.readInt(adr_eprom_Mac1);
  customMACAddress[2] = EEPROM.readInt(adr_eprom_Mac2);
  customMACAddress[3] = EEPROM.readInt(adr_eprom_Mac3);
  customMACAddress[4] = EEPROM.readInt(adr_eprom_Mac4);
  customMACAddress[5] = EEPROM.readInt(adr_eprom_Mac5);
  fifthWhweelDetectionActive = EEPROM.readInt(adr_eprom_fifthWhweelDetectionActive);
  tailLightBrightness = EEPROM.readInt(adr_eprom_tailLightBrightness);
  sideLightBrightness = EEPROM.readInt(adr_eprom_sideLightBrightness);
  reversingLightBrightness = EEPROM.readInt(adr_eprom_reversingLightBrightness);
  indicatorLightBrightness = EEPROM.readInt(adr_eprom_indicatorLightBrightness);
  readStringFromEEPROM(adr_eprom_ssid, &ssid);
  readStringFromEEPROM(adr_eprom_password, &password);

  Serial.println("EEPROM read.");
}

void eepromDebugRead()
{
#if defined DEBUG
  String eepromDebug;
  Serial.println("EEPROM debug dump begin **********************************************");
  eepromDebug = "";
  for (int i = 0; i < EEPROM_SIZE; ++i)
  {
    eepromDebug += char(EEPROM.read(i));
  }
  Serial.println(eepromDebug);
  Serial.println("");
  Serial.println("EEPROM debug dump end ************************************************");
#endif
}

//
// =======================================================================================================
// BATTERY MONITORING
// =======================================================================================================
//

float batteryVolts()
{
  static float raw[6];
  static bool initDone = false;
#define VOLTAGE_CALIBRATION (RESISTOR_TO_BATTTERY_PLUS + RESISTOR_TO_GND) / RESISTOR_TO_GND + DIODE_DROP

  if (!initDone)
  { // Init array, if first measurement (important for call in setup)!
    for (uint8_t i = 0; i <= 5; i++)
    {
      raw[i] = battery.readVoltage();
    }
    initDone = true;
  }

  raw[5] = raw[4]; // Move array content, then add latest measurement (averaging)
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];

  raw[0] = battery.readVoltage(); // read analog input

  float voltage = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6 * VOLTAGE_CALIBRATION;
  return voltage;
}
// -----------------------------------

void batteryProt()
{
#if defined BATTERY_PROTECTION
  static unsigned long lastBatteryTime;
  if (millis() - lastBatteryTime > 300)
  { // Check battery voltage every 300ms
    lastBatteryTime = millis();
    batteryVoltage = batteryVolts(); // Store voltage in global variable (also used in dashboard)
    if (batteryVoltage < batteryCutoffvoltage)
    {
      Serial.printf("Battery protection triggered, slowing down! Battery: %.2f V Threshold: %.2f V \n", batteryVoltage, batteryCutoffvoltage);
      Serial.printf("Disconnect battery to prevent it from overdischarging!\n", batteryVoltage, batteryCutoffvoltage);
      batteryProtection = true;
    }
    if (batteryVoltage > batteryCutoffvoltage + (FULLY_CHARGED_VOLTAGE * numberOfCells))
    { // Recovery hysteresis
      batteryProtection = false;
    }
  }
#endif
}

//
// =======================================================================================================
// WEB INTERFACE
// =======================================================================================================
//

#include "src/webInterface.h" // Configuration website

//
// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//

void loop()
{

  switchDetect();
  mcpwmOutput();
  led();
  webInterface();
  batteryProt();
}
