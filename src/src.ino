/* Trailer for RC engine sound & LED controller for Arduino ESP32. Written by TheDIYGuy999


*/

char codeVersion[] = "1.0.0 beta"; // Software revision.

//
// =======================================================================================================
// ! ! I M P O R T A N T ! !   ALL USER SETTINGS ARE DONE IN THE FOLLOWING TABS, WHICH ARE DISPLAYED ABOVE
// (ADJUST THEM BEFORE CODE UPLOAD), DO NOT CHANGE ANYTHING IN THIS TAB EXCEPT THE DEBUG OPTIONS
// =======================================================================================================
//

// All the required user settings are done in the following .h files:
#include "0_generalSettings.h"          // <<------- general settings
#include "7_adjustmentsServos.h"        // <<------- Servo output related adjustments

//
// =======================================================================================================
// LIRBARIES & HEADER FILES, REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder.
// No manual library download is required in Visual Studio Code IDE (see platformio.ini)
#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control

// No need to install these, they come with the ESP32 board definition
#include "driver/mcpwm.h"  // for servo PWM output
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Esp.h>           // for displaying memory information
#include "rom/rtc.h"       // for displaying reset reason
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

#define TAILLIGHT_PIN 15 // Red tail- & brake-lights (combined)
#define INDICATOR_LEFT_PIN 2 // Orange left indicator (turn signal) light
#define INDICATOR_RIGHT_PIN 4 // Orange right indicator (turn signal) light
#define REVERSING_LIGHT_PIN 17 // (TX2) White reversing light
#define SIDELIGHT_PIN 18 // Side lights

#define SERVO_1_PIN 13 // Servo CH1 legs
#define SERVO_2_PIN 12 // Servo CH2 ramps
#define SERVO_3_PIN 14 // Servo CH3 beacon control
#define SERVO_4_PIN 27 // Servo CH4 spare servo channel

#define FIFTH_WHEEL_DETECTION_PIN 32 // This NO switch is closed, if the trailer is coupled to the 5th wheel. Between Pin 32 and GND

// Objects *************************************************************************************
// Status LED objects -----
statusLED tailLight(false); // "false" = output not inversed
statusLED indicatorL(false);
statusLED indicatorR(false);
statusLED reversingLight(false);
statusLED sideLight(false);

// Global variables ****************************************************************************

esp_now_peer_info_t peerInfo; // This MUST be global!! Transmission is not working otherwise!

typedef struct struct_message { // This is the data packet
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

// Webserver on port 80
WiFiServer server(80);

// HTTP request memory
String header;

// For HTTP GET value
String valueString = "";
int pos1 = 0;
int pos2 = 0;

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

#define adr_eprom_init 0                          // Eeprom initialized or not?
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
#define adr_eprom_ssid 64  //64
#define adr_eprom_password 128 //128

//
// =======================================================================================================
// ESP NOW TRAILER DATA RECEIVED CALLBACK
// =======================================================================================================
//

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&trailerData, incomingData, sizeof(trailerData));

  //led(); // This is now handled in loop

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

void setupMcpwm() {
  // 1. set our servo output pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_1_PIN);    //Set legs as PWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_2_PIN);    //Set ramps as PWM0B
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, SERVO_4_PIN);    //Set beacon as PWM1A
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, SERVO_3_PIN);    //Set spare servo as PWM1B

  // 2. configure MCPWM parameters
  mcpwm_config_t pwm_config;
  pwm_config.frequency = SERVO_FREQUENCY;     //frequency usually = 50Hz, some servos may run smoother @ 100Hz
  pwm_config.cmpr_a = 0;                      //duty cycle of PWMxa = 0
  pwm_config.cmpr_b = 0;                      //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;   // 0 = not inverted, 1 = inverted

  // 3. configure channels with settings above
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B
}

//
// =======================================================================================================
// ESP NOW SETUP FOR WIRELESS TRAILER CONTROL & WIFI SETUP FOR WEB CONFIGURATION
// =======================================================================================================
//

void setupEspNow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set IP address
  IPAddress IP = WiFi.softAPIP();

  // Set custom MAC address
#if defined CUSTOM_MAC_ADDRESS
  if (useCustomMac) {
    esp_wifi_set_mac(WIFI_IF_STA, &customMACAddress[0]);
    //esp_wifi_set_mac(ESP_IF_WIFI_STA, &customMACAddress[0]); // Board before 1.0.5
  }
#endif

  // Print MAC address (this is the required MAC address in the transmitter)
  Serial.printf("\nInformations for communication with tractor ******************************************************\n");
  Serial.print("Currently used MAC address (add it in '10_adjustmentsTrailer.h' in the sound controller): ");
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
  WiFi.setTxPower (WIFI_POWER_MINUS_1dBm);

  //Serial.println(ssid.c_str());
  //Serial.println(password.c_str());
  WiFi.softAP(ssid.c_str(), password.c_str());
  //WiFi.softAP("My_Trailer", "1233456789");

  server.begin();  // Start Webserver

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

//
// =======================================================================================================
// EEPROM SETUP
// =======================================================================================================
//

void setupEeprom() {
  EEPROM.begin(EEPROM_SIZE);
#if defined  ERASE_EEPROM_ON_BOOT
  eepromErase(); // uncomment this option, if you want to erase all stored settings!
# endif
  eepromInit(); // Init new board with default values
  eepromRead(); // Read settings from Eeprom
  eepromDebugRead(); // Shows content of entire eeprom, except of empty areas
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

#if defined LOW_POWER_MODE
  setCpuFrequencyMhz(80); // Lower CPU clock frequency to 80MHz to save energy
#endif

  // Serial setup
  Serial.begin(115200); // USB serial (for DEBUG)

  // Print some system and software info to serial monitor
  delay(1000); // Give serial port/connection some time to get ready
  Serial.printf("\n**************************************************************************************************\n");
  Serial.printf("Wireless trailer for ESP32 software version %s\n", codeVersion);
  Serial.printf("https://github.com/TheDIYGuy999\n");
  Serial.printf("Use it in combination with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("XTAL Frequency: %i MHz, CPU Clock: %i MHz, APB Bus Clock: %i Hz\n", getXtalFrequencyMhz(), getCpuFrequencyMhz(), getApbFrequency());
  Serial.printf("Internal RAM size: %i Byte, Free: %i Byte\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("WiFi MAC address: %s\n", WiFi.macAddress().c_str());
  for (uint8_t coreNum = 0; coreNum < 2; coreNum++) {
    uint8_t resetReason = rtc_get_reset_reason(coreNum);
    if (resetReason <= (sizeof(RESET_REASONS) / sizeof(RESET_REASONS[0]))) {
      Serial.printf("Core %i reset reason: %i: %s\n", coreNum, rtc_get_reset_reason(coreNum), RESET_REASONS[resetReason - 1]);
    }
  }

  Serial.printf("**************************************************************************************************\n\n");

  //Eeprom setup
  setupEeprom();

  // Pin setup
  pinMode(FIFTH_WHEEL_DETECTION_PIN, INPUT_PULLUP);

  // ESP NOW setup
  setupEspNow();

  // LED setup (note, that we only have timers from 0 - 15)
  tailLight.begin(TAILLIGHT_PIN, 2, 20000); // Timer 2, 20kHz
  sideLight.begin(SIDELIGHT_PIN, 3, 20000); // Timer 3, 20kHz
  reversingLight.begin(REVERSING_LIGHT_PIN, 4, 20000); // Timer 4, 20kHz
  indicatorL.begin(INDICATOR_LEFT_PIN, 5, 20000); // Timer 5, 20kHz
  indicatorR.begin(INDICATOR_RIGHT_PIN, 6, 20000); // Timer 6, 20kHz

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

void switchDetect() {
  static unsigned long switchMillis;

  if (!digitalRead(FIFTH_WHEEL_DETECTION_PIN) || !fifthWhweelDetectionActive) { // only read switch, if enabled
    switchMillis = millis(); // if coupled
    //Serial.println("coupled");
  }

  trailerCoupled = (millis() - switchMillis <= 1000); // 1s delay, if not coupled
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

void led() {

  if (trailerCoupled) {
    tailLight.pwm(trailerData.tailLight * tailLightBrightness / 100);
    sideLight.pwm(trailerData.sideLight * sideLightBrightness / 100);
    reversingLight.pwm(trailerData.reversingLight * reversingLightBrightness / 100);
    indicatorL.pwm(trailerData.indicatorL * indicatorLightBrightness / 100);
    indicatorR.pwm(trailerData.indicatorR * indicatorLightBrightness / 100);
  }
  else {
    tailLight.pwm(0);
    sideLight.pwm(0);
    reversingLight.pwm(0);
    indicatorL.pwm(0);
    indicatorR.pwm(0);
  }
}

//
// =======================================================================================================
// ROTATING BEACON CONTROL (disconnect USB & battery after upload for correct beacon function)
// =======================================================================================================
//

bool beaconControl(uint8_t pulses) {

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

  if (millis() - pulseMillis > 40) { // Every 40ms (this is the required minimum)
    pulseMillis = millis();
    if (pulseWidth == CH3L) {
      pulseWidth = CH3R;
    }
    else {
      pulseWidth = CH3L;
      i++;
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, pulseWidth);
  }

  if (i >= pulses) {
    i = 0;
    return true;
  }
  else return false;
}

//
// =======================================================================================================
// MCPWM SERVO RC SIGNAL OUTPUT (BUS communication mode only)
// =======================================================================================================
//
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#configure

void mcpwmOutput() {

  // Legs servo CH1 (active, if 5th wheel is unlocked, use horn pot) **********************

#ifdef LEGS_ESC_MODE // ESC mode
  static uint16_t legsServoMicros = CH1L;
  if (trailerData.legsDown) legsServoMicros = CH1L; // down
  else if (trailerData.legsUp) legsServoMicros = CH1R; // up
  else legsServoMicros = CH1C; // off

#else // Servo mode  
  static uint16_t legsServoMicrosTarget = CH1L;
  static uint16_t legsServoMicros = CH1L;
  static unsigned long legsDelayMicros;
  if (micros() - legsDelayMicros > LEGS_RAMP_TIME) {
    legsDelayMicros = micros();
    if (trailerData.legsDown) legsServoMicrosTarget = CH1L; // down
    else if (trailerData.legsUp) legsServoMicrosTarget = CH1R; // up
    else legsServoMicrosTarget = legsServoMicros; // stop
    if (legsServoMicros < legsServoMicrosTarget) legsServoMicros ++;
    if (legsServoMicros > legsServoMicrosTarget) legsServoMicros --;
  }
#endif
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, legsServoMicros);

  // Ramps servo CH2 (active, if hazards are on, use horn pot) *****************************
#ifdef RAMPS_ESC_MODE // ESC mode
  static uint16_t rampsServoMicros = CH2L;
  if (trailerData.rampsDown) rampsServoMicros = CH2L; // down
  else if (trailerData.rampsUp) rampsServoMicros = CH2R; // up
  else rampsServoMicros = CH2C; // off

#else // Servo mode 
  static uint16_t rampsServoMicrosTarget = CH2R;
  static uint16_t rampsServoMicros = CH2R;
  static unsigned long rampsDelayMicros;
  if (micros() - rampsDelayMicros > RAMPS_RAMP_TIME) {
    rampsDelayMicros = micros();
    if (trailerData.rampsDown) rampsServoMicrosTarget = CH2L; // down
    else if (trailerData.rampsUp) rampsServoMicrosTarget = CH2R; // up
    else rampsServoMicrosTarget = rampsServoMicros; // stop
    if (rampsServoMicros < rampsServoMicrosTarget) rampsServoMicros ++;
    if (rampsServoMicros > rampsServoMicrosTarget) rampsServoMicros --;
  }
#endif
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, rampsServoMicros);

  // Beacon control CH3 (use horn / blue light pot)******************************************
  // Init (5 pulses are required to shut beacons off after power on)
  static bool blueLightInit;
  if (!blueLightInit) {
    if (beaconControl(5)) blueLightInit = true;
  }

  // Switching modes
  static uint16_t beaconServoMicros;
  static bool lockRotating, lockOff;
  if (blueLightInit) {
    if (trailerData.beaconsOn && !lockRotating) { // Rotating mode on (1 pulse)
      if (beaconControl(1)) {
        lockRotating = true;
        lockOff = false;
      }
    }
    if (!trailerData.beaconsOn && !lockOff && lockRotating) { // Off (4 pulses)
      if (beaconControl(4)) {
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

int writeStringToEEPROM(int addrOffset, const String &strToWrite) {
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++) {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
  return addrOffset + 1 + len;
}

// Read string from EEPROM ------
int readStringFromEEPROM(int addrOffset, String *strToRead) {
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++) {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';
  *strToRead = String(data);
  return addrOffset + 1 + newStrLen;
}

// Erase EEPROM ------
void eepromErase() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("EEPROM erased!");
  delay(500);
  eepromDebugRead();
}

// Init new board with the default values you want ------
void eepromInit() {
  if (EEPROM.readInt(adr_eprom_init) != eeprom_id) {
    EEPROM.writeInt(adr_eprom_init, eeprom_id);
    EEPROM.writeInt(adr_eprom_useCustomMac, false);
    EEPROM.writeInt(adr_eprom_Mac0, 0xFE); // Should always be 0xFE!
    EEPROM.writeInt(adr_eprom_Mac1, 0x00); // Country number
    EEPROM.writeInt(adr_eprom_Mac2, 0x00); // Region number
    EEPROM.writeInt(adr_eprom_Mac3, 0x00);
    EEPROM.writeInt(adr_eprom_Mac4, 0x00); // User Number
    EEPROM.writeInt(adr_eprom_Mac5, 0x01); // 0x01 = trailer #1
    EEPROM.writeInt(adr_eprom_fifthWhweelDetectionActive, false);
    EEPROM.writeInt(adr_eprom_tailLightBrightness, 100);
    EEPROM.writeInt(adr_eprom_sideLightBrightness, 100);
    EEPROM.writeInt(adr_eprom_reversingLightBrightness, 100);
    EEPROM.writeInt(adr_eprom_indicatorLightBrightness, 100);
    writeStringToEEPROM(adr_eprom_ssid, "My_Trailer");
    writeStringToEEPROM(adr_eprom_password, "123456789");
    EEPROM.commit();
    Serial.println("EEPROM initialized.");
  }
}

// Write new values to EEPROM ------
void eepromWrite() {
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
void eepromRead() {
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
  //readStringFromEEPROM(adr_eprom_ssid, &newStr3);
  readStringFromEEPROM(adr_eprom_ssid, &ssid);
  readStringFromEEPROM(adr_eprom_password, &password);

  Serial.println("EEPROM read.");
}

void eepromDebugRead() {
# if defined DEBUG
  String eepromDebug;
  Serial.println("EEPROM debug dump begin **********************************************");
  eepromDebug = "";
  for (int i = 0; i < EEPROM_SIZE; ++i) {
    eepromDebug += char(EEPROM.read(i));
  }
  Serial.println(eepromDebug);
  Serial.println("");
  Serial.println("EEPROM debug dump end ************************************************");
# endif
}

//
// =======================================================================================================
// WEB INTERFACE
// =======================================================================================================
//

void webInterface() {

  static unsigned long currentTime = millis();   // Current time
  static unsigned long previousTime = 0;         // Previous time
  const long timeoutTime = 2000;          // Define timeout time in milliseconds (example: 2000ms = 2s)

  static bool Mode = false; // TODO



  if (true) {     //Wifi on
    //if (WIFI_ON == 1) {     //Wifi on
    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                             // If a new client connects,
      currentTime = millis();
      previousTime = currentTime;
      Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
        currentTime = millis();
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Display the HTML web page
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");

#if defined USE_CSS
              // CSS styles for buttons
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; background-color: rgb(60, 161, 120);}");
              client.println(".button { border: yes; color: white; padding: 10px 40px; width: 90%;");
              client.println("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer;}");
              client.println(".slider { -webkit-appearance: none; width: 50%; height: 25px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s; }");
              client.println(".button1 {background-color: #4CAF50;}");
              client.println(".button2 {background-color: #ff0000;}");
              client.println(".textbox {font-size: 18px; text-align: center;}");
              client.println("</style></head>");
# endif

              // Website title
              client.println("</head><body><h1>TheDIYGuy999 Wireless Trailer</h1>");

              // Sub title
              //client.println("<h2>Settings</h2>");

              // Settings ------------------------------------------------------------------------------------------------------------

              // Set1 (ssid) ----------------------------------
              valueString = ssid; // Read current value
              //client.println("<p><h3>Trailer SSID: <span id=\"textSetting1Value\">" + valueString + "</span>"); // Display current value
              client.println("<p><h3>SSID: "); // Display current value

              client.println("<input type=\"text\" id=\"Setting1Input\" size=\"25\" class=\"textbox\" oninput=\"Setting1change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting1change(pos) { ");
              //client.println("var sliderValue = document.getElementById(\"Setting1Input\").value;");
              //client.println("document.getElementById(\"textSetting1Value\").innerHTML = sliderValue;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set1=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                Serial.println(valueString);
                //valueString.toCharArray(ssid, 33);
                ssid = valueString;
              }

              // Set2 (password) ----------------------------------
              valueString = password; // Read current value
              client.println("<p><h3>Password (min. 8): "); // Display current value

              client.println("<input type=\"text\" id=\"Setting2Input\" size=\"25\" class=\"textbox\" oninput=\"Setting2change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting2change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set2=") >= 0) {
                pos1 = header.indexOf('='); // Start pos
                pos2 = header.indexOf('&'); // End pos
                valueString = header.substring(pos1 + 1, pos2);
                //valueString.toCharArray(password, 33);
                password = valueString;
                //Serial.println(pos1);
                //Serial.println(pos2);
                //Serial.println(password);
              }

              client.println("<hr>"); // Horizontal line

#if defined CUSTOM_MAC_ADDRESS
              // Checkbox1 (use custom mac) ----------------------------------
              if (useCustomMac == true) {
                client.println("<p><h3><input type=\"checkbox\" id=\"tc\" checked onclick=\"Checkbox1Change(this.checked)\"> use custom MAC (save to EEPROM & reboot required) </input></p>");
              }
              else {
                client.println("<p><input type=\"checkbox\" id=\"tc\" unchecked onclick=\"Checkbox1Change(this.checked)\"> use custom MAC (save to EEPROM & reboot required) </input></p>");
              }
              client.println("<script> function Checkbox1Change(pos) { ");
              //client.println("ChangeCheckboxLabel(document.getElementById(\"tc\"));");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Checkbox1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Checkbox1=true") >= 0) {
                useCustomMac = true;
                Serial.println("use custom MAC (save to EEPROM & reboot required");
              }
              else if (header.indexOf("GET /?Checkbox1=false") >= 0) {
                useCustomMac = false;
                Serial.println("don't use custom MAC (save to EEPROM & reboot required");
              }





              // Set3 (MAC0) ----------------------------------
              valueString = String(customMACAddress[0], HEX); // Read current value
              client.println("<p><h3>Custom MAC: "); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting3change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting3change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set3=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set3=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[0] = strtol(valueString.c_str(), NULL, 16);
              }


              // Set4 (MAC1) ----------------------------------
              valueString = String(customMACAddress[1], HEX); // Read current value
              client.println(":"); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting4change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting4change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set4=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set4=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[1] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set5 (MAC2) ----------------------------------
              valueString = String(customMACAddress[2], HEX); // Read current value
              client.println(":"); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting5change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting5change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set5=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set5=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[2] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set6 (MAC3) ----------------------------------
              valueString = String(customMACAddress[3], HEX); // Read current value
              client.println(":"); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting6change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting6change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set6=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set6=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[3] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set7 (MAC4) ----------------------------------
              valueString = String(customMACAddress[4], HEX); // Read current value
              client.println(":"); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting7change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting7change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set7=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set7=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[4] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set8 (MAC5) ----------------------------------
              valueString = String(customMACAddress[5], HEX); // Read current value
              client.println(":"); // Display title

              client.println("<input type=\"text\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting8change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting8change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set8=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set8=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[5] = strtol(valueString.c_str(), NULL, 16);
              }
#endif

              // Currently uses MAC address info ----------------------------------
              client.print("Currently used MAC address: ");
              client.println(WiFi.macAddress());

              client.println("<hr>"); // Horizontal line

              // Checkbox2 (5th wheel setting) ----------------------------------
              if (fifthWhweelDetectionActive == true) {
                client.println("<p><h3><input type=\"checkbox\" id=\"tc\" checked onclick=\"Checkbox2Change(this.checked)\"> 5th wheel detection switch active = lights off, if not coupled </input></p>");
              }
              else {
                client.println("<p><input type=\"checkbox\" id=\"tc\" unchecked onclick=\"Checkbox2Change(this.checked)\"> 5th wheel detection switch active = lights off, if not coupled </input></p>");
              }
              client.println("<script> function Checkbox2Change(pos) { ");
              //client.println("ChangeCheckboxLabel(document.getElementById(\"tc\"));");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Checkbox2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Checkbox2=true") >= 0) {
                fifthWhweelDetectionActive = true;
                Serial.println("5th wheel detection switch active");
              }
              else if (header.indexOf("GET /?Checkbox2=false") >= 0) {
                fifthWhweelDetectionActive = false;
                Serial.println("5th wheel detection switch inactive");
              }

              // Slider1 (taillight brightness) ----------------------------------
              valueString = String(tailLightBrightness, DEC);
              client.println("<p><h3>Taillight brightness % : <span id=\"textSlider1Value\">" + valueString + "</span>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider1Input\" onchange=\"Slider1Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider1Change(pos) { ");
              client.println("var slider1Value = document.getElementById(\"Slider1Input\").value;");
              client.println("document.getElementById(\"textSlider1Value\").innerHTML = slider1Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider1=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                tailLightBrightness = (valueString.toInt());
              }

              // Slider2 (sidelight brightness) ----------------------------------
              valueString = String(sideLightBrightness, DEC);
              client.println("<p><h3>Sidelight brightness % : <span id=\"textSlider2Value\">" + valueString + "</span>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider2Input\" onchange=\"Slider2Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider2Change(pos) { ");
              client.println("var slider2Value = document.getElementById(\"Slider2Input\").value;");
              client.println("document.getElementById(\"textSlider2Value\").innerHTML = slider2Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider2=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                sideLightBrightness = (valueString.toInt());
              }

              // Slider3 (reversing light brightness) ----------------------------------
              valueString = String(reversingLightBrightness, DEC);
              client.println("<p><h3>Reversing light brightness % : <span id=\"textSlider3Value\">" + valueString + "</span>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider3Input\" onchange=\"Slider3Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider3Change(pos) { ");
              client.println("var slider3Value = document.getElementById(\"Slider3Input\").value;");
              client.println("document.getElementById(\"textSlider3Value\").innerHTML = slider3Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider3=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider3=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                reversingLightBrightness = (valueString.toInt());
              }

              // Slider4 (indicator light brightness) ----------------------------------
              valueString = String(indicatorLightBrightness, DEC);
              client.println("<p><h3>Indicator light brightness % : <span id=\"textSlider4Value\">" + valueString + "</span>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider4Input\" onchange=\"Slider4Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider4Change(pos) { ");
              client.println("var slider4Value = document.getElementById(\"Slider4Input\").value;");
              client.println("document.getElementById(\"textSlider4Value\").innerHTML = slider4Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider4=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider4=") >= 0) {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                indicatorLightBrightness = (valueString.toInt());
              }

              client.println("<hr>"); // Horizontal line

              // button1 (Save settings to EEPROM) ----------------------------------
              client.println("<p><a href=\"/save/on\"><button class=\"button button1\">Save settings to EEPROM</button></a></p>");

              if (header.indexOf("GET /save/on") >= 0)
              {
                eepromWrite();
              }

              // button2 (Reboot controller) ----------------------------------
              client.println("<p><a href=\"/reboot/on\"><button class=\"button button2\">Reboot controller (better use reset switch)</button></a></p>");

              if (header.indexOf("GET /reboot/on") >= 0)
              {
                ESP.restart();
              }

              //-----------------------------------------------------------------------------------------------------------------------
              client.println("</body></html>");

              // The HTTP-response ends with an empty column
              client.println();
              // Break out of the while loop
              break;
            } else { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        }
      }
      // Clear header
      header = "";
      // Disconnect client
      client.stop();
      Serial.println("Client disconnected.");

      Serial.print("5th wheel detection active: ");
      Serial.println(fifthWhweelDetectionActive);
      Serial.println("");
    }
  }
}

//
// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//

void loop() {

  switchDetect();
  mcpwmOutput();
  led();
  webInterface();
}
