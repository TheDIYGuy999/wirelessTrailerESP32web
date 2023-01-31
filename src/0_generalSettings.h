#include <Arduino.h>

/* GENERAL SETTINGS ************************************************************************************************
 *  Have a look at the serial monitor in order to check the current settings of your vehicle!
 *  The required login informations for the browser based configuration via 192.186.4.1 can also be found there.
 */

// Debug settings -----------------------------------------------------------------------------------------------
//#define DEBUG // More infos such as EEPROM dump on serial monitor, if defined

// EEPROM settings ----------------------------------------------------------------------------------------------
uint8_t eeprom_id = 2; // change this id (between 1 and 255), if you want to restore EEPROM defaults (only executed 1x) <<------------- NOTE!
//#define ERASE_EEPROM_ON_BOOT // EEPROM is completely overwritten, if defined! Never define it, trailer will not work!
// only define it in order to clean up junk from old projects in your EEPROM

// WiFi settings (for vehicle configuration website, open 192.168.4.1 in your browser)----------------------------
// Note: if these credentials were changed, using the configuration website, 
// you can find the current ones in the serial monitor!
String default_ssid = "My_Trailer"; // Select this network
String default_password = "123456789"; // Then enter this password

// Configuration website settings --------------------------------------------------------------------------------
#define USE_CSS // Simple Website style is used, if not defined

// ESP-Now settings (for wireless communication with your tractor) -----------------------------------------------
// Select, wether or not you want to use your own trailer MAC address below
// otherwise, the hardware MAC address is used
bool DefaultUseCustomMac = false; // true = use custom mac

// Custom MAC address for communication with tractor:
// Only addresses with low multicast bit are valid: http://sqa.fyicenter.com/1000208_MAC_Address_Validator.html
// This means, that the least significant bit of the first byte from the left always needs to be zero!
// Also, the second last significant bit of this byte should  always be high = locally administrated.
// So, it is recommended to use 0xFE for the first byte from the left.
// Use hex calculator for verification!
// This results in the following numbering schematic recommendation:
// Always FE : Country number (phone country code) : Region number (phone area code) : User Number 1 : User Number 2 : Trailer Number
uint8_t defaultCustomMACAddress[] = {0xFE, 0x00, 0x00, 0x00, 0x00, 0x01};
// Please use your own addresses, otherwise you could get in trouble and your trailer is suddenly controlled by someone else! <<------------- NOTE!

// Trailer settings ----------------------------------------------------------------------------------------------
bool defaulFifthWhweelDetectionActive = false; // 5th wheel detection switch active or not
uint8_t defaultLightsBrightness = 100; // Led lights default brightness

// Power settings ------------------------------------------------------------------------------------------------
#define LOW_POWER_MODE // Uncommenting this will lower the clock to 80MHz and will increase battery life
