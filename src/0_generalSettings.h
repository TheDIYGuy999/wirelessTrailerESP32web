#include <Arduino.h>

/* GENERAL SETTINGS ************************************************************************************************
 *  
 *  Have a look at the serial monitor in order to check the current settings of your vehicle!
 *  The required login informations for the browser based configuration via 192.186.4.1 can also be found there.
 */

// General settings ----------------------------------------------------------------------------------------------
#define DEBUG // More infos such as EEPROM dump on serial monitor, if defined
uint8_t eeprom_id = 2; // change this id (between 1 and 255), if you want to restore EEPROM defaults (only executed 1x)
//#define ERASE_EEPROM_ON_BOOT // EEPROM is completely overwritten, if defined! Never define it, trailer will not work!
#define USE_CSS // Simple Website style is used, if not defined

// Wireless settings ---------------------------------------------------------------------------------------------

// Wlan settings (open 192.168.4.1 in your browser for configuration)

// Uncomment the following line, if you want to be able to specify your own trailer MAC address below
#define CUSTOM_MAC_ADDRESS // Note, that it still may be disabled in the web configuration!

// Only addresses with low multicast bit are valid: http://sqa.fyicenter.com/1000208_MAC_Address_Validator.html
// This means, that the least significant bit of the first byte from the left always needs to be zero!
// Also, the second last significant bit of this byte should  always be high = locally administrated.
// So, it is recommended to use 0xFE for the first byte from the left.
// Use hex calculator for verification!

// You can't change the mac address here, do it in the browser!

// Please use your own addresses, otherwise you could get in trouble and your trailer is suddenly controlled by someone else!
uint8_t customMACAddress[] = {0xFE, 0x00, 0x00, 0x00, 0x0, 0x01}; // working. Don't use it, TheDIYGuy999 only!

// Power settings ------------------------------------------------------------------------------------------------
#define LOW_POWER_MODE // Uncommenting this will lower the clock to 80MHz and will increase battery life
