
/* Battery low discharge protection (only for boards with voltage divider resistors):
 *  IMPORTANT: Enter used resistor values in Ohms (Ω) and THEN adjust DIODE_DROP, until your readings match the actual battery voltage! */
//#define BATTERY_PROTECTION // This will disable the ESC output, if the battery cutout voltage is reached. 2 fast flashes = battery error!
const float CUTOFF_VOLTAGE = 3.3;        // Usually 3.3 V per LiPo cell. NEVER below 3.2 V!
const float FULLY_CHARGED_VOLTAGE = 4.2; // Usually 4.2 V per LiPo cell, NEVER above!
const float RECOVERY_HYSTERESIS = 0.2;   // around 0.2 V
/* Note on resistor values: These values will be used to calculate the actual ratio between these two resistors (which is also called a "voltage divider").
 * When selecting resistors, always use two of the same magnitude: Like, for example, 10k/2k, 20k/4k or 100k/20k. NEVER exceed a ratio LOWER than (4:1 = 4)!
 * WARNING: If the ratio is too LOW, like 10k/5k (2:1 = 2), the battery voltage will most likely DAMAGE the controller permanently!
 * Example calculation: 2000 / (2000 + 10000) = 0.166 666 666 7; 7.4 V * 0.167 = 1.2358 V (of 3.3 V maximum on GPIO Pin). */
uint32_t RESISTOR_TO_BATTTERY_PLUS = 9400; // Value in Ohms (Ω), for example 10000
uint32_t RESISTOR_TO_GND = 1000;           // Value in Ohms (Ω), for example 2000. Measuring exact resistor values before soldering, if possible is recommended!
float DIODE_DROP = 0;                   // Fine adjust measured value and/or consider diode voltage drop (about 0.34V for SS34 diode)
/* It is recommended to add a sticker to your ESP32, which includes the 3 calibration values above */