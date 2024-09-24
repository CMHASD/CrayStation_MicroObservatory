# Charging and Power

## Purpose

We aim to power the CrayStation Micro Observatory from Solar Panels, charging during the day and running off batteries at night.
Ideal outcome is we can run every night all year - but we think this might not be pratical. The key here is careful consideration to parts. We have to think about idle current, wake up modes and running current.

## Some Stats

_(this will change as we drill down onto the final design) values are estimates ultimatly we need to measure)_

| Board | Idle (A) | Power Consumption (W) | Tested/Researched |
| - | :- | :- | -: |
| Bare Raspberry Pi 4B | 600mA 5v| 3W | looked up |
| | 2A (max) 5v |10W | looked up |
| Pico W (wifi connected) | 50mA 5v|0.25W | looked up |
| Seestar S50 | 1-1.5A 12v?| 12-18 ? W |Based on battery capacity 6AH and quoted battery life |
| | 2A @ 5V | 10W | Charge 5V|
| | 3A @12V | 36W | Rapid Charge 12V |


## SeeStar s50 Battery Notes

- charging stops when the temperature of battery is below 0℃.
- Seestar S50 supports USB BC1.2 of adapters. Type-C cable which is the normal charging mode, supports 5V 2A; The maximum support is 12V 3A with the fast charge cable which is the fast charging mode.
- Seestar S50 battery has a rated capacity of 6000mAh, and the battery life verified by ZWO laboratory is about 6 hours.
- If the dew heater is turned on, the battery life will be significantly shortened.
- The Seestar S50 comes with a lithium battery capable of over 700 charge and discharge cycles.
- Seestar S50 supports physical buttons to turn on and off. In operation, press and hold the power button for 2s to shut down normally, or for 6s to forc shut down.
