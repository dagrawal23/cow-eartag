GPS Tracking eartag for cows designed for Dorea Lab at UW-Madison
HARDWARE-
UBLOXm10S GPS Module
SD-Card Slot
ESP32-C6
2N2222 BJT

Uses the proprietory UBX protocol to configure and interface with the UBLOX module over I2C. Goes into deep-sleep mode after configuring the GPS Module and initializing the SD card. Polls the GPS reciever using the ULP Coprocessor on the ESP Chip using the LP-I2C peripheral. Once a fix is found, the main CPU is woken up to store the readings on the SD card. Upon successfully storing the readings, both the CPUs sleep for a fixed-configurable period and the GPS module is also turned off to save power. After the fixed period has elasped, the ULP CPU wakes up again and the cycle goes on....
