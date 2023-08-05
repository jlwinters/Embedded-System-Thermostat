# Embedded System Thermostat

## Morse Code

The Morse Code directory contains a project that utilizes the CC3220S LaunchXL Development kit to blink the green, yellow, and red LEDs in the lower right corner of the board. The project implements a synchronous state machine that creates a pattern of blinking lights from the LEDs, which contain a message in Morse code. When a button is pressed, the Morse code message of the blinking LEDs changes. The initial message is "SOS", and when the button is pressed, it will change to "OK" before replaying the "SOS" message. The alternative message will not display until the current message is complete.

## Thermostat

The Thermostat directory contains a project that utilizes the TMP006 temperature sensor on the CC3220S LaunchXL Development kit to read the room temperature via I2C. It also uses an LED to indicate the output to the thermostat where LED on = heat on via GPIO. Two buttons are used to increase and decrease the set temperature via GPIO interrupt. Additionally, the UART is used to simulate the data being sent to the server.
