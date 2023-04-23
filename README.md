# CS-350: Emerging Systems Architecture & Technology

## Morse Code

The Morse Code directory contains a project that utilizes the CC3220S LaunchXL Development kit to blink the green, yellow, and red LEDs in the lower right corner of the board. The project implements a synchronous state machine that creates a pattern of blinking lights from the LEDs, which contain a message in Morse code. When a button is pressed, the Morse code message of the blinking LEDs changes. The initial message is "SOS", and when the button is pressed, it will change to "OK" before replaying the "SOS" message. The alternative message will not display until the current message is complete.

## Thermostat

The Thermostat directory contains a project that utilizes the TMP006 temperature sensor on the CC3220S LaunchXL Development kit to read the room temperature via I2C. It also uses an LED to indicate the output to the thermostat where LED on = heat on via GPIO. Two buttons are used to increase and decrease the set temperature via GPIO interrupt. Additionally, the UART is used to simulate the data being sent to the server.

## Reflection

I believe that I did particularly well with analyzing the project requirements, coming up with a plan via state machine diagrams, and implementing this in my code. Futhermore, I am satisified with the in-depth code comments that I utilized during the development process. With that being said, If I had to choose an area where I could improve, I would like to condense my comments so that they aren't convoluted or repetitive. Throughout these projects, I utilized many resources, such as technical manuals from TI for the CC3220S LaunchXL Development kit, and videos on Embedded C programming. In the future, I am confident that this introduction to Embedded C programming will be a helpful path towards learning more advanced implementations in embedded systems, and I am open to working with other low-level hardware interfaces.
