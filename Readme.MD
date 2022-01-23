FiatMon is a device which takes certain CAN messages from Fiat 500L CAN network using OBD2 connector and displays them on the LCD/OLED screen. Most CAN messages are captured from a broadcast stream, but few are requested/received.

Current version of FialMon is based on STM32F103C8T6 MCU on a BluePill board and GMG12864-06D monochrome 128x64 LCD screen with ST7565 controller. 

STM32CubeIDE is used as a develoment tool.

Now FiatMon captures and displays the following parameters:
- speed
- engine coolant temperature, with alarm on values higher than 85 C
- instant fuel consumption 
- fuel consumption averaged by 30 seconds
- handbrake activation
- marker lights when turned on change the display backlit brightness

FiatMon v. 0.4
- implementation of CANsniffer protocol

FiatMon v. 0.3
- screen items implemented as classes
- added a queue for CAN messages

FiatMon v. 0.2

- added support of a variable width fonts
- added alignment of text to the right
- medium and big fonts changed to Arial