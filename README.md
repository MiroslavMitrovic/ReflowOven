# ReflowOven
Reflow Oven powered by STM32F407VGT6 MCU. Custom bootloader implemented so that application can be flashed via USB. 
### Implementation 
- Temperature control is controlled via PID controller from ARM CMSIS library. Due to that that Temperature change is a very slow process, most efficient control was P control, and therefore it is only used from the whole PID controller.
-  As a temperature sensor it is used K-Type Thermocouple with MAX 6675 converter. 
-  Heaters are controlled as two separate banks with an option to hardcode the percentage of power for each bank separately. AC Power control for heaters is achieved via the Zero Crossing method.
-  As a user interface, Nextion Display is used with implementation from Victor Vano that was modified according to the custom needs-Different display was used.(https://www.instructables.com/DIY-REFLOW-OVEN/)
-   As a notification for the finished process buzzer is also implemented, code for songs was from the Arduino library- (http://www.linuxcircle.com/2013/03/31/playing-mario-bros-tune-with-arduino-and-piezo-buzzer/)
-   Bootloader was implemented from Victor Vano repository(https://github.com/viktorvano/STM32-Bootloader) and modified to custom needs. Only one app is supported and adaptation to the current MCU is performed. 

#### Note 
Bootloader implementation can be found on the master branch. 
