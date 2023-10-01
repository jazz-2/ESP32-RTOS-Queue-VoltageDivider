 [__*`github.com/jazz-2`*__](https://github.com/jazz-2)
# ESP32 with FreeRTOS Queue and Voltage Divider

### Demonstration 
Changing the ambient brightness changes the brightness of the LED

![photoresistor](https://github.com/jazz-2/ESP32-RTOS-Queue-VoltageDivider/assets/141406828/6219aa22-03cf-47a5-a9e8-4c1eb58936ba)

--------------

#### Description:
Using ESP32 with Arduino Framework the ***FreeRTOS Queue*** pass `struct Sensor{};` data between the tasks. Choose the amount of samples *`samplesAmount`* and read the ADC data from the voltage divider with a photoresistor. Make calculations and control the LED with PWM. Calculate the desired resistors ratio values for the voltage divider with **<ins>VoltageDividerCalc.ods</ins>**.

