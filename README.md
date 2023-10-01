 [__*`github.com/jazz-2`*__](https://github.com/jazz-2)
# ESP32 with FreeRTOS Queue and Voltage Divider

### Demonstration 
Changing the ambient brightness changes the brightness of the LED

![photoresistor](https://github.com/jazz-2/ESP32-RTOS-Queue-VoltageDivider/assets/141406828/d5b53e0c-5c34-456c-a38c-f981f21b1fae)

--------------

#### Description:
Using ESP32 with Arduino Framework the ***FreeRTOS Queue*** pass `struct Sensor{};` data between the tasks. Choose the amount of samples *`samplesAmount`* and read the ADC data from the voltage divider with a photoresistor. Make calculations and control the LED with PWM. Calculate the desired resistors ratio values for the voltage divider with **<ins>VoltageDividerCalc.ods</ins>**.

