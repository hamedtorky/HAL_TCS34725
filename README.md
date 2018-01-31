# HAL_TCS34725
color sensor 

TCS3472
COLOR LIGHT-TO-DIGITAL CONVERTER
with IR FILTER

Description sensor

The TCS3472 device provides a digital return of red, green, blue (RGB), and clear light sensing values. An IR
blocking filter, integrated on-chip and localized to the color sensing photodiodes, minimizes the IR spectral
component of the incoming light and allows color measurements to be made accurately. The high sensitivity,
wide dynamic range, and IR blocking filter make the TCS3472 an ideal color sensor solution for use under
varying lighting conditions and through attenuating materials.
The TCS3472 color sensor has a wide range of applications including RGB LED backlight control, solid-state
lighting, health/fitness products, industrial process controls and medical diagnostic 
equipment. In addition, the
IR blocking filter enables the TCS3472 to perform ambient light sensing (ALS). Ambient light sensing is widely
used in display-
based products such as cell phones, notebooks, and TVs to sense the lighting environment and
enable automatic display brightness for optimal viewing and power savings. The TCS3472, itself, can enter a
lower-power 
wait state between light sensing measurements to further reduce the average power consumption

test function 
```cpp
  void tcs3472_test( void ) 
  {
    uint16_t r, g, b, c, colorTemp, lux;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    tcs3272_init();

    getRawData(&r, &g, &b, &c);
    colorTemp = calculateColorTemperature(r, g, b);
    lux = calculateLux(r, g, b);



  }
```
