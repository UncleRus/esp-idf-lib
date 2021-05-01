# LED effects example

This example works with a 16x16 matrix of WS2812b LEDs (256 pieces in total):

Connect DIN input of matrix to GPIO 5 of ESP32.

Keep in mind that the power consumption of so many LEDs can be overwhelming!
Use a 5V power supply with high output current to power the matrix or lower
`LED_BRIGHTNESS`.

Example uses `frambuffer` component to render some effects.
