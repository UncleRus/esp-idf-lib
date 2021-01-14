#if !defined(__LED_STRIP_SPI_SK9822_H__)
#define __LED_STRIP_SPI_SK9822_H__

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/* https://cpldcpu.wordpress.com/2016/12/13/sk9822-a-clone-of-the-apa102/
 * https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/
 *
 * A start frame of 32 zero bits (<0x00> <0x00> <0x00> <0x00>)
 *
 * 32 bit LED frames for each LED in the string (<0xE0+brightness> <blue> <green> <red>)
 *
 * A SK9822 reset frame of 32 zero bits (<0x00> <0x00> <0x00> <0x00>)
 *
 * An end frame consisting of at least (n/2) bits of 1, where n is the number
 * of LEDs in the string.
 */
#define LED_STRIP_SPI_FRAME_SK9822_START_SIZE  (4)
#define LED_STRIP_SPI_FRAME_SK9822_LED_SIZE    (4)
#define LED_STRIP_SPI_FRAME_SK9822_LEDS_SIZE(N_PIXEL) (LED_STRIP_SPI_FRAME_SK9822_LED_SIZE * N_PIXEL)
#define LED_STRIP_SPI_FRAME_SK9822_RESET_SIZE  (4)
#define LED_STRIP_SPI_FRAME_SK9822_END_SIZE(N_PIXLE) ((N_PIXLE / 16) + 1)

#define LED_STRIP_SPI_FRAME_SK9822_LED_MSB3    (0xE0) // A magic number of [31:29] in LED frames. The bits must be 1 (APA102, SK9822)

/*
 * A macro to caliculate required size of buffer
 * N_PIXEL: number of the pixel in the strip
 */
#define LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL) (\
        LED_STRIP_SPI_FRAME_SK9822_START_SIZE + \
        LED_STRIP_SPI_FRAME_SK9822_LEDS_SIZE(N_PIXEL)  + \
        LED_STRIP_SPI_FRAME_SK9822_RESET_SIZE + \
        LED_STRIP_SPI_FRAME_SK9822_END_SIZE(N_PIXEL))

#ifdef __cplusplus
}
#endif

#endif
