/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 FastLED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "color.h"
#include <math.h>
#include <lib8tion.h>

////////////////////////////////////////////////////////////////////////////////

#define APPLY_DIMMING(X) (X)
#define HSV_SECTION_6 (0x20)
#define HSV_SECTION_3 (0x40)

rgb_t hsv2rgb_raw(hsv_t hsv)
{
    // Convert hue, saturation and brightness ( HSV/HSB ) to RGB
    // "Dimming" is used on saturation and brightness to make
    // the output more visually linear.

    // Apply dimming curves
    uint8_t value = APPLY_DIMMING(hsv.val);
    uint8_t saturation = hsv.sat;

    // The brightness floor is minimum number that all of
    // R, G, and B will be set to.
    uint8_t invsat = APPLY_DIMMING(255 - saturation);
    uint8_t brightness_floor = (value * invsat) / 256;

    // The color amplitude is the maximum amount of R, G, and B
    // that will be added on top of the brightness_floor to
    // create the specific hue desired.
    uint8_t color_amplitude = value - brightness_floor;

    // Figure out which section of the hue wheel we're in,
    // and how far offset we are withing that section
    uint8_t section = hsv.hue / HSV_SECTION_3; // 0..2
    uint8_t offset = hsv.hue % HSV_SECTION_3;  // 0..63

    uint8_t rampup = offset; // 0..63
    uint8_t rampdown = (HSV_SECTION_3 - 1) - offset; // 63..0

    // We now scale rampup and rampdown to a 0-255 range -- at least
    // in theory, but here's where architecture-specific decsions
    // come in to play:
    // To scale them up to 0-255, we'd want to multiply by 4.
    // But in the very next step, we multiply the ramps by other
    // values and then divide the resulting product by 256.
    // So which is faster?
    //   ((ramp * 4) * othervalue) / 256
    // or
    //   ((ramp    ) * othervalue) /  64
    // It depends on your processor architecture.
    // On 8-bit AVR, the "/ 256" is just a one-cycle register move,
    // but the "/ 64" might be a multicycle shift process. So on AVR
    // it's faster do multiply the ramp values by four, and then
    // divide by 256.
    // On ARM, the "/ 256" and "/ 64" are one cycle each, so it's
    // faster to NOT multiply the ramp values by four, and just to
    // divide the resulting product by 64 (instead of 256).
    // Moral of the story: trust your profiler, not your insticts.

    // Since there's an AVR assembly version elsewhere, we'll
    // assume what we're on an architecture where any number of
    // bit shifts has roughly the same cost, and we'll remove the
    // redundant math at the source level:

    //  // scale up to 255 range
    //  //rampup *= 4; // 0..252
    //  //rampdown *= 4; // 0..252

    // compute color-amplitude-scaled-down versions of rampup and rampdown
    uint8_t rampup_amp_adj = (rampup * color_amplitude) / (256 / 4);
    uint8_t rampdown_amp_adj = (rampdown * color_amplitude) / (256 / 4);

    // add brightness_floor offset to everything
    uint8_t rampup_adj_with_floor = rampup_amp_adj + brightness_floor;
    uint8_t rampdown_adj_with_floor = rampdown_amp_adj + brightness_floor;

    rgb_t rgb;
    if (section)
    {
        if (section == 1)
        {
            // section 1: 0x40..0x7F
            rgb.r = brightness_floor;
            rgb.g = rampdown_adj_with_floor;
            rgb.b = rampup_adj_with_floor;
        }
        else
        {
            // section 2; 0x80..0xBF
            rgb.r = rampup_adj_with_floor;
            rgb.g = brightness_floor;
            rgb.b = rampdown_adj_with_floor;
        }
    }
    else
    {
        // section 0: 0x00..0x3F
        rgb.r = rampdown_adj_with_floor;
        rgb.g = rampup_adj_with_floor;
        rgb.b = brightness_floor;
    }

    return rgb;
}

rgb_t hsv2rgb_spectrum(hsv_t hsv)
{
    hsv.hue = scale8(hsv.hue, HUE_MAX_RAW);
    return hsv2rgb_raw(hsv);
}

#define K255 255
#define K171 171
#define K170 170
#define K85  85

rgb_t hsv2rgb_rainbow(hsv_t hsv)
{
    // Yellow has a higher inherent brightness than
    // any other color; 'pure' yellow is perceived to
    // be 93% as bright as white.  In order to make
    // yellow appear the correct relative brightness,
    // it has to be rendered brighter than all other
    // colors.
    // Level Y1 is a moderate boost, the default.
    // Level Y2 is a strong boost.
    const uint8_t Y1 = 1;
    const uint8_t Y2 = 0;

    // G2: Whether to divide all greens by two.
    // Depends GREATLY on your particular LEDs
    const uint8_t G2 = 0;

    // Gscale: what to scale green down by.
    // Depends GREATLY on your particular LEDs
    const uint8_t Gscale = 0;


    uint8_t hue = hsv.hue;
    uint8_t sat = hsv.sat;
    uint8_t val = hsv.val;

    uint8_t offset = hue & 0x1F; // 0..31

    // offset8 = offset * 8
    uint8_t offset8 = offset << 3;
    uint8_t third = scale8(offset8, (256 / 3)); // max = 85

    uint8_t r, g, b;

    if (!(hue & 0x80))
    {
        // 0XX
        if (!(hue & 0x40))
        {
            // 00X
            //section 0-1
            if (!(hue & 0x20))
            {
                // 000
                //case 0: // R -> O
                r = K255 - third;
                g = third;
                b = 0;
            }
            else
            {
                // 001
                //case 1: // O -> Y
                if (Y1)
                {
                    r = K171;
                    g = K85 + third;
                    b = 0;
                }
                if (Y2)
                {
                    r = K170 + third;
                    //uint8_t twothirds = (third << 1);
                    uint8_t twothirds = scale8(offset8, ((256 * 2) / 3)); // max=170
                    g = K85 + twothirds;
                    b = 0;
                }
            }
        }
        else
        {
            //01X
            // section 2-3
            if (!(hue & 0x20))
            {
                // 010
                //case 2: // Y -> G
                if (Y1)
                {
                    //uint8_t twothirds = (third << 1);
                    uint8_t twothirds = scale8(offset8, ((256 * 2) / 3)); // max=170
                    r = K171 - twothirds;
                    g = K170 + third;
                    b = 0;
                }
                if (Y2)
                {
                    r = K255 - offset8;
                    g = K255;
                    b = 0;
                }
            }
            else
            {
                // 011
                // case 3: // G -> A
                r = 0;
                g = K255 - third;
                b = third;
            }
        }
    }
    else
    {
        // section 4-7
        // 1XX
        if (!(hue & 0x40))
        {
            // 10X
            if (!(hue & 0x20))
            {
                // 100
                //case 4: // A -> B
                r = 0;
                //uint8_t twothirds = (third << 1);
                uint8_t twothirds = scale8(offset8, ((256 * 2) / 3)); // max=170
                g = K171 - twothirds; //K170?
                b = K85 + twothirds;

            }
            else
            {
                // 101
                //case 5: // B -> P
                r = third;
                g = 0;
                b = K255 - third;

            }
        }
        else
        {
            if (!(hue & 0x20))
            {
                // 110
                //case 6: // P -- K
                r = K85 + third;
                g = 0;
                b = K171 - third;

            }
            else
            {
                // 111
                //case 7: // K -> R
                r = K170 + third;
                g = 0;
                b = K85 - third;

            }
        }
    }

    // This is one of the good places to scale the green down,
    // although the client can scale green down as well.
    if (G2)
        g = g >> 1;
    if (Gscale)
        g = scale8_video(g, Gscale);

    // Scale down colors if we're desaturated at all
    // and add the brightness_floor to r, g, and b.
    if (sat != 255)
    {
        if (sat == 0)
        {
            r = 255;
            b = 255;
            g = 255;
        }
        else
        {
            uint8_t desat = 255 - sat;
            desat = scale8_video(desat, desat);

            uint8_t satscale = 255 - desat;
            //satscale = sat; // uncomment to revert to pre-2021 saturation behavior

            //nscale8x3_video( r, g, b, sat);
            r = scale8(r, satscale);
            g = scale8(g, satscale);
            b = scale8(b, satscale);

            uint8_t brightness_floor = desat;
            r += brightness_floor;
            g += brightness_floor;
            b += brightness_floor;
        }
    }

    // Now scale everything down if we're at value < 255.
    if (val != 255)
    {

        val = scale8_video(val, val);
        if (val == 0)
        {
            r = 0;
            g = 0;
            b = 0;
        }
        else
        {
            // nscale8x3_video( r, g, b, val);
            r = scale8(r, val);
            g = scale8(g, val);
            b = scale8(b, val);
        }
    }

    return rgb_from_values(r, g, b);
}

#define FIXFRAC8(N,D) (((N) * 256) / (D))

// This function is only an approximation, and it is not
// nearly as fast as the normal HSV-to-RGB conversion.
// See extended notes in the .h file.
hsv_t rgb2hsv_approximate(rgb_t rgb)
{
    uint8_t r = rgb.r;
    uint8_t g = rgb.g;
    uint8_t b = rgb.b;
    uint8_t h, s, v;

    // find desaturation
    uint8_t desat = 255;
    if (r < desat) desat = r;
    if (g < desat) desat = g;
    if (b < desat) desat = b;

    // remove saturation from all channels
    r -= desat;
    g -= desat;
    b -= desat;

    s = 255 - desat;
    if (s != 255)
    {
        // undo 'dimming' of saturation
        s = 255 - sqrt16((255 - s) * 256);
    }
    // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
    // if( s != 255 ) s = (255 - (256.0 * sqrt( (float)(255-s) / 256.0)));

    // at least one channel is now zero
    // if all three channels are zero, we had a
    // shade of gray.
    if ((r + g + b) == 0)
    {
        // we pick hue zero for no special reason
        hsv_t res = {
           .h = 0, .s = 0, .v = 255 - s
        };
        return res;
    }

    // scale all channels up to compensate for desaturation
    if (s < 255)
    {
        if (s == 0)
            s = 1;
        uint32_t scaleup = 65535 / (s);
        r = ((uint32_t) (r) * scaleup) / 256;
        g = ((uint32_t) (g) * scaleup) / 256;
        b = ((uint32_t) (b) * scaleup) / 256;
    }

    uint16_t total = r + g + b;

    // scale all channels up to compensate for low values
    if (total < 255)
    {
        if (total == 0)
            total = 1;
        uint32_t scaleup = 65535 / (total);
        r = ((uint32_t) (r) * scaleup) / 256;
        g = ((uint32_t) (g) * scaleup) / 256;
        b = ((uint32_t) (b) * scaleup) / 256;
    }

    if (total > 255)
    {
        v = 255;
    }
    else
    {
        v = qadd8(desat, total);
        // undo 'dimming' of brightness
        if (v != 255)
            v = sqrt16(v * 256);
        // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
        // if( v != 255) v = (256.0 * sqrt( (float)(v) / 256.0));
    }

    // since this wasn't a pure shade of gray,
    // the interesting question is what hue is it

    // start with which channel is highest
    // (ties don't matter)
    uint8_t highest = r;
    if (g > highest) highest = g;
    if (b > highest) highest = b;

    if (highest == r)
    {
        // Red is highest.
        // Hue could be Purple/Pink-Red,Red-Orange,Orange-Yellow
        if (g == 0)
        {
            // if green is zero, we're in Purple/Pink-Red
            h = (HUE_PURPLE + HUE_PINK) / 2;
            h += scale8(qsub8(r, 128), FIXFRAC8(48, 128));
        }
        else if ((r - g) > g)
        {
            // if R-G > G then we're in Red-Orange
            h = HUE_RED;
            h += scale8(g, FIXFRAC8(32, 85));
        }
        else
        {
            // R-G < G, we're in Orange-Yellow
            h = HUE_ORANGE;
            h += scale8(qsub8((g - 85) + (171 - r), 4), FIXFRAC8(32, 85)); //221
        }

    }
    else if (highest == g)
    {
        // Green is highest
        // Hue could be Yellow-Green, Green-Aqua
        if (b == 0)
        {
            // if Blue is zero, we're in Yellow-Green
            //   G = 171..255
            //   R = 171..  0
            h = HUE_YELLOW;
            uint8_t radj = scale8(qsub8(171, r), 47); //171..0 -> 0..171 -> 0..31
            uint8_t gadj = scale8(qsub8(g, 171), 96); //171..255 -> 0..84 -> 0..31;
            uint8_t rgadj = radj + gadj;
            uint8_t hueadv = rgadj / 2;
            h += hueadv;
            //h += scale8( qadd8( 4, qadd8((g - 128), (128 - r))),
            //             FIXFRAC8(32,255)); //
        }
        else
        {
            // if Blue is nonzero we're in Green-Aqua
            if ((g - b) > b)
            {
                h = HUE_GREEN;
                h += scale8(b, FIXFRAC8(32, 85));
            }
            else
            {
                h = HUE_AQUA;
                h += scale8(qsub8(b, 85), FIXFRAC8(8, 42));
            }
        }

    }
    else /* highest == b */
    {
        // Blue is highest
        // Hue could be Aqua/Blue-Blue, Blue-Purple, Purple-Pink
        if (r == 0)
        {
            // if red is zero, we're in Aqua/Blue-Blue
            h = HUE_AQUA + ((HUE_BLUE - HUE_AQUA) / 4);
            h += scale8(qsub8(b, 128), FIXFRAC8(24, 128));
        }
        else if ((b - r) > r)
        {
            // B-R > R, we're in Blue-Purple
            h = HUE_BLUE;
            h += scale8(r, FIXFRAC8(32, 85));
        }
        else
        {
            // B-R < R, we're in Purple-Pink
            h = HUE_PURPLE;
            h += scale8(qsub8(r, 85), FIXFRAC8(32, 85));
        }
    }

    h += 1;

    return hsv_from_values(h, s, v);
}

////////////////////////////////////////////////////////////////////////////////

rgb_t rgb_heat_color(uint8_t temperature)
{
    rgb_t heatcolor;

    // Scale 'heat' down from 0-255 to 0-191,
    // which can then be easily divided into three
    // equal 'thirds' of 64 units each.
    uint8_t t192 = scale8_video(temperature, 191);

    // calculate a value that ramps up from
    // zero to 255 in each 'third' of the scale.
    uint8_t heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2; // scale up to 0..252

    // now figure out which third of the spectrum we're in:
    if (t192 & 0x80)
    {
        // we're in the hottest third
        heatcolor.r = 255; // full red
        heatcolor.g = 255; // full green
        heatcolor.b = heatramp; // ramp up blue
    }
    else if (t192 & 0x40)
    {
        // we're in the middle third
        heatcolor.r = 255; // full red
        heatcolor.g = heatramp; // ramp up green
        heatcolor.b = 0; // no blue
    }
    else
    {
        // we're in the coolest third
        heatcolor.r = heatramp; // ramp up red
        heatcolor.g = 0; // no green
        heatcolor.b = 0; // no blue
    }

    return heatcolor;
}

////////////////////////////////////////////////////////////////////////////////

typedef uint16_t saccum87;

void hsv_fill_solid_hsv(hsv_t *target, hsv_t color, size_t num)
{
    for (size_t i = 0; i < num; ++i)
        target[i] = color;
}

void rgb_fill_solid_hsv(rgb_t *target, hsv_t color, size_t num)
{
    rgb_t rgb = hsv2rgb_rainbow(color);
    for (size_t i = 0; i < num; ++i)
        target[i] = rgb;
}

void rgb_fill_solid_rgb(rgb_t *target, rgb_t color, size_t num)
{
    for (size_t i = 0; i < num; ++i)
        target[i] = color;
}

void hsv_fill_gradient_hsv(hsv_t *target, size_t startpos, hsv_t startcolor, size_t endpos, hsv_t endcolor,
        color_gradient_direction_t direction)
{
    // if the points are in the wrong order, straighten them
    if (endpos < startpos)
    {
        size_t t = endpos;
        hsv_t tc = endcolor;
        endcolor = startcolor;
        endpos = startpos;
        startpos = t;
        startcolor = tc;
    }

    // If we're fading toward black (val=0) or white (sat=0),
    // then set the endhue to the starthue.
    // This lets us ramp smoothly to black or white, regardless
    // of what 'hue' was set in the endcolor (since it doesn't matter)
    if (endcolor.value == 0 || endcolor.saturation == 0)
        endcolor.hue = startcolor.hue;

    // Similarly, if we're fading in from black (val=0) or white (sat=0)
    // then set the starthue to the endhue.
    // This lets us ramp smoothly up from black or white, regardless
    // of what 'hue' was set in the startcolor (since it doesn't matter)
    if (startcolor.value == 0 || startcolor.saturation == 0)
        startcolor.hue = endcolor.hue;

    saccum87 huedistance87;
    saccum87 satdistance87;
    saccum87 valdistance87;

    satdistance87 = (endcolor.sat - startcolor.sat) << 7;
    valdistance87 = (endcolor.val - startcolor.val) << 7;

    uint8_t huedelta8 = endcolor.hue - startcolor.hue;

    if (direction == COLOR_SHORTEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 > 127)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_LONGEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 < 128)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_FORWARD_HUES)
    {
        huedistance87 = huedelta8 << 7;
    }
    else /* direction == BACKWARD_HUES */
    {
        huedistance87 = (uint8_t) (256 - huedelta8) << 7;
        huedistance87 = -huedistance87;
    }

    size_t pixeldistance = endpos - startpos;
    int16_t divisor = pixeldistance ? pixeldistance : 1;

    saccum87 huedelta87 = huedistance87 / divisor;
    saccum87 satdelta87 = satdistance87 / divisor;
    saccum87 valdelta87 = valdistance87 / divisor;

    huedelta87 *= 2;
    satdelta87 *= 2;
    valdelta87 *= 2;

    accum88 hue88 = startcolor.hue << 8;
    accum88 sat88 = startcolor.sat << 8;
    accum88 val88 = startcolor.val << 8;
    for (size_t i = startpos; i <= endpos; ++i)
    {
        target[i].hue = hue88 >> 8;
        target[i].sat = sat88 >> 8;
        target[i].val = val88 >> 8;
        hue88 += huedelta87;
        sat88 += satdelta87;
        val88 += valdelta87;
    }
}

void rgb_fill_gradient_hsv(rgb_t *target, size_t startpos, hsv_t startcolor, size_t endpos, hsv_t endcolor,
        color_gradient_direction_t direction)
{
    // if the points are in the wrong order, straighten them
    if (endpos < startpos)
    {
        size_t t = endpos;
        hsv_t tc = endcolor;
        endcolor = startcolor;
        endpos = startpos;
        startpos = t;
        startcolor = tc;
    }

    // If we're fading toward black (val=0) or white (sat=0),
    // then set the endhue to the starthue.
    // This lets us ramp smoothly to black or white, regardless
    // of what 'hue' was set in the endcolor (since it doesn't matter)
    if (endcolor.value == 0 || endcolor.saturation == 0)
        endcolor.hue = startcolor.hue;

    // Similarly, if we're fading in from black (val=0) or white (sat=0)
    // then set the starthue to the endhue.
    // This lets us ramp smoothly up from black or white, regardless
    // of what 'hue' was set in the startcolor (since it doesn't matter)
    if (startcolor.value == 0 || startcolor.saturation == 0)
        startcolor.hue = endcolor.hue;

    saccum87 huedistance87;
    saccum87 satdistance87;
    saccum87 valdistance87;

    satdistance87 = (endcolor.sat - startcolor.sat) << 7;
    valdistance87 = (endcolor.val - startcolor.val) << 7;

    uint8_t huedelta8 = endcolor.hue - startcolor.hue;

    if (direction == COLOR_SHORTEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 > 127)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_LONGEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 < 128)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_FORWARD_HUES)
    {
        huedistance87 = huedelta8 << 7;
    }
    else /* direction == BACKWARD_HUES */
    {
        huedistance87 = (uint8_t) (256 - huedelta8) << 7;
        huedistance87 = -huedistance87;
    }

    size_t pixeldistance = endpos - startpos;
    int16_t divisor = pixeldistance ? pixeldistance : 1;

    saccum87 huedelta87 = huedistance87 / divisor;
    saccum87 satdelta87 = satdistance87 / divisor;
    saccum87 valdelta87 = valdistance87 / divisor;

    huedelta87 *= 2;
    satdelta87 *= 2;
    valdelta87 *= 2;

    accum88 hue88 = startcolor.hue << 8;
    accum88 sat88 = startcolor.sat << 8;
    accum88 val88 = startcolor.val << 8;
    for (size_t i = startpos; i <= endpos; ++i)
    {
        target[i] = hsv2rgb_rainbow(hsv_from_values(hue88 >> 8, sat88 >> 8, val88 >> 8));
        hue88 += huedelta87;
        sat88 += satdelta87;
        val88 += valdelta87;
    }
}

void rgb_fill_gradient_rgb(rgb_t *leds, size_t startpos, rgb_t startcolor, size_t endpos, rgb_t endcolor)
{
    // if the points are in the wrong order, straighten them
    if (endpos < startpos)
    {
        size_t t = endpos;
        rgb_t tc = endcolor;
        endcolor = startcolor;
        endpos = startpos;
        startpos = t;
        startcolor = tc;
    }

    saccum87 rdistance87;
    saccum87 gdistance87;
    saccum87 bdistance87;

    rdistance87 = (endcolor.r - startcolor.r) << 7;
    gdistance87 = (endcolor.g - startcolor.g) << 7;
    bdistance87 = (endcolor.b - startcolor.b) << 7;

    size_t pixeldistance = endpos - startpos;
    int16_t divisor = pixeldistance ? pixeldistance : 1;

    saccum87 rdelta87 = rdistance87 / divisor;
    saccum87 gdelta87 = gdistance87 / divisor;
    saccum87 bdelta87 = bdistance87 / divisor;

    rdelta87 *= 2;
    gdelta87 *= 2;
    bdelta87 *= 2;

    accum88 r88 = startcolor.r << 8;
    accum88 g88 = startcolor.g << 8;
    accum88 b88 = startcolor.b << 8;
    for (uint16_t i = startpos; i <= endpos; ++i)
    {
        leds[i].r = r88 >> 8;
        leds[i].g = g88 >> 8;
        leds[i].b = b88 >> 8;
        r88 += rdelta87;
        g88 += gdelta87;
        b88 += bdelta87;
    }
}

////////////////////////////////////////////////////////////////////////////////

hsv_t color_from_palette_hsv(const hsv_t *palette, uint8_t pal_size, uint8_t index, uint8_t brightness, bool blend)
{
    uint8_t div = 256 / pal_size;

    uint8_t hi = index / div;
    uint8_t lo = index % div;

    const hsv_t *entry = palette + hi;

    uint8_t hue1   = entry->hue;
    uint8_t sat1   = entry->sat;
    uint8_t val1   = entry->val;

    if (blend && lo)
    {
        if (hi == pal_size - 1)
            entry = palette;
        else
            ++entry;

        uint8_t f2 = lo * pal_size;
        uint8_t f1 = 255 - f2;

        uint8_t hue2  = entry->hue;
        uint8_t sat2  = entry->sat;
        uint8_t val2  = entry->val;

        // Now some special casing for blending to or from
        // either black or white.  Black and white don't have
        // proper 'hue' of their own, so when ramping from
        // something else to/from black/white, we set the 'hue'
        // of the black/white color to be the same as the hue
        // of the other color, so that you get the expected
        // brightness or saturation ramp, with hue staying
        // constant:

        // If we are starting from white (sat=0)
        // or black (val=0), adopt the target hue.
        if (sat1 == 0 || val1 == 0)
            hue1 = hue2;

        // If we are ending at white (sat=0)
        // or black (val=0), adopt the starting hue.
        if (sat2 == 0 || val2 == 0)
            hue2 = hue1;

        sat1 = scale8(sat1, f1);
        val1 = scale8(val1, f1);

        sat2 = scale8(sat2, f2);
        val2 = scale8(val2, f2);

        // These sums can't overflow, so no qadd8 needed.
        sat1 += sat2;
        val1 += val2;

        uint8_t delta_hue = (uint8_t) (hue2 - hue1);
        if (delta_hue & 0x80)
            // go backwards
            hue1 -= scale8(256 - delta_hue, f2);
        else
            // go forwards
            hue1 += scale8(delta_hue, f2);
    }

    if (brightness != 255)
        val1 = scale8_video(val1, brightness);

    return hsv_from_values(hue1, sat1, val1);
}

#include <stdlib.h>

rgb_t color_from_palette_rgb(const rgb_t *palette, uint8_t pal_size, uint8_t index, uint8_t brightness, bool blend)
{
    uint8_t div = 256 / pal_size;

    uint8_t hi = index / div;
    uint8_t lo = index % div;

    const rgb_t *entry = palette + hi;

    uint8_t red1   = entry->red;
    uint8_t green1 = entry->green;
    uint8_t blue1  = entry->blue;

    if (blend && lo)
    {
        if (hi == pal_size - 1)
            entry = palette;
        else
            ++entry;

        uint8_t f2 = lo * pal_size;
        uint8_t f1 = 255 - f2;

        uint8_t red2 = entry->red;
        red1 = scale8(red1, f1);
        red2 = scale8(red2, f2);
        red1 += red2;

        uint8_t green2 = entry->green;
        green1 = scale8(green1, f1);
        green2 = scale8(green2, f2);
        green1 += green2;

        uint8_t blue2 = entry->blue;
        blue1 = scale8(blue1, f1);
        blue2 = scale8(blue2, f2);
        blue1 += blue2;
    }

    if (brightness != 255)
    {
        if (brightness)
        {
            ++brightness; // adjust for rounding
            // Now, since brightness is nonzero, we don't need the full scale8_video logic;
            // we can just to scale8 and then add one (unless scale8 fixed) to all nonzero inputs.
            if (red1)
                red1 = scale8(red1, brightness);
            if (green1)
                green1 = scale8(green1, brightness);
            if (blue1)
                blue1 = scale8(blue1, brightness);
        }
        else
        {
            red1 = 0;
            green1 = 0;
            blue1 = 0;
        }
    }

    return rgb_from_values(red1, green1, blue1);
}

////////////////////////////////////////////////////////////////////////////////

hsv_t blend(hsv_t existing, hsv_t overlay, fract8 amount, color_gradient_direction_t direction)
{
    if (amount == 0)
        return existing;

    if (amount == 255)
    {
        existing = overlay;
        return existing;
    }

    fract8 amount_of_keep = 255 - amount;
    uint8_t huedelta8 = overlay.hue - existing.hue;

    if (direction == COLOR_SHORTEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 > 127)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_LONGEST_HUES)
    {
        direction = COLOR_FORWARD_HUES;
        if (huedelta8 < 128)
            direction = COLOR_BACKWARD_HUES;
    }

    if (direction == COLOR_FORWARD_HUES)
    {
        existing.hue = existing.hue + scale8(huedelta8, amount);
    }
    else /* direction == BACKWARD_HUES */
    {
        huedelta8 = -huedelta8;
        existing.hue = existing.hue - scale8(huedelta8, amount);
    }

    existing.sat = scale8(existing.sat, amount_of_keep) + scale8(overlay.sat, amount);
    existing.val = scale8(existing.val, amount_of_keep) + scale8(overlay.val, amount);

    return existing;
}

void blur1d(rgb_t *leds, size_t num_leds, fract8 blur_amount)
{
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    rgb_t carryover = rgb_from_code(0);
    for (size_t i = 0; i < num_leds; ++i)
    {
        rgb_t cur = leds[i];
        rgb_t part = cur;
        part = rgb_scale(part, seep);
        cur = rgb_add_rgb(rgb_scale(cur, keep), carryover);
        if (i)
            leds[i - 1] = rgb_add_rgb(leds[i - 1], part);
        leds[i] = cur;
        carryover = part;
    }
}

void blur_columns(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx)
{
    // blur columns
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    for (size_t col = 0; col < width; ++col)
    {
        rgb_t carryover = rgb_from_code(0);
        for (size_t i = 0; i < height; ++i)
        {
            size_t offs = xy(ctx, col, i);
            rgb_t cur = leds[offs];
            rgb_t part = cur;
            part = rgb_scale(part, seep);
            cur = rgb_add_rgb(rgb_scale(cur, keep), carryover);
            if (i)
            {
                size_t prev_offs = xy(ctx, col, i - 1);
                leds[prev_offs] = rgb_add_rgb(leds[prev_offs], part);
            }
            leds[offs] = cur;
            carryover = part;
        }
    }
}

void blur_rows(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx)
{
    // blur rows same as columns, for irregular matrix
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    for (size_t row = 0; row < height; row++)
    {
        rgb_t carryover = rgb_from_code(0);
        for (size_t i = 0; i < width; i++)
        {
            size_t offs = xy(ctx, i, row);
            rgb_t cur = leds[offs];
            rgb_t part = cur;
            part = rgb_scale(part, seep);
            cur = rgb_add_rgb(rgb_scale(cur, keep), carryover);
            if (i)
            {
                size_t prev_offs = xy(ctx, i - 1, row);
                leds[prev_offs] = rgb_add_rgb(leds[prev_offs], part);
            }
            leds[offs] = cur;
            carryover = part;
        }
    }
}

void blur2d(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx)
{
    blur_rows(leds, width, height, blur_amount, xy, ctx);
    blur_columns(leds, width, height, blur_amount, xy, ctx);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t apply_gamma2brightness(uint8_t brightness, float gamma)
{
    float orig = (float)brightness / 255.0;
    float adj = powf(orig, gamma) * 255.0;
    uint8_t result = (uint8_t)adj;
    if (brightness > 0 && !result)
        result = 1; // never gamma-adjust a positive number down to zero
    return result;
}

rgb_t apply_gamma2rgb(rgb_t c, float gamma)
{
    rgb_t res = {
        .r = apply_gamma2brightness(c.r, gamma),
        .g = apply_gamma2brightness(c.g, gamma),
        .b = apply_gamma2brightness(c.b, gamma),
    };
    return res;
}

rgb_t apply_gamma2rgb_channels(rgb_t c, float gamma_r, float gamma_g, float gamma_b)
{
    rgb_t res = {
        .r = apply_gamma2brightness(c.r, gamma_r),
        .g = apply_gamma2brightness(c.g, gamma_g),
        .b = apply_gamma2brightness(c.b, gamma_b),
    };
    return res;
}
