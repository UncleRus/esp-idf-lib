#include "color.h"
#include <lib8tion.h>

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
    hsv_fill_gradient_hsv((hsv_t *)target, startpos, startcolor, endpos, endcolor, direction);
    for (size_t i = startpos; i <= endpos; ++i)
        target[i] = hsv2rgb_rainbow(*((hsv_t *)(target + i)));
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

void blur_columns(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy)
{
    // blur columns
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    for (size_t col = 0; col < width; ++col)
    {
        rgb_t carryover = rgb_from_code(0);
        for (size_t i = 0; i < height; ++i)
        {
            size_t offs = xy(col, i);
            rgb_t cur = leds[offs];
            rgb_t part = cur;
            part = rgb_scale(part, seep);
            cur = rgb_add_rgb(rgb_scale(cur, keep), carryover);
            if (i)
            {
                size_t prev_offs = xy(col, i - 1);
                leds[prev_offs] = rgb_add_rgb(leds[prev_offs], part);
            }
            leds[offs] = cur;
            carryover = part;
        }
    }
}

void blur_rows(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy)
{
    // blur rows same as columns, for irregular matrix
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    for (size_t row = 0; row < height; row++)
    {
        rgb_t carryover = rgb_from_code(0);
        for (size_t i = 0; i < width; i++)
        {
            size_t offs = xy(i, row);
            rgb_t cur = leds[offs];
            rgb_t part = cur;
            part = rgb_scale(part, seep);
            cur = rgb_add_rgb(rgb_scale(cur, keep), carryover);
            if (i)
            {
                size_t prev_offs = xy(i - 1, row);
                leds[prev_offs] = rgb_add_rgb(leds[prev_offs], part);
            }
            leds[offs] = cur;
            carryover = part;
        }
    }
}

void blur2d(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy)
{
    blur_rows(leds, width, height, blur_amount, xy);
    blur_columns(leds, width, height, blur_amount, xy);
}
