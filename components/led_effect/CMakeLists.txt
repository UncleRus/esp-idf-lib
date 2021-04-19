idf_component_register(
    SRCS led_effect.c led_effects/noise1.c led_effects/plasma_waves.c led_effects/rainbow1.c led_effects/waterfall.c
    INCLUDE_DIRS .
    REQUIRES log color lib8tion noise
)
