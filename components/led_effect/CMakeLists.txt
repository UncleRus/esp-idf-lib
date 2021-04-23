idf_component_register(
    SRCS led_effect.c 
    	 led_animation.c
    	 led_effects/noise1.c
    	 led_effects/plasma_waves.c
    	 led_effects/rainbow1.c
    	 led_effects/waterfall.c
    	 led_effects/dna.c
    	 led_effects/rays.c
    	 led_effects/crazybees.c
    INCLUDE_DIRS .
    REQUIRES log color lib8tion noise
)
