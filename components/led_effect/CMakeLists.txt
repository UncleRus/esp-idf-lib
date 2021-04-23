idf_component_register(
    SRCS noise1.c
         plasma_waves.c
         rainbow1.c
         waterfall.c
         dna.c
         rays.c
         crazybees.c
         sparkles1.c
         matrix.c
         rain.c
    INCLUDE_DIRS .
    REQUIRES framebuffer color lib8tion noise
)
