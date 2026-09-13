idf_component_register(SRCS "audio_in.c" INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2s esp_driver_gpio esp_ringbuf esp_timer)
