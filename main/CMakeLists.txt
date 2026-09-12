idf_component_register(SRCS "main.c" "module_state.c" INCLUDE_DIRS "."
                       REQUIRES jbt_proto jbt_link bt_stack a2dp_sink audio_out nvs_flash)
