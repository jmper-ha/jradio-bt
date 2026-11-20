idf_component_register(SRCS "bt_stack.c" "bt_sdu_buffer.c" INCLUDE_DIRS "include" REQUIRES bt nvs_flash)
# See bt_sdu_buffer.c. -u keeps the wrapper in the link, which nothing else
# names.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=osi_malloc_func"
                      "-u __wrap_osi_malloc_func")
