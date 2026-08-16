idf_component_register(SRCS "main.c"
                    REQUIRES wifi web nvs_flash motor encoder lidar
                    INCLUDE_DIRS ".")
