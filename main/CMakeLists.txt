idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES 
                        spi_flash
                        esp_timer
                        SPI_BUS
                        MahonyAHRS
                    INCLUDE_DIRS "")
