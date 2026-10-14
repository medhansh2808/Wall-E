idf_component_register(
    SRCS "main.c" "sac_perception.c" "sac_controller.c" "sac_navigation.c" "sac_drive.c" "sac_obstacle.c"
    INCLUDE_DIRS "."
    REQUIRES sra-board-component esp_timer
)
