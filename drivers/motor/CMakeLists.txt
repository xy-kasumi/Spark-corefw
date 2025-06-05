zephyr_sources_ifdef(CONFIG_TMC2209_MOTOR_DRIVER tmc2209.c)
zephyr_sources_ifdef(CONFIG_TMC2209_MOTOR_DRIVER uart1wire.c)

zephyr_include_directories_ifdef(CONFIG_TMC2209_MOTOR_DRIVER ../../include)