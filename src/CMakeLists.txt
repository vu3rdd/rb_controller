add_executable(rb_controller
  rb_controller.c
  mcp23017.c
  radio.c
  encoder.c
  keyer.c
  )

# pull in common dependencies
target_link_libraries(rb_controller pico_stdlib hardware_i2c hardware_pwm hardware_gpio)

# create map/bin/hex file etc.
pico_add_extra_outputs(rb_controller)

# add url via pico_set_program_url
example_auto_set_url(rb_controller)

# enable usb output, disable uart output
pico_enable_stdio_usb(rb_controller 1)
pico_enable_stdio_uart(rb_controller 0)
