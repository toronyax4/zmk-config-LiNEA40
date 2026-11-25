target_sources(app PRIVATE src/input_processor_directional_gestures.c)

if(CONFIG_ZMK_SPLIT_ROLE_CENTRAL AND CONFIG_ZMK_STUDIO)
  target_sources(app PRIVATE src/261125_usb_probe.c)
  zephyr_link_options(-Wl,--wrap=raise_zmk_position_state_changed)
endif()
