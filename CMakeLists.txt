target_sources(app PRIVATE src/input_processor_directional_gestures.c)

if(CONFIG_ZMK_SPLIT_ROLE_CENTRAL AND CONFIG_ZMK_STUDIO)
  target_sources(app PRIVATE src/261125_live.c)
  zephyr_link_libraries(-Wl,--wrap=raise_zmk_position_state_changed)
endif()
